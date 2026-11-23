#include "diagnostics.hpp"
#include "wall_clock.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <ctime>
#include <deque>
#include <iomanip>
#include <mutex>
#include <optional>
#include <random>
#include <regex>
#include <sstream>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <fcntl.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sys/stat.h>
#include <unistd.h>

namespace web_control
{
namespace
{
using Json = nlohmann::json;
using Steady = std::chrono::steady_clock;
using Seconds = std::chrono::duration<double>;
using Imu = sensor_msgs::msg::Imu;
using Odom = nav_msgs::msg::Odometry;
using DriverStats = diagnostic_msgs::msg::DiagnosticArray;
namespace fs = std::filesystem;

enum TopicIndex {imu, lidar, odom, topic_count};

template<typename T>
T param(rclcpp::Node & node, const char * name, const T & value)
{
  return node.has_parameter(name) ? node.get_parameter(name).get_value<T>() :
         node.declare_parameter<T>(name, value);
}

double monotonic()
{
  return Seconds(Steady::now().time_since_epoch()).count();
}

std::string format_time(double stamp, const char * format, bool utc = false)
{
  const auto seconds = static_cast<std::time_t>(stamp);
  std::tm parts{};
  if (utc) {
    gmtime_r(&seconds, &parts);
  } else {
    localtime_r(&seconds, &parts);
  }
  std::ostringstream text;
  text << std::put_time(&parts, format);
  return text.str();
}

std::string iso(double stamp)
{
  std::ostringstream text;
  text << format_time(stamp, "%Y-%m-%dT%H:%M:%S", true) << "." <<
    std::setw(3) << std::setfill('0') << static_cast<int>((stamp - std::floor(stamp)) * 1000) <<
    "+00:00";
  return text.str();
}

template<typename T>
Json value_or_null(const std::optional<T> & value)
{
  return value ? Json(*value) : Json(nullptr);
}

Json finite(double value)
{
  return std::isfinite(value) ? Json(value) : Json(nullptr);
}

double rounded(double value, double scale = 1000)
{
  return std::round(value * scale) / scale;
}

std::string trimmed(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  return first == std::string::npos ? std::string{} :
    value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
}

std::string random_hex(std::size_t digits)
{
  thread_local std::mt19937_64 random(std::random_device{}());
  constexpr char symbols[] = "0123456789abcdef";
  std::string result;
  while (result.size() < digits) {
    const auto word = random();
    for (std::size_t offset = 0; offset < 64 && result.size() < digits; offset += 4) {
      result += symbols[(word >> offset) & 15];
    }
  }
  return result;
}

struct Presence
{
  std::optional<std::size_t> publishers;
  std::optional<double> checked;

  Json checked_age(double now) const
  {
    return checked ? Json(std::max(0.0, now - *checked)) : Json(nullptr);
  }
};

struct Topic
{
  const char * key;
  const char * label;
  std::string name;
  const char * type;
  bool receive;
  Presence presence;
  std::deque<double> arrivals;
  std::optional<double> last;
  std::uint64_t total = 0;

  void record(double now)
  {
    if (arrivals.size() == 8192) {
      arrivals.pop_front();
    }
    arrivals.push_back(now);
    last = now;
    ++total;
  }

  Json row(double now, double window, double stale, bool enabled)
  {
    while (!arrivals.empty() && arrivals.front() < now - window) {
      arrivals.pop_front();
    }
    const auto age = last ? std::optional<double>(std::max(0.0, now - *last)) : std::nullopt;
    const auto status = !enabled ? "unavailable" : presence.publishers == 0 ? "no_publisher" :
      !receive ? (presence.publishers ? "publisher_only" : "unknown") :
      !last ? "waiting" : *age > stale ? "stale" : "active";
    const auto span = arrivals.size() > 1 ? arrivals.back() - arrivals.front() : 0.0;
    return {
      {"key", key}, {"label", label}, {"topic", name}, {"message_type", type},
      {"enabled", enabled}, {"available", true}, {"receive_enabled", receive},
      {"status", status}, {"publisher_count", value_or_null(presence.publishers)},
      {"publisher_checked_age_sec", presence.checked_age(now)},
      {"hz", receive ? Json(span > 0 ? rounded((arrivals.size() - 1) / span) : 0.0) : Json(nullptr)},
      {"window_sec", window}, {"received_count", total}, {"window_count", arrivals.size()},
      {"last_age_sec", age ? Json(rounded(*age)) : Json(nullptr)},
      {"last_received_stamp", value_or_null(last)}, {"error", nullptr}
    };
  }
};

struct Report
{
  double publish_hz;
  double window;
  double last_cloud_age;
  std::uint64_t window_count;
  std::uint64_t total;
  std::uint64_t sequence;
  std::string session;
  std::string state;
};

std::optional<Report> parse_report(const DriverStats & message, const std::string & source)
{
  const auto status = std::find_if(message.status.begin(), message.status.end(), [](const auto & row) {
    return row.name == "hesai_lidar_publish";
  });
  if (status == message.status.end() ||
    (!status->hardware_id.empty() && status->hardware_id != source))
  {
    return std::nullopt;
  }
  std::unordered_map<std::string, std::string> values;
  for (const auto & field : status->values) {
    values[field.key] = trimmed(field.value);
  }
  if (source.empty() || values["source_topic"] != source) {
    return std::nullopt;
  }
  auto number = [&](const char * key) {
      const auto & text = values.at(key);
      std::size_t end;
      const auto result = std::stod(text, &end);
      if (end != text.size() || !std::isfinite(result)) {
        throw std::invalid_argument("non-finite driver statistic");
      }
      return result;
    };
  auto integer = [&](const char * key) {
      const auto & text = values.at(key);
      std::uint64_t result = 0;
      const auto converted = std::from_chars(text.data(), text.data() + text.size(), result);
      if (converted.ec != std::errc{} || converted.ptr != text.data() + text.size()) {
        throw std::invalid_argument("invalid driver counter");
      }
      return result;
    };
  try {
    Report report{
      number("publish_rate_hz"), number("window_sec"), number("last_publish_age_sec"),
      integer("frames_in_window"), integer("total_frames"), integer("sequence"),
      values["session_id"], values["state"]};
    if (report.publish_hz < 0 || report.publish_hz > 1000 || report.window <= 0 ||
      report.window_count > report.total ||
      (report.last_cloud_age < 0 && (report.last_cloud_age != -1 || report.total != 0)) ||
      report.session.empty() || report.session.size() > 128)
    {
      return std::nullopt;
    }
    if (report.state.empty()) {
      report.state = report.total == 0 ? "waiting" : report.window_count == 0 ? "idle" : "publishing";
    }
    if (report.state != "waiting" && report.state != "idle" && report.state != "publishing") {
      return std::nullopt;
    }
    return report;
  } catch (const std::logic_error &) {
    return std::nullopt;
  }
}

struct LidarMonitor
{
  Presence presence;
  std::optional<Report> report;
  std::optional<double> arrival;
  std::uint64_t accepted = 0;
  std::uint64_t invalid = 0;
  std::uint64_t duplicates = 0;
  std::uint64_t session_changes = 0;

  void record(std::optional<Report> next, double now)
  {
    if (!next) {
      ++invalid;
    } else if (report && report->session == next->session && report->sequence >= next->sequence) {
      ++duplicates;
    } else {
      if (report && report->session != next->session) {
        ++session_changes;
      }
      report = std::move(next);
      arrival = now;
      ++accepted;
    }
  }

  Json snapshot(
    double now, double timeout, bool enabled, const std::string & topic,
    const std::string & source) const
  {
    const auto age = arrival ? std::max(0.0, now - *arrival) : 0.0;
    const bool fresh = enabled && report && age <= timeout;
    const auto status = !enabled ? "unavailable" : report ? (fresh ? "fresh" : "stale") :
      !presence.publishers ? "unknown" : *presence.publishers == 0 ? "no_publisher" : "waiting";
    const auto cloud_age = fresh && report->last_cloud_age >= 0 ?
      Json(rounded(report->last_cloud_age + age)) : Json(nullptr);
    return {
      {"status", status}, {"enabled", enabled}, {"available", true}, {"stats_topic", topic},
      {"source_topic", source}, {"hz", fresh ? Json(rounded(report->publish_hz)) : Json(nullptr)},
      {"hz_source", fresh ? Json("driver_publish") : Json(nullptr)},
      {"last_publish_hz", report ? Json(rounded(report->publish_hz)) : Json(nullptr)},
      {"window_sec", report ? Json(report->window) : Json(nullptr)},
      {"window_count", report ? Json(report->window_count) : Json(nullptr)},
      {"total_frames", report ? Json(report->total) : Json(nullptr)}, {"last_publish_age_sec", cloud_age},
      {"cloud_age_sec", cloud_age}, {"stats_age_sec", arrival ? Json(rounded(age)) : Json(nullptr)},
      {"state", report ? Json(report->state) : Json(nullptr)},
      {"session_id", report ? Json(report->session) : Json(nullptr)},
      {"sequence", report ? Json(report->sequence) : Json(nullptr)},
      {"publisher_count", value_or_null(presence.publishers)}, {"publisher_checked_age_sec", presence.checked_age(now)},
      {"accepted_count", accepted}, {"invalid_count", invalid},
      {"duplicate_count", duplicates}, {"session_changes", session_changes}
    };
  }
};

void merge_lidar(Json & row, const Json & stats)
{
  row["driver_stats"] = stats;
  for (const auto * field : {"cloud_age_sec", "stats_age_sec", "last_publish_hz"}) {
    row[field] = stats.at(field);
  }
  row["stats_status"] = stats["status"];
  row["received_hz"] = row["receive_enabled"] == true ? row["hz"] : Json(nullptr);
  row["received_hz_source"] = row["received_hz"].is_null() ? Json(nullptr) : Json("received");
  for (const auto * field : {
      "window_sec", "window_count", "total_frames", "stats_age_sec", "session_id",
      "sequence", "state", "last_publish_hz"})
  {
    row[std::string("driver_") + field] = stats.at(field);
  }
  row["driver_cloud_age_sec"] = stats["last_publish_age_sec"];
  row["publish_hz"] = stats["hz"];
  row["driver_publish_hz"] = stats["hz"];
  row["publish_hz_source"] = stats["hz_source"];
  if (stats["status"] == "fresh") {
    row["hz"] = stats["hz"];
    row["hz_source"] = "driver_publish";
    row["status"] = stats["hz"].get<double>() <= 0 ? "idle" : "active";
    row["last_age_sec"] = stats["last_publish_age_sec"];
  } else if (stats["status"] == "stale") {
    row["hz"] = nullptr;
    row["hz_source"] = nullptr;
    row["status"] = "stats_stale";
    row["last_age_sec"] = nullptr;
  } else {
    row["hz_source"] = row["received_hz_source"];
  }
  row["rate_source"] = row["hz_source"];
}

class Motion
{
public:
  Motion(rclcpp::Node & node, std::string source, double timeout)
  : source_(std::move(source)), timeout_(timeout),
    start_(std::max(0.0, param(node, "motion_start_threshold_mps", 0.05))),
    stop_(std::clamp(param(node, "motion_stop_threshold_mps", 0.02), 0.0, start_)),
    debounce_(std::max(0.0, param(node, "motion_debounce_sec", 0.3))),
    max_gap_(std::max(0.001, param(node, "motion_max_gap_sec", 0.5))),
    max_speed_(std::max(0.001, param(node, "motion_max_speed_mps", 3.0)))
  {
  }

  void record(const Odom & message, double now, double at)
  {
    const auto speed = message.twist.twist.linear.x;
    if (!std::isfinite(speed)) {
      ++invalid_;
      return;
    }
    const auto stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9;
    const std::pair frames{message.header.frame_id, message.child_frame_id};
    if (last_arrival_ && now - *last_arrival_ > timeout_ && state_ == "moving") {
      interrupt("odom_stale", at, stamp);
    }
    last_arrival_ = now;
    if (frames_ && *frames_ != frames) {
      interrupt("frame_changed", at, stamp);
    }
    frames_ = frames;
    if (std::abs(speed) > max_speed_) {
      interrupt("implausible_speed", at, stamp);
      ++invalid_;
      return;
    }
    if (previous_stamp_ && stamp <= *previous_stamp_) {
      interrupt("timestamp_reset", at, stamp);
    } else if (previous_stamp_ && stamp - *previous_stamp_ > max_gap_) {
      interrupt("timestamp_gap", at, stamp);
    }
    if (!total_) {
      total_ = 0;
      incomplete_ = false;
      reason_.reset();
    }
    if (previous_stamp_ && previous_speed_ && (state_ == "moving" || std::abs(speed) >= start_)) {
      const auto previous = *previous_speed_;
      const auto magnitude = std::abs(previous) + std::abs(speed);
      const auto dt = stamp - *previous_stamp_;
      // Integrating absolute speed across a reversal requires splitting the
      // interval at the zero crossing instead of averaging signed speeds.
      const auto distance = previous * speed >= 0 ? 0.5 * dt * magnitude :
        dt * (previous * previous + speed * speed) / (2 * magnitude);
      if (std::isfinite(distance) && distance >= 0) {
        *total_ += distance;
        if (!segment_start_) {
          segment_start_ = *previous_stamp_;
        }
        segment_distance_ += distance;
      }
    }
    const auto & q = message.pose.pose.orientation;
    const auto norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    yaw_ = std::isfinite(norm) && norm > 0 ?
      std::optional<double>(std::atan2(
        2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))) :
      std::nullopt;
    previous_speed_ = speed;
    last_speed_ = speed;
    previous_stamp_ = stamp;
    last_stamp_ = stamp;
    last_wall_ = at;
    paused_ = false;
    transition(speed, stamp, at);
  }

  void lifecycle(bool started)
  {
    const std::string reason = started ? "chassis_start" : "chassis_stop";
    const auto stamp = last_stamp_.value_or(0);
    const auto at = wall_time();
    if (state_ == "moving") {
      interrupt(reason, at, stamp);
    } else {
      emit("motion_lifecycle", "unknown", reason, at, stamp, segment_distance_, last_duration_);
      state_ = "unknown";
      reason_ = reason;
      paused_ = true;
      incomplete_ = incomplete_ || (!started && total_.has_value());
      reset_baseline();
    }
    last_arrival_.reset();
  }

  Json snapshot(double now)
  {
    const auto age = last_arrival_ ? std::optional<double>(std::max(0.0, now - *last_arrival_)) :
      std::nullopt;
    const bool stale = age && *age > timeout_;
    if (stale && state_ == "moving") {
      interrupt("odom_stale", last_wall_.value_or(wall_time()), last_stamp_.value_or(0));
    }
    auto valid = validity();
    const auto state = stale ? "unknown" : state_;
    if (stale) {
      incomplete_ = true;
      if (!reason_) {
        reason_ = "odom_stale";
      }
      valid["valid"] = false;
      valid["paused"] = true;
      valid["incomplete"] = true;
      valid["reason"] = "odom_stale";
    }
    const auto duration = state == "moving" && segment_start_ && last_stamp_ ?
      std::max(0.0, *last_stamp_ - *segment_start_) : last_duration_;
    return {
      {"source", "wheel_odometry"}, {"source_topic", source_}, {"topic", source_},
      {"state", state}, {"motion", state}, {"speed_mps", value_or_null(last_speed_)},
      {"yaw_rad", value_or_null(yaw_)}, {"direction", direction()},
      {"segment_distance_m", rounded(state == "moving" ? segment_distance_ : last_distance_, 1e6)},
      {"segment_duration_sec", rounded(duration)},
      {"total_distance_m", total_ ? Json(rounded(*total_, 1e6)) : Json(nullptr)},
      {"distance_estimate", "wheel_estimated"},
      {"distance_label", "wheel estimate (current web session)"},
      {"validity", valid}, {"received", last_arrival_.has_value()},
      {"last_age_sec", age ? Json(rounded(*age)) : Json(nullptr)}, {"stale", stale},
      {"paused", stale || paused_}, {"incomplete", incomplete_},
      {"reason", stale ? Json("odom_stale") : value_or_null(reason_)},
      {"last_sample_stamp", value_or_null(last_stamp_)}
    };
  }

  std::deque<Json> take_events()
  {
    return std::exchange(events_, {});
  }

private:
  void transition(double speed, double stamp, double at)
  {
    const auto magnitude = std::abs(speed);
    const bool moving = state_ == "moving";
    const bool crossing = moving ? magnitude <= stop_ : magnitude >= start_;
    if (!crossing) {
      candidate_.reset();
      if (!moving && magnitude <= stop_) {
        state_ = "stationary";
      }
      return;
    }
    if (!candidate_) {
      candidate_ = stamp;
    }
    if (stamp - *candidate_ < debounce_ - 1e-9) {
      return;
    }
    candidate_.reset();
    if (moving) {
      last_distance_ = segment_distance_;
      last_duration_ = duration(stamp);
      emit("motion_stop", "stationary", std::nullopt, at, stamp, last_distance_, last_duration_);
      state_ = "stationary";
      reset_baseline();
    } else {
      state_ = "moving";
      if (!segment_start_) {
        segment_start_ = stamp;
      }
      emit("motion_start", "moving", std::nullopt, at, stamp, segment_distance_, duration(stamp));
    }
  }

  double duration(double stamp) const
  {
    return std::max(0.0, stamp - segment_start_.value_or(stamp));
  }

  void reset_baseline()
  {
    previous_stamp_.reset();
    previous_speed_.reset();
    segment_start_.reset();
    candidate_.reset();
    segment_distance_ = 0;
  }

  void interrupt(const std::string & reason, double at, double stamp)
  {
    if (state_ == "moving") {
      last_distance_ = segment_distance_;
      last_duration_ = duration(last_stamp_.value_or(stamp));
      emit("motion_interruption", "unknown", reason, at, stamp, last_distance_, last_duration_);
    }
    state_ = "unknown";
    paused_ = true;
    incomplete_ = true;
    reason_ = reason;
    reset_baseline();
  }

  Json validity() const
  {
    return {
      {"available", total_.has_value()}, {"valid", total_.has_value() && !paused_},
      {"paused", paused_}, {"incomplete", incomplete_},
      {"reason", value_or_null(reason_)}, {"invalid_samples", invalid_}
    };
  }

  Json direction() const
  {
    return last_speed_ && std::abs(*last_speed_) > stop_ ?
      Json(*last_speed_ < 0 ? "reverse" : "forward") : Json(nullptr);
  }

  void emit(
    const char * event, const char * state, const std::optional<std::string> & reason,
    double at, double stamp, double distance, double seconds)
  {
    if (events_.size() == 512) {
      events_.pop_front();
    }
    events_.push_back({
      {"kind", "motion"}, {"event", event}, {"at", at}, {"at_iso", iso(at)},
      {"source", source_}, {"topic", source_}, {"sample_stamp", stamp}, {"state", state},
      {"motion_state", state}, {"reason", value_or_null(reason)}, {"speed_mps", value_or_null(last_speed_)},
      {"direction", direction()}, {"yaw_rad", value_or_null(yaw_)},
      {"segment_distance_m", rounded(distance, 1e6)}, {"segment_duration_sec", rounded(seconds)},
      {"total_distance_m", value_or_null(total_)}, {"distance_source", "wheel_odometry"},
      {"distance_validity", validity()}
    });
  }

  const std::string source_;
  const double timeout_;
  const double start_;
  const double stop_;
  const double debounce_;
  const double max_gap_;
  const double max_speed_;
  std::string state_ = "unknown";
  std::optional<std::string> reason_ = "no_odom";
  std::optional<std::pair<std::string, std::string>> frames_;
  std::optional<double> total_;
  std::optional<double> previous_stamp_;
  std::optional<double> previous_speed_;
  std::optional<double> last_arrival_;
  std::optional<double> last_stamp_;
  std::optional<double> last_speed_;
  std::optional<double> last_wall_;
  std::optional<double> yaw_;
  std::optional<double> segment_start_;
  std::optional<double> candidate_;
  double segment_distance_ = 0;
  double last_distance_ = 0;
  double last_duration_ = 0;
  bool paused_ = false;
  bool incomplete_ = false;
  std::uint64_t invalid_ = 0;
  std::deque<Json> events_;
};

class DiagnosticsLog
{
public:
  DiagnosticsLog(rclcpp::Node & node, const fs::path & root, bool enabled)
  : logger_(node.get_logger()), root_(root / "diagnostics"), id_(random_hex(32)), enabled_(enabled),
    max_bytes_(std::max<std::int64_t>(1024, param(node, "diagnostics_max_log_bytes", std::int64_t(10485760)))),
    retain_days_(std::max<std::int64_t>(1, param(node, "diagnostics_retain_days", std::int64_t(7)))),
    retain_bytes_(std::max<std::int64_t>(1024, param(node, "diagnostics_retain_bytes", std::int64_t(104857600))))
  {
    try {
      open(wall_time());
      prune();
    } catch (const std::exception & problem) {
      fail(problem);
    }
  }

  ~DiagnosticsLog()
  {
    close();
  }

  Json snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return {
      {"logging", {
        {"session_id", id_}, {"active_session", file_ >= 0 ? Json(session_) : Json(nullptr)},
        {"active_bytes", file_ >= 0 ? bytes_ : 0}, {"error", value_or_null(error_)},
        {"last_entry_at", value_or_null(last_entry_)}, {"rotations", rotations_},
        {"max_bytes", max_bytes_}, {"retain_days", retain_days_}, {"retain_bytes", retain_bytes_}
      }},
      {"topic_logging_enabled", enabled_}
    };
  }

  Json session() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return {
      {"session", file_ >= 0 ? Json(session_) : Json(nullptr)},
      {"started_at", file_ >= 0 ? Json(started_) : Json(nullptr)},
      {"bytes", file_ >= 0 ? bytes_ : 0}, {"closed", file_ < 0},
      {"error", value_or_null(error_)}, {"persist_error", value_or_null(error_)},
      {"path", file_ >= 0 ? Json((root_ / (session_ + ".log")).string()) : Json(nullptr)}
    };
  }

  void set_enabled(bool enabled)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (enabled == enabled_ || closed_) {
      return;
    }
    enabled_ = enabled;
    write_unlocked({
      {"kind", "logging_toggle"}, {"event", "topic_logging"}, {"enabled", enabled_},
      {"topic_logging_enabled", enabled_},
      {"message", enabled_ ? "topic/IMU diagnostics log writing enabled" :
        "topic/IMU diagnostics log writing disabled"}
    });
  }

  void write(Json row)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    write_unlocked(std::move(row));
  }

  void shutdown(Json row)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    write_unlocked(std::move(row));
    closed_ = true;
    close();
  }

private:
  void write_unlocked(Json row)
  {
    if (closed_ || (!enabled_ && row["kind"] == "topic_status")) {
      return;
    }
    if (!enabled_) {
      for (const auto * key : {
          "topics", "imu_observation", "topic_hz", "topic_age_sec", "publisher_count",
          "topic_status", "status", "lidar_receive_enabled", "lidar_rate_source",
          "lidar_publish_stats", "lidar_stats_topic", "lidar_stats_stale_timeout_sec",
          "lidar_stats_available"})
      {
        row.erase(key);
      }
    }
    const auto at = wall_time();
    if (!row.contains("at")) {
      row["at"] = at;
      row["at_iso"] = iso(at);
    }
    row["session_id"] = id_;
    if (!row.contains("source")) {
      row["source"] = "diagnostics";
    }
    try {
      if (file_ < 0) {
        open(at);
      }
      if (bytes_ >= max_bytes_ || format_time(at, "%Y%m%d") != day_) {
        close();
        open(at);
        ++rotations_;
        prune();
      }
      write_bytes(row.dump(-1, ' ', false, Json::error_handler_t::replace) + "\n");
      last_entry_ = at;
      error_.reset();
    } catch (const std::exception & problem) {
      fail(problem);
    }
  }

  void write_bytes(const std::string & text)
  {
    std::size_t offset = 0;
    while (offset < text.size()) {
      const auto size = ::write(file_, text.data() + offset, text.size() - offset);
      if (size < 0 && errno == EINTR) {
        continue;
      }
      if (size <= 0) {
        throw std::system_error(size == 0 ? EIO : errno, std::generic_category(), "diagnostics log write");
      }
      offset += size;
      bytes_ += size;
    }
  }

  void open(double at)
  {
    fs::create_directories(root_);
    day_ = format_time(at, "%Y%m%d");
    session_ = format_time(at, "%Y%m%d-%H%M%S") + "-" + random_hex(12);
    file_ = ::open((root_ / (session_ + ".log")).c_str(),
      O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0644);
    if (file_ < 0) {
      throw std::system_error(errno, std::generic_category(), "diagnostics log open");
    }
    started_ = at;
    bytes_ = 0;
    write_bytes("# managed log target=diagnostics session=" + session_ + " started=" +
      format_time(at, "%Y-%m-%dT%H:%M:%S") + "\n$ diagnostics jsonl node=" + id_ + "\n");
    const Json header{{"kind", "log_start"}, {"session_id", id_}, {"at", at}, {"at_iso", iso(at)}};
    write_bytes(header.dump() + "\n");
  }

  void prune() const
  {
    struct Completed
    {
      time_t stamp;
      std::uintmax_t size;
      fs::path path;
    };
    static const std::regex session_pattern("[0-9]{8}-[0-9]{6}-[a-f0-9]{8,32}\\.log");
    std::vector<Completed> completed;
    std::uintmax_t bytes = 0;
    std::error_code error;
    for (fs::directory_iterator it(root_, error), end; it != end && !error; it.increment(error)) {
      const auto name = it->path().filename().string();
      struct stat stat{};
      if (name != session_ + ".log" && std::regex_match(name, session_pattern) &&
        lstat(it->path().c_str(), &stat) == 0 && S_ISREG(stat.st_mode) && stat.st_size >= 0)
      {
        const auto size = static_cast<std::uintmax_t>(stat.st_size);
        completed.push_back({stat.st_mtime, size, it->path()});
        bytes += size;
      }
    }
    std::sort(completed.begin(), completed.end(), [](const auto & a, const auto & b) {
      return a.stamp < b.stamp;
    });
    const double cutoff = wall_time() - static_cast<double>(retain_days_) * 86400;
    for (const auto & entry : completed) {
      if (entry.stamp >= cutoff && bytes <= retain_bytes_) {
        break;
      }
      std::error_code remove_error;
      if (fs::remove(entry.path, remove_error)) {
        bytes -= entry.size;
      }
    }
  }

  void close()
  {
    if (file_ >= 0) {
      ::close(file_);
      file_ = -1;
    }
  }

  void fail(const std::exception & problem)
  {
    if (error_ != problem.what()) {
      RCLCPP_ERROR(logger_, "diagnostics log write failed: %s", problem.what());
    }
    error_ = problem.what();
    close();
  }

  const rclcpp::Logger logger_;
  const fs::path root_;
  const std::string id_;
  mutable std::mutex mutex_;
  bool enabled_;
  const std::uintmax_t max_bytes_;
  const std::int64_t retain_days_;
  const std::uintmax_t retain_bytes_;
  int file_ = -1;
  std::uintmax_t bytes_ = 0;
  std::uint64_t rotations_ = 0;
  std::string session_;
  std::string day_;
  double started_ = 0;
  std::optional<double> last_entry_;
  std::optional<std::string> error_;
  bool closed_ = false;
};
}  // namespace

class Diagnostics::Impl
{
public:
  Impl(rclcpp::Node & node, const fs::path & log_directory, bool logging_enabled)
  : node_(node), log_directory_(log_directory),
    enabled_(param(node, "diagnostics_enabled", true)),
    lidar_receive_(param(node, "diagnostics_lidar_receive_enabled", false)),
    window_(std::max(0.1, param(node, "diagnostics_window_sec", 5.0))),
    stale_(std::max(0.1, param(node, "diagnostics_stale_timeout_sec", 2.0))),
    lidar_stale_(std::max(0.5, param(node, "diagnostics_lidar_stats_stale_timeout_sec", 3.0))),
    sample_period_(std::max(0.1, param(node, "diagnostics_sample_period_sec", 1.0))),
    snapshot_period_(std::max(1.0, param(node, "diagnostics_snapshot_period_sec", 5.0))),
    publisher_refresh_(std::max(0.5, param(node, "diagnostics_publisher_refresh_sec", 1.0))),
    stats_topic_(param(node, "diagnostics_lidar_stats_topic", std::string("/lidar_points/statistics"))),
    topics_{{
        {"imu", "IMU", param(node, "diagnostics_imu_topic", std::string("/imu")),
          "sensor_msgs/msg/Imu", true, {}, {}, std::nullopt, 0},
        {"lidar", "Raw LiDAR", param(node, "diagnostics_lidar_topic", std::string("/lidar_points")),
          "sensor_msgs/msg/PointCloud2", lidar_receive_, {}, {}, std::nullopt, 0},
        {"odom", "Chassis Odom", param(node, "diagnostics_chassis_odom_topic", std::string("/odom1")),
          "nav_msgs/msg/Odometry", true, {}, {}, std::nullopt, 0}
      }},
    motion_(node, topics_[odom].name, stale_), log_(node, log_directory, logging_enabled)
  {
    if (!enabled_) {
      return;
    }
    const auto qos = rclcpp::QoS(10).best_effort().durability_volatile();
    imu_subscription_ = node.create_subscription<Imu>(topics_[imu].name, qos, [this](const Imu & msg) {
      const auto now = monotonic();
      std::lock_guard<std::mutex> lock(state_mutex_);
      auto & topic = topics_[imu];
      if (topic.last && now - *topic.last >= 0.5 && now - last_imu_warning_ >= 1.0) {
        RCLCPP_WARN(node_.get_logger(), "IMU callback gap %.2fs", now - *topic.last);
        last_imu_warning_ = now;
      }
      topic.record(now);
      imu_observation_ = ImuSample{msg.linear_acceleration, msg.angular_velocity, wall_time()};
    });
    odom_subscription_ = node.create_subscription<Odom>(topics_[odom].name, qos, [this](const Odom & msg) {
      const auto now = monotonic();
      const auto at = wall_time();
      std::lock_guard<std::mutex> lock(state_mutex_);
      topics_[odom].record(now);
      motion_.record(msg, now, at);
    });
    const auto qos_one = rclcpp::QoS(1).best_effort().durability_volatile();
    stats_subscription_ = node.create_subscription<DriverStats>(stats_topic_, qos_one,
      [this](const DriverStats & msg) {
        auto report = parse_report(msg, topics_[lidar].name);
        std::lock_guard<std::mutex> lock(state_mutex_);
        lidar_.record(std::move(report), monotonic());
      });
    if (lidar_receive_) {
      raw_subscription_ = node.create_generic_subscription(
        topics_[lidar].name, "sensor_msgs/msg/PointCloud2", qos_one,
        [this](std::shared_ptr<rclcpp::SerializedMessage>) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          topics_[lidar].record(monotonic());
        });
    }
    worker_ = std::thread([this]() {run();});
  }

  Json snapshot() const
  {
    Json state;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state = snapshot_locked(monotonic());
    }
    state.update(log_.snapshot());
    return state;
  }

  Json session() const
  {
    return log_.session();
  }

  void set_logging(bool enabled)
  {
    log_.set_enabled(enabled);
  }

  void chassis_lifecycle(bool started)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    motion_.lifecycle(started);
  }

  void shutdown()
  {
    std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
    {
      std::lock_guard<std::mutex> lock(wake_mutex_);
      if (stopping_) {
        return;
      }
      stopping_ = true;
    }
    wake_.notify_one();
    if (worker_.joinable()) {
      worker_.join();
    }
    Json state;
    std::deque<Json> events;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state = snapshot_locked(monotonic());
      events = motion_.take_events();
    }
    state.update(log_.snapshot());
    write_motion_events(events, state);
    state["kind"] = "shutdown";
    for (const auto * key : {
        "speed_mps", "direction", "segment_distance_m", "segment_duration_sec", "total_distance_m",
        "validity"})
    {
      state[key] = state.at("motion").at(key);
    }
    log_.shutdown(std::move(state));
  }

private:
  struct ImuSample
  {
    geometry_msgs::msg::Vector3 acceleration;
    geometry_msgs::msg::Vector3 angular_velocity;
    double at;
  };

  std::optional<std::size_t> count(const std::string & topic) const
  {
    try {
      return node_.count_publishers(topic);
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }

  Json snapshot_locked(double now) const
  {
    Json rows = Json::array();
    for (auto & topic : topics_) {
      rows.push_back(topic.row(now, window_, stale_, enabled_));
    }
    auto stats = lidar_.snapshot(now, lidar_stale_, enabled_, stats_topic_, topics_[lidar].name);
    merge_lidar(rows[lidar], stats);
    const auto rate_source = rows[lidar]["rate_source"];
    Json observation = nullptr;
    if (imu_observation_) {
      const auto vector = [](const auto & value) {
          return Json{{"x", finite(value.x)}, {"y", finite(value.y)}, {"z", finite(value.z)}};
        };
      observation = {
        {"at", imu_observation_->at}, {"at_iso", iso(imu_observation_->at)},
        {"acceleration_mps2", vector(imu_observation_->acceleration)},
        {"angular_velocity_rps", vector(imu_observation_->angular_velocity)},
        {"motion_proof", false}, {"note", "auxiliary observation; IMU alone is not motion proof"},
        {"status", rows[imu]["status"]}, {"last_age_sec", rows[imu]["last_age_sec"]}
      };
    }
    return {
      {"ok", true}, {"source_mode", "wheel_odometry_primary_imu_auxiliary"},
      {"lidar_rate_source", rate_source}, {"lidar_publish_stats", std::move(stats)},
      {"topics", std::move(rows)}, {"imu_observation", std::move(observation)},
      {"motion", motion_.snapshot(now)}, {"stamp", wall_time()}, {"enabled", enabled_},
      {"lidar_receive_enabled", lidar_receive_}, {"lidar_stats_topic", stats_topic_},
      {"lidar_stats_stale_timeout_sec", lidar_stale_}, {"lidar_stats_available", true},
      {"log_dir", log_directory_.string()}, {"sample_period_sec", sample_period_},
      {"snapshot_period_sec", snapshot_period_}
    };
  }

  void run()
  {
    std::unique_lock<std::mutex> lock(wake_mutex_);
    while (!wake_.wait_for(lock, Seconds(sample_period_), [this]() {return stopping_;})) {
      lock.unlock();
      try {
        tick();
      } catch (const std::exception & problem) {
        RCLCPP_ERROR(node_.get_logger(), "diagnostics sample failed: %s", problem.what());
      }
      lock.lock();
    }
  }

  void tick()
  {
    const auto now = monotonic();
    std::optional<std::array<Presence, topic_count + 1>> presence;
    if (now - last_publisher_check_ >= publisher_refresh_) {
      presence.emplace();
      for (std::size_t i = 0; i < topics_.size(); ++i) {
        (*presence)[i] = {count(topics_[i].name), now};
      }
      (*presence)[topic_count] = {count(stats_topic_), now};
      last_publisher_check_ = now;
    }
    Json state;
    std::deque<Json> events;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (presence) {
        for (std::size_t i = 0; i < topics_.size(); ++i) {
          topics_[i].presence = (*presence)[i];
        }
        lidar_.presence = (*presence)[topic_count];
      }
      state = snapshot_locked(now);
      events = motion_.take_events();
    }
    state.update(log_.snapshot());
    if (!started_) {
      started_ = true;
      auto startup = state;
      startup["kind"] = "startup";
      log_.write(std::move(startup));
    }
    for (std::size_t i = 0; i < topic_count; ++i) {
      const auto & row = state["topics"][i];
      const auto status = row.at("status").get<std::string>();
      if (previous_status_[i] != status) {
        auto entry = row;
        entry.update({
          {"kind", "topic_status"}, {"event", "status_change"}, {"source", row["topic"]},
          {"previous_status", value_or_null(previous_status_[i])}, {"motion", state["motion"]}
        });
        log_.write(std::move(entry));
        previous_status_[i] = status;
      }
    }
    write_motion_events(events, state);
    if (now - last_snapshot_ >= snapshot_period_) {
      last_snapshot_ = now;
      state["kind"] = "snapshot";
      log_.write(std::move(state));
    }
  }

  void write_motion_events(std::deque<Json> & events, const Json & state)
  {
    for (auto & event : events) {
      event["motion"] = state["motion"];
      event["validity"] = state["motion"]["validity"];
      const auto & row = state["topics"][odom];
      event["publisher_count"] = row["publisher_count"];
      event["topic_hz"] = row["hz"];
      event["topic_age_sec"] = row["last_age_sec"];
      event["topic_status"] = row["status"];
      event["status"] = row["status"];
      log_.write(std::move(event));
    }
  }

  rclcpp::Node & node_;
  const fs::path log_directory_;
  const bool enabled_;
  const bool lidar_receive_;
  const double window_;
  const double stale_;
  const double lidar_stale_;
  const double sample_period_;
  const double snapshot_period_;
  const double publisher_refresh_;
  const std::string stats_topic_;
  mutable std::mutex state_mutex_;
  mutable std::array<Topic, topic_count> topics_;
  LidarMonitor lidar_;
  mutable Motion motion_;
  DiagnosticsLog log_;
  std::optional<ImuSample> imu_observation_;
  double last_imu_warning_ = 0;
  double last_publisher_check_ = 0;
  double last_snapshot_ = 0;
  std::array<std::optional<std::string>, topic_count> previous_status_{};
  bool started_ = false;

  std::mutex wake_mutex_;
  std::condition_variable wake_;
  std::mutex shutdown_mutex_;
  bool stopping_ = false;
  std::thread worker_;
  rclcpp::Subscription<Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<Odom>::SharedPtr odom_subscription_;
  rclcpp::Subscription<DriverStats>::SharedPtr stats_subscription_;
  rclcpp::GenericSubscription::SharedPtr raw_subscription_;
};

Diagnostics::Diagnostics(rclcpp::Node & node, const fs::path & log_directory, bool logging_enabled)
: impl_(std::make_unique<Impl>(node, log_directory, logging_enabled))
{
}

Diagnostics::~Diagnostics()
{
  try {
    shutdown();
  } catch (const std::exception &) {
    // The file closes on destruction even when the last event cannot be saved.
  }
}

Json Diagnostics::snapshot() const
{
  return impl_->snapshot();
}

Json Diagnostics::session() const
{
  return impl_->session();
}

void Diagnostics::set_logging(bool enabled)
{
  impl_->set_logging(enabled);
}

void Diagnostics::chassis_lifecycle(bool started)
{
  impl_->chassis_lifecycle(started);
}

void Diagnostics::shutdown()
{
  impl_->shutdown();
}
}  // namespace web_control
