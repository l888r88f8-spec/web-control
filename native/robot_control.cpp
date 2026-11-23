#include "robot_control.hpp"
#include "wall_clock.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

namespace web_control
{
namespace
{
using Clock = std::chrono::steady_clock;
using Seconds = std::chrono::duration<double>;
using Json = nlohmann::json;
using StatusArray = action_msgs::msg::GoalStatusArray;
using GoalStatus = action_msgs::msg::GoalStatus;
using Cancel = action_msgs::srv::CancelGoal;
using CancelClient = rclcpp::Client<Cancel>;
using Goal = geometry_msgs::msg::PoseStamped;
using InitialPose = geometry_msgs::msg::PoseWithCovarianceStamped;
using Twist = geometry_msgs::msg::Twist;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

template<typename T>
T parameter(rclcpp::Node & node, const std::string & name, const T & fallback)
{
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }
  return node.declare_parameter<T>(name, fallback);
}

ControlResponse response(int status, const std::string & message)
{
  return {status, {{"ok", status < 400}, {"message", message}, {"stamp", wall_time()}}};
}

std::string trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\n\r\f\v");
  return first == std::string::npos ? std::string{} :
    value.substr(first, value.find_last_not_of(" \t\n\r\f\v") - first + 1);
}

double finite_number(const Json & value, const std::string & key, bool allow_conversion)
{
  double result;
  if (value.is_number()) {
    result = value.get<double>();
  } else if (allow_conversion && value.is_boolean()) {
    result = value.get<bool>() ? 1.0 : 0.0;
  } else if (allow_conversion && value.is_string()) {
    const auto text = trim(value.get<std::string>());
    std::size_t end = 0;
    try {
      result = std::stod(text, &end);
    } catch (const std::exception &) {
      throw std::invalid_argument(key + " must be numeric");
    }
    if (end != text.size() || text.find_first_of("xX") != std::string::npos) {
      throw std::invalid_argument(key + " must be numeric");
    }
  } else {
    throw std::invalid_argument(key + (allow_conversion ? " must be numeric" : " must be a number"));
  }
  if (!std::isfinite(result)) {
    throw std::invalid_argument(key + " must be finite");
  }
  return result;
}

const Json & field(const Json & payload, const std::string & key)
{
  const auto value = payload.find(key);
  if (value == payload.end()) {
    throw std::invalid_argument("missing " + key);
  }
  return *value;
}

struct Pose
{
  double x;
  double y;
  double z;
  double yaw;
};

Pose read_pose(const Json & payload)
{
  return {
    finite_number(field(payload, "x"), "x", true),
    finite_number(field(payload, "y"), "y", true),
    payload.contains("z") ? finite_number(payload.at("z"), "z", true) : 0.0,
    finite_number(field(payload, "yaw"), "yaw", true)
  };
}

geometry_msgs::msg::Pose pose_message(const Pose & pose)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.x;
  msg.position.y = pose.y;
  msg.position.z = pose.z;
  msg.orientation.z = std::sin(pose.yaw / 2);
  msg.orientation.w = std::cos(pose.yaw / 2);
  return msg;
}

enum class Phase {idle, navigating, pausing, paused, resuming, clearing};

const char * phase_name(Phase phase)
{
  switch (phase) {
    case Phase::navigating: return "navigating";
    case Phase::pausing: return "pausing";
    case Phase::paused: return "paused";
    case Phase::resuming: return "resuming";
    case Phase::clearing: return "clearing";
    case Phase::idle: return "idle";
  }
  return "idle";
}

bool active_status(int value)
{
  return value == GoalStatus::STATUS_ACCEPTED || value == GoalStatus::STATUS_EXECUTING ||
         value == GoalStatus::STATUS_CANCELING;
}

bool terminal_status(int value)
{
  return value == GoalStatus::STATUS_SUCCEEDED || value == GoalStatus::STATUS_CANCELED ||
         value == GoalStatus::STATUS_ABORTED;
}

const char * action_name(int value)
{
  switch (value) {
    case GoalStatus::STATUS_ACCEPTED: return "accepted";
    case GoalStatus::STATUS_EXECUTING: return "executing";
    case GoalStatus::STATUS_CANCELING: return "canceling";
    case GoalStatus::STATUS_SUCCEEDED: return "succeeded";
    case GoalStatus::STATUS_CANCELED: return "canceled";
    case GoalStatus::STATUS_ABORTED: return "aborted";
    default: return "unknown";
  }
}

std::int64_t goal_stamp(const GoalStatus & status)
{
  const auto & stamp = status.goal_info.stamp;
  return std::int64_t{stamp.sec} * 1000000000 + stamp.nanosec;
}

struct Record
{
  Phase phase;
  Pose pose;
  std::string frame;
  std::string map_name;
  double saved_at;

  Json json() const
  {
    return {
      {"schema_version", 1}, {"phase", phase_name(phase)},
      {"x", pose.x}, {"y", pose.y}, {"z", pose.z}, {"yaw", pose.yaw},
      {"frame", frame}, {"map_name", map_name}, {"saved_at", saved_at}
    };
  }
};

fs::path default_record_path()
{
  const auto * home = std::getenv("HOME");
  return home && *home ? fs::path(home) / ".config/web_control/paused_navigation.json" : fs::path{};
}

Record read_record(const Json & json)
{
  const auto state = json.at("phase").get<std::string>();
  const auto frame = json.at("frame").get<std::string>();
  const auto map = json.at("map_name").get<std::string>();
  if (json.at("schema_version") != 1 ||
    (state != "pausing" && state != "resuming" && state != "paused") ||
    trim(frame).empty() || trim(map).empty() || map == "." || map == ".." ||
    map.find_first_of("/\\") != std::string::npos || map.find('\0') != std::string::npos)
  {
    throw std::invalid_argument("invalid paused navigation record");
  }
  field(json, "z");
  const auto phase = state == "pausing" ? Phase::pausing :
    (state == "resuming" ? Phase::resuming : Phase::paused);
  return {phase, read_pose(json), frame, map, finite_number(json.at("saved_at"), "saved_at", false)};
}

void write_record(const fs::path & path, const Record & record)
{
  const auto parent = path.has_parent_path() ? path.parent_path() : fs::path(".");
  fs::create_directories(parent);
  auto name = (parent / ("." + path.filename().string() + ".XXXXXX")).string();
  const int fd = mkstemp(name.data());
  if (fd < 0) {
    throw std::system_error(errno, std::generic_category(), "create paused navigation file");
  }
  bool closed = false;
  try {
    const auto bytes = record.json().dump() + "\n";
    std::size_t offset = 0;
    while (offset < bytes.size()) {
      const auto size = write(fd, bytes.data() + offset, bytes.size() - offset);
      if (size < 0 && errno == EINTR) {
        continue;
      }
      if (size <= 0) {
        throw std::system_error(size == 0 ? EIO : errno, std::generic_category(),
          "write paused navigation file");
      }
      offset += static_cast<std::size_t>(size);
    }
    if (fsync(fd) != 0) {
      throw std::system_error(errno, std::generic_category(), "flush paused navigation file");
    }
    const auto close_result = close(fd);
    closed = true;
    if (close_result != 0) {
      throw std::system_error(errno, std::generic_category(), "close paused navigation file");
    }
    fs::rename(name, path);
  } catch (...) {
    if (!closed) {
      close(fd);
    }
    std::error_code ignored;
    fs::remove(name, ignored);
    throw;
  }
}

class Teleop
{
public:
  explicit Teleop(rclcpp::Node & node)
  : logger_(node.get_logger()),
    max_speed_(parameter(node, "teleop_max_linear_speed", 1.5)),
    wheelbase_(parameter(node, "teleop_wheelbase", 0.550)),
    max_steer_(parameter(node, "teleop_max_steer_angle", 0.3270520925102273)),
    exponent_(parameter(node, "teleop_steering_exponent", 0.5)),
    deadzone_(parameter(node, "teleop_deadzone", 0.08)),
    timeout_(parameter(node, "teleop_command_timeout", 0.4))
  {
    const auto rate = parameter(node, "teleop_publish_rate", 20.0);
    for (const auto & [name, value] : std::initializer_list<std::pair<const char *, double>>{
        {"max_linear_speed", max_speed_}, {"wheelbase", wheelbase_},
        {"max_steer_angle", max_steer_}, {"steering_exponent", exponent_},
        {"command_timeout", timeout_.count()}, {"publish_rate", rate}})
    {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and positive");
      }
    }
    if (max_steer_ >= std::acos(0.0)) {
      throw std::invalid_argument("max_steer_angle must be within (0, pi/2)");
    }
    if (!std::isfinite(deadzone_) || deadzone_ < 0 || deadzone_ >= 0.5) {
      throw std::invalid_argument("deadzone must be within [0, 0.5)");
    }
    const auto period = Seconds(1.0 / rate);
    if (period < 1ns || period >= std::chrono::nanoseconds::max()) {
      throw std::invalid_argument("publish_rate is outside the wall timer range");
    }
    if (!std::isfinite(max_speed_ / wheelbase_ * std::tan(max_steer_))) {
      throw std::invalid_argument("teleop parameters can produce a non-finite angular velocity");
    }
    publisher_ = node.create_publisher<Twist>(
      parameter(node, "teleop_cmd_vel_topic", std::string("/cmd_vel")), 10);
    auto group = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = node.create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {tick();}, group);
    group_ = std::move(group);
  }

  ControlResponse command(const Json & payload)
  {
    const auto & throttle_raw = field(payload, "throttle");
    const auto & steering_raw = field(payload, "steering");
    const auto id = payload.find("client_id");
    const auto client = id != payload.end() && id->is_string() ? trim(id->get<std::string>()) :
      std::string{};
    if (client.empty() || client.size() > 128) {
      throw std::invalid_argument("client_id must be a nonempty bounded string");
    }
    const auto sequence = payload.find("seq");
    if (sequence == payload.end() || !sequence->is_number_integer() ||
      (sequence->is_number_unsigned() ? sequence->get<std::uint64_t>() == 0 :
      sequence->get<std::int64_t>() <= 0))
    {
      throw std::invalid_argument("seq must be a positive integer");
    }
    const auto seq = sequence->get<std::uint64_t>();
    const auto throttle = axis(throttle_raw, "throttle");
    const auto steering = axis(steering_raw, "steering");
    const auto angle = std::copysign(std::pow(std::abs(steering), exponent_), steering) * max_steer_;
    Twist twist;
    twist.linear.x = throttle * max_speed_;
    twist.angular.z = twist.linear.x / wheelbase_ * std::tan(angle);
    const bool active = twist.linear.x != 0.0;

    // Serialize the output with every stop and sequence update. The timer
    // cannot select an old velocity then publish it after a newer stop.
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      return response(503, "teleop controller is closed");
    }
    auto & accepted = last_seq_[client];
    if (seq <= accepted) {
      auto result = response(200, "stale teleop command ignored");
      result.body["stale"] = true;
      return result;
    }
    accepted = seq;
    if (active) {
      active_ = twist;
      command_at_ = Clock::now();
    } else {
      active_.reset();
      stop_pending_ = true;
    }
    auto result = response(200, "teleop command accepted");
    result.body["stale"] = false;
    result.body["teleop"] = {
      {"active", active}, {"throttle", throttle}, {"steering", steering},
      {"steering_angle", angle}, {"linear_x", twist.linear.x}, {"angular_z", twist.angular.z}
    };
    return result;
  }

  void shutdown()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      return;
    }
    closed_ = true;
    active_.reset();
    stop_pending_ = false;
    try {
      timer_->cancel();
      publisher_->publish(Twist{});
    } catch (const std::exception & problem) {
      RCLCPP_ERROR(logger_, "failed to publish teleop stop command: %s", problem.what());
    }
  }

private:
  double axis(const Json & value, const char * name) const
  {
    const auto clamped = std::clamp(finite_number(value, name, false), -1.0, 1.0);
    return std::abs(clamped) < deadzone_ ? 0.0 : clamped;
  }

  void tick()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      return;
    }
    try {
      // A release already queued must be emitted once, even if the operator
      // has pressed the joystick again before the timer runs.
      if (stop_pending_) {
        publisher_->publish(Twist{});
        stop_pending_ = false;
        return;
      }
      if (!active_) {
        return;
      }
      if (Clock::now() - command_at_ > timeout_) {
        active_.reset();
        stop_pending_ = true;
        publisher_->publish(Twist{});
        stop_pending_ = false;
        return;
      }
      publisher_->publish(*active_);
    } catch (const std::exception & problem) {
      RCLCPP_ERROR(logger_, "failed to publish teleop command: %s", problem.what());
    }
  }

  rclcpp::Logger logger_;
  const double max_speed_;
  const double wheelbase_;
  const double max_steer_;
  const double exponent_;
  const double deadzone_;
  const Seconds timeout_;
  rclcpp::Publisher<Twist>::SharedPtr publisher_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  std::unordered_map<std::string, std::uint64_t> last_seq_;
  std::optional<Twist> active_;
  Clock::time_point command_at_;
  bool stop_pending_ = false;
  bool closed_ = false;
};
}  // namespace

class RobotControl::Impl
{
public:
  explicit Impl(rclcpp::Node & node)
  : node_(node),
    map_frame_(parameter(node, "map_frame", std::string("map"))),
    record_path_(parameter(node, "navigation_record_path", default_record_path().string())),
    stale_timeout_(parameter(node, "stale_timeout_sec", 2.0)),
    viewer_timeout_(parameter(node, "viewer_timeout_sec", 10.0)),
    teleop_(node)
  {
    if (trim(map_frame_).empty()) {
      throw std::invalid_argument("map_frame must be a nonempty string");
    }
    if (!std::isfinite(viewer_timeout_.count()) || viewer_timeout_.count() <= 0.0) {
      throw std::invalid_argument("viewer_timeout_sec must be finite and positive");
    }
    load_record();
    initial_pose_ = node.create_publisher<InitialPose>(
      parameter(node, "initialpose_topic", std::string("/initialpose")), 5);
    goal_ = node.create_publisher<Goal>(
      parameter(node, "goal_topic", std::string("goal_pose_3d")), 5);
    cancel_client_ = node.create_client<Cancel>(
      parameter(node, "cancel_goal_service", std::string("/p2p_move_base/_action/cancel_goal")));
    status_subscription_ = node.create_subscription<StatusArray>(
      parameter(node, "action_status_topic", std::string("/p2p_move_base/_action/status")),
      rclcpp::QoS(10).reliable().durability_volatile(),
      [this](const StatusArray & message) {
        std::lock_guard<std::mutex> lock(navigation_mutex_);
        if (closed_) {
          return;
        }
        try {
          update_action_status(message);
        } catch (const std::exception & problem) {
          RCLCPP_ERROR(node_.get_logger(), "navigation status update failed: %s", problem.what());
        }
      });
    cancel_timer_ = node.create_wall_timer(20ms, [this]() {
      std::lock_guard<std::mutex> lock(navigation_mutex_);
      poll_cancel();
    });
  }

  std::optional<ControlResponse> route(
    std::string_view path, const Json & payload,
    const std::optional<std::string> & selected_map)
  {
    if (path != "/api/teleop" && path != "/api/viewer" && path != "/api/initialpose" &&
      path != "/api/goal" && path != "/api/navigation/pause" &&
      path != "/api/navigation/resume" && path != "/api/navigation/clear")
    {
      return std::nullopt;
    }
    try {
      if (!payload.is_object()) {
        return response(400, "payload must be an object");
      }
      if (path == "/api/teleop") {
        return teleop_.command(payload);
      }
      if (path == "/api/viewer") {
        const auto value = payload.find("viewer_id");
        const auto id = value == payload.end() ? std::string{} :
          trim(value->is_string() ? value->get<std::string>() : value->dump());
        if (id.empty()) {
          return response(400, "viewer_id is required");
        }
        std::lock_guard<std::mutex> lock(viewer_mutex_);
        const auto now = Clock::now();
        viewers_[id] = now;
        auto result = response(200, "viewer heartbeat");
        result.body["viewers"] = viewer_count(now);
        return result;
      }

      std::unique_lock<std::mutex> lock(navigation_mutex_);
      if (closed_) {
        return response(503, "robot control is closed");
      }
      if (path == "/api/initialpose") {
        const auto pose = read_pose(payload);
        InitialPose message;
        message.header.stamp = node_.get_clock()->now();
        message.header.frame_id = map_frame_;
        message.pose.pose = pose_message(pose);
        message.pose.covariance[0] = 0.25;
        message.pose.covariance[7] = 0.25;
        message.pose.covariance[35] = 0.0685;
        initial_pose_->publish(message);
        return response(200, "initial pose published");
      }
      if (path == "/api/goal") {
        return new_goal(payload, selected_map);
      }
      if (path == "/api/navigation/resume") {
        return resume(selected_map);
      }
      if (path == "/api/navigation/pause") {
        return pause(selected_map, lock);
      }
      return clear(selected_map, lock);
    } catch (const std::invalid_argument & problem) {
      return response(400, problem.what());
    } catch (const Json::exception & problem) {
      return response(400, problem.what());
    } catch (const std::exception & problem) {
      RCLCPP_ERROR(node_.get_logger(), "robot control request failed: %s", problem.what());
      return response(500, "internal server error");
    }
  }

  Json status(const std::optional<std::string> & map) const
  {
    Json result;
    {
      std::lock_guard<std::mutex> lock(navigation_mutex_);
      const auto now = Clock::now();
      const auto age = action_received_ ? Seconds(now - *action_received_).count() : 0.0;
      result["navigation_action"] = {
        {"status", !action_received_ ? "waiting" : (age > stale_timeout_ ? "stale" : "active")},
        {"stamp", action_received_ ? Json(action_wall_stamp_) : Json(nullptr)},
        {"age", action_received_ ? Json(age) : Json(nullptr)},
        {"frame_id", ""}, {"count", action_values_.size()},
        {"data", {{"status", action_status_}, {"status_values", action_values_}}}
      };
      result["navigation_control"] = navigation_status(map);
    }
    {
      std::lock_guard<std::mutex> lock(viewer_mutex_);
      result["viewers"] = viewer_count(Clock::now());
    }
    return result;
  }

  void map_selected(const std::string & name)
  {
    std::lock_guard<std::mutex> lock(navigation_mutex_);
    if (record_ && record_->map_name != name) {
      remove_record();
      if (phase_ == Phase::paused || phase_ == Phase::pausing) {
        phase_ = active_ ? Phase::navigating : Phase::idle;
      }
      retry_cancel_on_active_ = false;
    }
  }

  void clear_action_status()
  {
    std::lock_guard<std::mutex> lock(navigation_mutex_);
    clear_action_status_locked();
  }

  void navigation_stopped()
  {
    std::lock_guard<std::mutex> lock(navigation_mutex_);
    try {
      if (phase_ == Phase::clearing) {
        remove_record();
      } else if (record_ && (phase_ == Phase::resuming || phase_ == Phase::pausing)) {
        set_record_phase(Phase::paused);
      }
    } catch (const std::exception & problem) {
      RCLCPP_ERROR(node_.get_logger(), "failed to settle persisted navigation: %s", problem.what());
    }
    phase_ = record_ ? Phase::paused : Phase::idle;
    active_ = false;
    current_goal_.reset();
    floor_.reset();
    last_status_stamp_ = -1;
    reconciled_ = false;
    reset_cancel_tracking();
    abort_cancel("navigation stopped");
    clear_action_status_locked();
  }

  void shutdown()
  {
    teleop_.shutdown();
    std::lock_guard<std::mutex> lock(navigation_mutex_);
    if (!closed_) {
      closed_ = true;
      cancel_timer_->cancel();
      abort_cancel("robot control is closed");
    }
  }

private:
  enum class CancelResult {accepted, empty, rejected, unavailable, timeout};

  struct CancelRequest
  {
    std::optional<CancelClient::FutureAndRequestId> future;
    Clock::time_point deadline;
    bool retry;
    std::optional<CancelResult> result;
  };

  void clear_action_status_locked()
  {
    action_received_.reset();
    action_status_ = "idle";
    action_values_.clear();
  }

  std::size_t viewer_count(Clock::time_point now) const
  {
    for (auto it = viewers_.begin(); it != viewers_.end(); ) {
      if (now - it->second > viewer_timeout_) {
        it = viewers_.erase(it);
      } else {
        ++it;
      }
    }
    return viewers_.size();
  }

  void load_record()
  {
    try {
      if (record_path_.empty() || !fs::exists(record_path_)) {
        return;
      }
      std::ifstream file(record_path_);
      Json json;
      file >> json;
      record_ = read_record(json);
      phase_ = record_->phase;
    } catch (const std::exception & problem) {
      RCLCPP_WARN(node_.get_logger(), "invalid paused navigation record: %s", problem.what());
    }
  }

  void save_record(const Record & next)
  {
    if (record_path_.empty()) {
      throw std::runtime_error("paused navigation record path is unavailable");
    }
    write_record(record_path_, next);
    record_ = next;
  }

  void set_record_phase(Phase phase)
  {
    if (record_) {
      auto next = *record_;
      next.phase = phase;
      next.saved_at = wall_time();
      save_record(next);
    }
  }

  void remove_record()
  {
    if (!record_path_.empty() && unlink(record_path_.c_str()) != 0 && errno != ENOENT) {
      throw std::system_error(errno, std::generic_category(), "remove paused navigation file");
    }
    record_.reset();
  }

  bool can_pause() const
  {
    return phase_ == Phase::navigating && active_ && current_goal_.has_value() && !cancel_request_;
  }

  Json navigation_status(const std::optional<std::string> & map) const
  {
    const bool has_goal = active_ || current_goal_ || record_;
    return {
      {"state", phase_name(phase_)},
      {"has_goal", has_goal},
      {"can_pause", can_pause()},
      {"can_resume", phase_ == Phase::paused && record_ && map && record_->map_name == *map &&
        record_->frame == map_frame_ && !cancel_request_},
      {"can_clear", has_goal &&
        (phase_ == Phase::navigating || phase_ == Phase::paused || phase_ == Phase::pausing)},
      {"paused_goal", record_ ? record_->json() : Json(nullptr)}
    };
  }

  ControlResponse navigation_response(const char * message, const std::optional<std::string> & map)
  {
    auto result = response(200, message);
    result.body["navigation_control"] = navigation_status(map);
    return result;
  }

  void publish_goal(const Pose & pose, const std::string & frame)
  {
    Goal message;
    message.header.stamp = node_.get_clock()->now();
    message.header.frame_id = frame;
    message.pose = pose_message(pose);
    goal_->publish(message);
  }

  void begin_confirmation(const Pose & pose, Phase phase)
  {
    current_goal_ = pose;
    phase_ = phase;
    active_ = false;
    floor_ = last_status_stamp_;
    reconciled_ = true;
    reset_cancel_tracking();
  }

  ControlResponse new_goal(const Json & payload, const std::optional<std::string> & map)
  {
    if (cancel_request_ || phase_ == Phase::pausing || phase_ == Phase::resuming ||
      phase_ == Phase::clearing)
    {
      return response(409, "navigation is busy; wait for the current command to finish");
    }
    const auto pose = read_pose(payload);
    // Hold the navigation mutex across publish and the transition so an
    // Action status received during publication sees the new status floor.
    publish_goal(pose, map_frame_);
    begin_confirmation(pose, Phase::navigating);
    remove_record();
    return navigation_response("goal published", map);
  }

  ControlResponse resume(const std::optional<std::string> & map)
  {
    if (cancel_request_) {
      return response(409, "navigation command in progress; try again");
    }
    if (phase_ != Phase::paused || !record_) {
      return response(409, "no paused navigation goal to resume");
    }
    if (!map || *map != record_->map_name) {
      return response(409, "paused goal belongs to a different map");
    }
    if (record_->frame != map_frame_) {
      return response(409, "paused goal belongs to a different frame");
    }
    set_record_phase(Phase::resuming);
    try {
      publish_goal(record_->pose, record_->frame);
    } catch (const std::exception & problem) {
      set_record_phase(Phase::paused);
      return response(503, std::string("resume publish failed: ") + problem.what());
    }
    begin_confirmation(record_->pose, Phase::resuming);
    return navigation_response("resume published", map);
  }

  ControlResponse pause(
    const std::optional<std::string> & map, std::unique_lock<std::mutex> & lock)
  {
    if (!map) {
      return response(409, "no map selected; cannot pause navigation");
    }
    if (!can_pause()) {
      return response(409, "no active navigation goal to pause");
    }
    save_record({Phase::pausing, *current_goal_, map_frame_, *map, wall_time()});
    phase_ = Phase::pausing;
    tracked_pause_cancel_ = true;
    return wait_for_cancel(map, lock, true);
  }

  ControlResponse clear(
    const std::optional<std::string> & map, std::unique_lock<std::mutex> & lock)
  {
    if (phase_ == Phase::clearing) {
      return response(409, "clear is already in progress");
    }
    if (phase_ == Phase::resuming || (cancel_request_ && phase_ != Phase::pausing)) {
      return response(409, "navigation command in progress; try again");
    }
    if (phase_ == Phase::idle) {
      return response(409, "no navigation goal to clear");
    }
    if (phase_ == Phase::paused || (phase_ == Phase::navigating && !active_ && !floor_)) {
      finish_clear();
      reconciled_ = true;
      return navigation_response("clear requested", map);
    }
    const bool reuse_pause_cancel = phase_ == Phase::pausing &&
      (tracked_pause_cancel_ || cancel_request_);
    clear_started_unconfirmed_ = phase_ == Phase::navigating && floor_.has_value();
    phase_ = Phase::clearing;
    reconciled_ = true;
    retry_cancel_on_active_ = false;
    if (reuse_pause_cancel) {
      return navigation_response("clear requested", map);
    }
    return wait_for_cancel(map, lock, false);
  }

  std::shared_ptr<CancelRequest> start_cancel(bool retry)
  {
    auto request = std::make_shared<CancelRequest>();
    request->retry = retry;
    request->deadline = Clock::now() + 2s;
    try {
      if (!cancel_client_->service_is_ready()) {
        request->result = CancelResult::unavailable;
      } else {
        // A zero GoalInfo is the Action cancellation request for all goals.
        request->future.emplace(cancel_client_->async_send_request(std::make_shared<Cancel::Request>()));
        cancel_request_ = request;
      }
    } catch (const std::exception & problem) {
      RCLCPP_WARN(node_.get_logger(), "cancel goal service call failed: %s", problem.what());
      request->result = CancelResult::unavailable;
    }
    if (request->result) {
      apply_cancel(*request->result, retry);
    }
    return request;
  }

  ControlResponse wait_for_cancel(
    const std::optional<std::string> & map, std::unique_lock<std::mutex> & lock, bool pausing)
  {
    const auto request = start_cancel(false);
    // HTTP workers wait independently of the ROS executor, which delivers the
    // service result. The wall timer also services retries without blocking it.
    if (!cancel_ready_.wait_until(lock, request->deadline, [&]() {return request->result.has_value();})) {
      poll_cancel();
    }
    switch (request->result.value_or(CancelResult::timeout)) {
      case CancelResult::timeout:
        return response(504, "cancel goal request timed out");
      case CancelResult::unavailable:
        return response(503, "cancel goal service unavailable");
      case CancelResult::empty:
        if (pausing) {
          return response(409, "no active navigation goal to cancel");
        }
        break;
      case CancelResult::rejected:
        return pausing ? response(409, "no active navigation goal to cancel") :
               response(503, "cancel goal request rejected");
      case CancelResult::accepted:
        break;
    }
    return navigation_response(pausing ? "pause requested" : "clear requested", map);
  }

  void poll_cancel()
  {
    if (!cancel_request_) {
      return;
    }
    const auto request = cancel_request_;
    auto & future = *request->future;
    CancelResult result;
    if (future.wait_for(0ms) == std::future_status::ready) {
      result = CancelResult::unavailable;
      try {
        const auto reply = future.get();
        if (reply) {
          if (reply->return_code != Cancel::Response::ERROR_NONE) {
            result = CancelResult::rejected;
          } else {
            result = reply->goals_canceling.empty() ? CancelResult::empty : CancelResult::accepted;
          }
        }
      } catch (const std::exception & problem) {
        RCLCPP_WARN(node_.get_logger(), "cancel goal service response failed: %s", problem.what());
      }
    } else if (Clock::now() >= request->deadline) {
      cancel_client_->remove_pending_request(future);
      result = CancelResult::timeout;
    } else {
      return;
    }
    cancel_request_.reset();
    request->result = result;
    try {
      apply_cancel(result, request->retry);
    } catch (const std::exception & problem) {
      request->result = CancelResult::unavailable;
      RCLCPP_ERROR(node_.get_logger(), "cancel goal state update failed: %s", problem.what());
    }
    cancel_ready_.notify_all();
  }

  void abort_cancel(const char * reason)
  {
    if (cancel_request_) {
      cancel_client_->remove_pending_request(*cancel_request_->future);
      cancel_request_->result = CancelResult::unavailable;
      cancel_request_.reset();
      cancel_ready_.notify_all();
      RCLCPP_DEBUG(node_.get_logger(), "cancel request dropped: %s", reason);
    }
  }

  void apply_cancel(CancelResult result, bool retry)
  {
    if (phase_ != Phase::clearing && phase_ != Phase::pausing) {
      return;
    }
    if (result == CancelResult::accepted) {
      tracked_pause_cancel_ = phase_ == Phase::pausing;
      retry_cancel_on_active_ = false;
      clear_started_unconfirmed_ = false;
      return;
    }
    if (result == CancelResult::timeout && !retry) {
      tracked_pause_cancel_ = false;
      start_cancel(true);
      return;
    }
    if (result == CancelResult::empty && phase_ == Phase::clearing) {
      if (floor_) {
        if (retry) {
          finish_cancel_failure();
        } else {
          retry_cancel_on_active_ = true;
        }
      } else if (!retry && clear_started_unconfirmed_) {
        clear_started_unconfirmed_ = false;
        start_cancel(true);
      } else {
        finish_clear();
      }
      return;
    }
    finish_cancel_failure();
  }

  void finish_cancel_failure()
  {
    reset_cancel_tracking();
    phase_ = active_ || floor_ || current_goal_ ? Phase::navigating : Phase::idle;
    remove_record();
  }

  void finish_clear()
  {
    reset_cancel_tracking();
    phase_ = Phase::idle;
    floor_.reset();
    current_goal_.reset();
    active_ = false;
    remove_record();
  }

  void reset_cancel_tracking()
  {
    tracked_pause_cancel_ = false;
    retry_cancel_on_active_ = false;
    clear_started_unconfirmed_ = false;
  }

  void settle_paused()
  {
    reset_cancel_tracking();
    set_record_phase(Phase::paused);
    phase_ = record_ ? Phase::paused : Phase::idle;
    current_goal_.reset();
    active_ = false;
  }

  void update_action_status(const StatusArray & message)
  {
    action_received_ = Clock::now();
    action_wall_stamp_ = wall_time();
    action_values_.clear();
    const GoalStatus * selected = nullptr;
    for (const auto & item : message.status_list) {
      action_values_.push_back(item.status);
      if (selected == nullptr || (active_status(item.status) && !active_status(selected->status)) ||
        (active_status(item.status) == active_status(selected->status) &&
        goal_stamp(item) >= goal_stamp(*selected)))
      {
        selected = &item;
      }
    }
    action_status_ = selected ? action_name(selected->status) : "idle";
    if (!selected) {
      if (!reconciled_) {
        reconciled_ = true;
        if (phase_ == Phase::pausing || phase_ == Phase::resuming) {
          settle_paused();
        }
      }
      return;
    }

    const auto stamp = goal_stamp(*selected);
    if (stamp < last_status_stamp_ || (floor_ && stamp <= *floor_)) {
      return;
    }
    last_status_stamp_ = stamp;
    const auto value = selected->status;
    if (floor_) {
      if (terminal_status(value)) {
        floor_.reset();
        if (phase_ == Phase::resuming) {
          settle_paused();
        } else {
          finish_clear();
        }
        return;
      }
      if (value != GoalStatus::STATUS_ACCEPTED && value != GoalStatus::STATUS_EXECUTING) {
        return;
      }
      floor_.reset();
      if (phase_ == Phase::clearing && retry_cancel_on_active_) {
        active_ = true;
        retry_cancel_on_active_ = false;
        clear_started_unconfirmed_ = false;
        start_cancel(true);
        return;
      }
    }

    if (!reconciled_) {
      reconciled_ = true;
      if (phase_ == Phase::pausing && active_status(value)) {
        active_ = true;
        start_cancel(true);
        return;
      }
      if ((phase_ == Phase::pausing || phase_ == Phase::resuming) && !active_status(value)) {
        settle_paused();
        return;
      }
    }
    if (active_status(value)) {
      active_ = true;
      if (phase_ == Phase::idle || phase_ == Phase::paused || phase_ == Phase::resuming) {
        phase_ = Phase::navigating;
        remove_record();
      }
    } else {
      active_ = false;
      if (!terminal_status(value)) {
        return;
      }
      if (phase_ == Phase::pausing) {
        settle_paused();
      } else if (phase_ == Phase::clearing ||
        (phase_ == Phase::navigating && value != GoalStatus::STATUS_CANCELED))
      {
        finish_clear();
      }
    }
  }

  rclcpp::Node & node_;
  const std::string map_frame_;
  const fs::path record_path_;
  const double stale_timeout_;
  const Seconds viewer_timeout_;
  Teleop teleop_;
  rclcpp::Publisher<InitialPose>::SharedPtr initial_pose_;
  rclcpp::Publisher<Goal>::SharedPtr goal_;
  CancelClient::SharedPtr cancel_client_;
  rclcpp::Subscription<StatusArray>::SharedPtr status_subscription_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;

  mutable std::mutex viewer_mutex_;
  mutable std::unordered_map<std::string, Clock::time_point> viewers_;
  mutable std::mutex navigation_mutex_;
  std::condition_variable cancel_ready_;
  std::optional<Record> record_;
  std::optional<Pose> current_goal_;
  std::shared_ptr<CancelRequest> cancel_request_;
  std::optional<Clock::time_point> action_received_;
  double action_wall_stamp_ = 0.0;
  std::string action_status_ = "idle";
  std::vector<int> action_values_;
  std::optional<std::int64_t> floor_;
  std::int64_t last_status_stamp_ = -1;
  Phase phase_ = Phase::idle;
  bool active_ = false;
  bool reconciled_ = false;
  bool clear_started_unconfirmed_ = false;
  bool retry_cancel_on_active_ = false;
  bool tracked_pause_cancel_ = false;
  bool closed_ = false;
};

RobotControl::RobotControl(rclcpp::Node & node)
: impl_(std::make_unique<Impl>(node))
{
}

RobotControl::~RobotControl()
{
  shutdown();
}

std::optional<ControlResponse> RobotControl::route(
  std::string_view path, const nlohmann::json & payload,
  const std::optional<std::string> & selected_map)
{
  return impl_->route(path, payload, selected_map);
}

nlohmann::json RobotControl::status(const std::optional<std::string> & selected_map) const
{
  return impl_->status(selected_map);
}

void RobotControl::map_selected(const std::string & name)
{
  impl_->map_selected(name);
}

void RobotControl::clear_action_status()
{
  impl_->clear_action_status();
}

void RobotControl::navigation_stopped()
{
  impl_->navigation_stopped();
}

void RobotControl::shutdown()
{
  impl_->shutdown();
}
}  // namespace web_control
