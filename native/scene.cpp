#include "scene.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iterator>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "cloud_response.hpp"
#include "terrain.hpp"
#include "wall_clock.hpp"

namespace web_control
{
namespace
{
using json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
using sensor_msgs::msg::PointField;

std::string trim(const std::string & text)
{
  const auto begin = text.find_first_not_of(" \t\n\r\v\f");
  if (begin == std::string::npos) {
    return {};
  }
  return text.substr(begin, text.find_last_not_of(" \t\n\r\v\f") - begin + 1);
}

template<typename T>
T parameter(rclcpp::Node & node, const std::string & name, T fallback)
{
  return node.has_parameter(name) ? node.get_parameter(name).get_value<T>() :
    node.declare_parameter<T>(name, fallback);
}

std::vector<std::string> split(const std::string & list)
{
  std::istringstream input(list);
  std::vector<std::string> result;
  for (std::string item; std::getline(input, item, ',');) {
    auto value = trim(item);
    if (!value.empty()) {
      result.push_back(std::move(value));
    }
  }
  return result;
}

std::size_t field_width(unsigned type)
{
  switch (type) {
    case PointField::FLOAT64: return 8;
    case PointField::FLOAT32:
    case PointField::INT32:
    case PointField::UINT32: return 4;
    case PointField::INT16:
    case PointField::UINT16: return 2;
    case PointField::INT8:
    case PointField::UINT8: return 1;
    default: throw std::invalid_argument("unsupported point field type");
  }
}

template<typename T>
T read_number(const std::uint8_t * bytes, bool big_endian)
{
  std::uint8_t ordered[sizeof(T)];
  const std::uint16_t host_order = 1;
  const bool host_big_endian = *reinterpret_cast<const std::uint8_t *>(&host_order) == 0;
  if (big_endian == host_big_endian) {
    std::copy_n(bytes, sizeof(T), ordered);
  } else {
    std::reverse_copy(bytes, bytes + sizeof(T), ordered);
  }
  T result;
  std::memcpy(&result, ordered, sizeof(T));
  return result;
}

double coordinate(const std::uint8_t * point, const PointField & field, bool big_endian)
{
  const auto * bytes = point + field.offset;
  switch (field.datatype) {
    case PointField::FLOAT64: return read_number<double>(bytes, big_endian);
    case PointField::FLOAT32: return read_number<float>(bytes, big_endian);
    case PointField::INT32: return read_number<std::int32_t>(bytes, big_endian);
    case PointField::UINT32: return read_number<std::uint32_t>(bytes, big_endian);
    case PointField::INT16: return read_number<std::int16_t>(bytes, big_endian);
    case PointField::UINT16: return read_number<std::uint16_t>(bytes, big_endian);
    case PointField::INT8: return read_number<std::int8_t>(bytes, big_endian);
    default: return *bytes;
  }
}

std::vector<float> cloud_points(const sensor_msgs::msg::PointCloud2 & message)
{
  if (message.width == 0 || message.height == 0) {
    return {};
  }
  const auto row_bytes = std::uint64_t{message.width} * message.point_step;
  const auto total_bytes = std::uint64_t{message.height} * message.row_step;
  if (message.point_step == 0 || message.row_step < row_bytes ||
    total_bytes > message.data.size())
  {
    throw std::invalid_argument("invalid point cloud stride or size");
  }
  const PointField * fields[3];
  for (std::size_t i = 0; i < 3; ++i) {
    const auto found = std::find_if(message.fields.begin(), message.fields.end(), [i](auto & field) {
      return field.name.size() == 1 && field.name.front() == "xyz"[i];
    });
    if (found == message.fields.end() || found->count == 0 ||
      std::uint64_t{found->offset} + field_width(found->datatype) > message.point_step)
    {
      throw std::invalid_argument("invalid or missing x/y/z point field");
    }
    fields[i] = &*found;
  }
  std::vector<float> result;
  result.reserve(static_cast<std::size_t>(message.width) * message.height * 3);
  for (std::size_t row = 0; row < message.height; ++row) {
    const auto * point = message.data.data() + row * message.row_step;
    for (std::size_t column = 0; column < message.width; ++column, point += message.point_step) {
      const double x = coordinate(point, *fields[0], message.is_bigendian);
      const double y = coordinate(point, *fields[1], message.is_bigendian);
      const double z = coordinate(point, *fields[2], message.is_bigendian);
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
        std::max({std::abs(x), std::abs(y), std::abs(z)}) <= std::numeric_limits<float>::max())
      {
        result.insert(result.end(), {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
      }
    }
  }
  return result;
}

const geometry_msgs::msg::Pose & as_pose(const geometry_msgs::msg::Pose & pose) {return pose;}
const geometry_msgs::msg::Pose & as_pose(const geometry_msgs::msg::PoseStamped & pose) {return pose.pose;}

json path_points(const std::vector<float> & points, bool project_2d = false)
{
  json result = json::array();
  for (std::size_t i = 0; i < points.size(); i += 3) {
    if (project_2d) {
      result.push_back({points[i], points[i + 1]});
    } else {
      result.push_back({points[i], points[i + 1], points[i + 2]});
    }
  }
  return result;
}
}  // namespace

Scene::Scene(rclcpp::Node & node)
: node_(node),
  cloud_group_(node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false)),
  epoch_(std::to_string(std::chrono::system_clock::now().time_since_epoch().count())),
  map_frame_(parameter(node, "map_frame", std::string("map"))),
  trajectory_(trim(parameter(node, "trajectory_topic", std::string("/best_trajectory")))),
  probe_trajectory_(trim(parameter(node, "probe_trajectory_topic", std::string("/best_trajectory_probe")))),
  stale_timeout_(parameter(node, "stale_timeout_sec", 2.0)),
  demand_ttl_(std::max(0.0, parameter(node, "cloud_demand_ttl_sec", 2.0))),
  idle_timeout_(parameter(node, "cloud_subscription_idle_sec", 5.0))
{
  add_cloud({"nav", "global_map"}, "global_map_topic", "/lio_sam/localization/global_map", true, true, true);
  add_cloud({"nav", "local_map"}, "local_map_topic", "/lio_sam/localization/local_map", true, false, true);
  add_cloud({"nav", "current_lidar"}, "current_lidar_topic", "/lio_sam/localization/current_lidar", false);
  add_cloud({"nav", "mapcloud"}, "mapcloud_topic", "mapcloud", true, true, true);
  add_cloud({"nav", "mapground"}, "mapground_topic", "mapground", true, true, true);
  add_cloud({"nav", "obstacles"}, "obstacle_topic", "/aggregated_pc", false);
  add_cloud({"nav", "grid_terrain"}, "grid_terrain_topic", "/grid_global_planner/terrain", true, false, true);
  add_cloud({"mapping", "global_map"}, "mapping_global_map_topic", "/lio_sam/mapping/map_global",
    false, false, true);
  add_cloud({"mapping", "ground"}, "mapping_ground_topic", "/lio_sam/mapping/ground_cloud_global",
    false, false, true);

  add_pose("nav", parameter(node, "odom_topic", std::string("/lio_sam/localization/odometry")));
  add_pose("mapping", parameter(node, "mapping_odom_topic", std::string("/lio_sam/mapping/odometry")));
  for (const auto & topic : split(parameter(node, "path_topics", std::string()))) {
    if (topic != trajectory_ && topic != probe_trajectory_) {
      add_path(topic, topic, false);
    }
  }
  add_path("mapping_path", parameter(node, "mapping_path_topic", std::string("/lio_sam/mapping/path")), false);
  if (!trajectory_.empty()) {
    add_path(trajectory_, trajectory_, true);
  }
  if (!probe_trajectory_.empty()) {
    add_path(probe_trajectory_, probe_trajectory_, true);
  }
  timer_ = node_.create_wall_timer(200ms, [this]() {reconcile();}, cloud_group_);
}

void Scene::add_cloud(Key key, const std::string & name, const std::string & topic,
  bool latched, bool keep_subscription, bool static_source)
{
  auto & resource = resources_[key];
  resource.topic = trim(parameter(node_, name, topic));
  resource.latched = latched;
  resource.keep_subscription = keep_subscription;
  resource.static_source = static_source;
}

void Scene::add_pose(const std::string & layer, const std::string & topic)
{
  const Key key{"pose", layer};
  auto & resource = resources_[key];
  if (topic.empty()) {
    return;
  }
  resource.subscription = node_.create_subscription<nav_msgs::msg::Odometry>(
    topic, rclcpp::QoS(10).reliable().durability_volatile(),
    [this, key](nav_msgs::msg::Odometry::ConstSharedPtr message) {
      const auto version = generation(key);
      const auto & pose = message->pose.pose;
      const auto & q = pose.orientation;
      const auto yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      auto frame = std::make_shared<Frame>();
      frame->stamp = wall_time();
      frame->frame_id = message->header.frame_id;
      frame->pose = {{"x", pose.position.x}, {"y", pose.position.y}, {"z", pose.position.z},
        {"qx", q.x}, {"qy", q.y}, {"qz", q.z}, {"qw", q.w}, {"yaw", yaw}, {"frame_id", frame->frame_id}};
      publish(key, version, std::move(frame));
    });
}

void Scene::add_path(const std::string & layer, const std::string & topic, bool pose_array)
{
  const Key key{"path", layer};
  auto & resource = resources_[key];
  if (topic.empty()) {
    return;
  }
  rclcpp::SubscriptionOptions options;
  options.callback_group = cloud_group_;
  auto callback = [this, key](const auto & message) {
      const auto version = generation(key);
      auto frame = std::make_shared<Frame>();
      frame->stamp = wall_time();
      frame->frame_id = message.header.frame_id;
      frame->points.reserve(message.poses.size() * 3);
      for (const auto & entry : message.poses) {
        const auto & point = as_pose(entry).position;
        frame->points.insert(frame->points.end(),
          {static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z)});
      }
      publish(key, version, std::move(frame));
    };
  const auto qos = rclcpp::QoS(10).reliable().durability_volatile();
  if (pose_array) {
    resource.subscription = node_.create_subscription<geometry_msgs::msg::PoseArray>(
      topic, qos, [callback](geometry_msgs::msg::PoseArray::ConstSharedPtr message) {
        callback(*message);
      }, options);
  } else {
    resource.subscription = node_.create_subscription<nav_msgs::msg::Path>(
      topic, qos, [callback](nav_msgs::msg::Path::ConstSharedPtr message) {
        callback(*message);
      }, options);
  }
}

bool Scene::demand_active(const Resource & resource, Clock::time_point now) const
{
  return resource.demand &&
         std::chrono::duration<double>(now - *resource.demand).count() <= demand_ttl_;
}

bool Scene::cloud_decode_ready(const Resource & resource, Clock::time_point now) const
{
  // Only the three permanent latched navigation maps have a 1 Hz decode ceiling.
  return demand_active(resource, now) &&
         (!resource.keep_subscription || !resource.last_cloud_decode ||
         now - *resource.last_cloud_decode >= 1s);
}

rclcpp::SubscriptionBase::SharedPtr Scene::subscribe_cloud(const Key & key, const Resource & resource)
{
  rclcpp::SubscriptionOptions options;
  options.callback_group = cloud_group_;
  auto qos = rclcpp::QoS(1);
  if (resource.latched) {
    qos.reliable().transient_local();
  } else {
    qos.best_effort().durability_volatile();
  }
  if (key.second == "grid_terrain") {
    return node_.create_subscription<visualization_msgs::msg::MarkerArray>(
      resource.topic, qos, [this, key](visualization_msgs::msg::MarkerArray::ConstSharedPtr message) {
        receive_cloud(key, std::move(message));
      }, options);
  }
  return node_.create_subscription<sensor_msgs::msg::PointCloud2>(
    resource.topic, qos, [this, key](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
      receive_cloud(key, std::move(message));
    }, options);
}

void Scene::reconcile()
{
  for (auto & [key, resource] : resources_) {
    if ((key.first != "nav" && key.first != "mapping") || resource.topic.empty()) {
      continue;
    }
    bool subscribe = false;
    bool decode = false;
    std::optional<PendingCloud> pending;
    rclcpp::SubscriptionBase::SharedPtr release;
    {
      const auto now = Clock::now();
      std::lock_guard<std::mutex> lock(mutex_);
      if (resource.subscription) {
        const auto max_idle = std::chrono::duration<double>(std::max(demand_ttl_, idle_timeout_));
        if (!resource.keep_subscription && idle_timeout_ > 0 &&
          (!resource.demand || now - *resource.demand > max_idle))
        {
          release = std::move(resource.subscription);
          pending = std::exchange(resource.pending_cloud, std::nullopt);
          resource.demand.reset();
          resource.last_cloud_decode.reset();
        } else if (resource.pending_cloud && cloud_decode_ready(resource, now)) {
          pending = std::exchange(resource.pending_cloud, std::nullopt);
          resource.last_cloud_decode = now;
          decode = true;
        }
      } else if (demand_active(resource, now) &&
        (!resource.subscribe_retry || now >= *resource.subscribe_retry))
      {
        subscribe = true;
      }
    }
    // DDS setup and teardown may block. Keep them on the cloud executor and
    // outside the scene mutex so SSE and state reads can proceed.
    if (subscribe) {
      try {
        auto subscription = subscribe_cloud(key, resource);
        std::lock_guard<std::mutex> lock(mutex_);
        resource.subscription = std::move(subscription);
        resource.subscribe_retry.reset();
      } catch (const std::exception & error) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          resource.subscribe_retry = Clock::now() + 5s;
        }
        RCLCPP_WARN(node_.get_logger(), "cloud subscription %s/%s: %s",
          key.first.c_str(), key.second.c_str(), error.what());
      }
    }
    if (decode) {
      decode_cloud(key, *pending);
    }
  }
}

void Scene::receive_cloud(const Key & key, CloudMessage message)
{
  PendingCloud pending{std::move(message), wall_time(), 0};
  std::optional<PendingCloud> previous;
  {
    const auto now = Clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    auto & resource = resources_.at(key);
    pending.generation = resource.generation;
    previous = std::exchange(resource.pending_cloud, std::nullopt);
    if (!cloud_decode_ready(resource, now)) {
      resource.pending_cloud = std::move(pending);
      return;
    }
    resource.last_cloud_decode = now;
  }
  decode_cloud(key, pending);
}

void Scene::decode_cloud(const Key & key, const PendingCloud & pending)
{
  try {
    auto frame = std::make_shared<Frame>();
    frame->stamp = pending.stamp;
    if (const auto * pointcloud =
      std::get_if<sensor_msgs::msg::PointCloud2::ConstSharedPtr>(&pending.message))
    {
      frame->points = cloud_points(**pointcloud);
      frame->frame_id = (*pointcloud)->header.frame_id;
      const bool live_empty_replaces = key.second == "obstacles" || key.second == "current_lidar";
      if (frame->points.empty() && !live_empty_replaces) {
        return;
      }
    } else {
      const auto & message = std::get<visualization_msgs::msg::MarkerArray::ConstSharedPtr>(
        pending.message);
      auto mesh = convert_terrain(*message);
      if (mesh.positions.empty()) {
        publish(key, pending.generation, nullptr);
        return;
      }
      frame->frame_id = mesh.frame_id.empty() ? map_frame_ : std::move(mesh.frame_id);
      frame->points = std::move(mesh.positions);
      frame->colors = std::move(mesh.colors);
    }
    publish(key, pending.generation, std::move(frame));
  } catch (const std::exception & error) {
    RCLCPP_WARN(node_.get_logger(), "cloud %s/%s: %s",
      key.first.c_str(), key.second.c_str(), error.what());
  }
}

std::size_t Scene::generation(const Key & key) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return resources_.at(key).generation;
}

void Scene::publish(const Key & key, std::size_t generation, std::shared_ptr<const Frame> frame)
{
  std::shared_ptr<const Frame> previous;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto & resource = resources_.at(key);
    if (resource.generation != generation || (!resource.frame && !frame)) {
      return;
    }
    previous = std::exchange(resource.frame, std::move(frame));
    ++resource.version;
  }
  notify_watchers({key});
}

void Scene::clear(const std::string & target)
{
  std::vector<std::shared_ptr<const Frame>> previous;
  std::vector<PendingCloud> pending;
  std::vector<Key> changed;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & [key, resource] : resources_) {
      const auto & [group, layer] = key;
      const bool mapping = group == "mapping" || (group == "pose" && layer == "mapping") ||
        (group == "path" && layer == "mapping_path");
      const bool nav_path = group == "path" && !mapping;
      const bool nav_pose = group == "pose" && layer == "nav";
      const bool localization_start = group == "nav" &&
        (layer == "global_map" || layer == "current_lidar");
      const bool navigation = nav_path || (group == "nav" &&
        (layer == "mapcloud" || layer == "mapground" || layer == "obstacles" ||
        layer == "grid_terrain"));
      const bool nav = group == "nav" || nav_path || nav_pose;
      const bool clear_frame = (target == "mapping" && mapping) ||
        (target == "localization" && nav) ||
        (target == "navigation" && navigation) ||
        (target == "localization_start" && localization_start) ||
        (target == "selection" && (localization_start || nav_path || nav_pose));
      if (!clear_frame && !(target == "selection" && nav)) {
        continue;
      }
      ++resource.generation;
      changed.push_back(key);
      resource.last_cloud_decode.reset();
      if (resource.pending_cloud) {
        pending.push_back(std::move(*resource.pending_cloud));
        resource.pending_cloud.reset();
      }
      if (clear_frame) {
        previous.push_back(std::move(resource.frame));
        ++resource.version;
      }
    }
  }
  notify_watchers(changed);
}

json Scene::summary(const Resource & resource, double now) const
{
  const auto & frame = resource.frame;
  if (!frame) {
    return {{"status", "waiting"}, {"age", nullptr}, {"stamp", nullptr}, {"frame_id", ""}, {"count", 0}};
  }
  const double age = std::max(0.0, now - frame->stamp);
  return {{"status", resource.static_source ? "loaded" : (age > stale_timeout_ ? "stale" : "active")},
    {"age", age}, {"stamp", frame->stamp}, {"frame_id", frame->frame_id}, {"count", frame->count()}};
}

json Scene::state(bool mapping) const
{
  const auto now = wall_time();
  const std::string group = mapping ? "mapping" : "nav";
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & odom = resources_.at({"pose", group});
  json payload = {{"layers", json::object()}, {"odom", summary(odom, now)},
    {"map_frame", map_frame_}, {"server_time", now}};
  payload["odom"]["pose"] = odom.frame ? odom.frame->pose : json(nullptr);
  for (const auto & [key, resource] : resources_) {
    if (key.first == group) {
      payload["layers"][key.second] = summary(resource, now);
    }
  }
  if (mapping) {
    payload["path"] = summary(resources_.at({"path", "mapping_path"}), now);
  } else {
    payload["paths"] = json::object();
    for (const auto & [key, resource] : resources_) {
      if (key.first == "path" && key.second != "mapping_path") {
        payload["paths"][key.second] = summary(resource, now);
      }
    }
    const bool mapped = bool(resources_.at({"nav", "mapground"}).frame);
    payload["navigation_ready"] = !odom.frame ?
      json{{"ready", false}, {"status", "waiting_for_relocalization"}, {"message", "waiting for relocalization"}} :
      (!mapped ? json{{"ready", false}, {"status", "waiting_for_navigation"}, {"message", "waiting for navigation"}} :
      json{{"ready", true}, {"status", "ready"}, {"message", "ready"}});
    payload["trajectory_topic"] = trajectory_;
    payload["probe_trajectory_topic"] = probe_trajectory_;
  }
  return payload;
}

HttpResponse Scene::cloud(const Key & key, bool binary, bool project_2d)
{
  json header;
  std::shared_ptr<const Frame> frame;
  std::size_t generation = 0;
  const double now = wall_time();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = resources_.find(key);
    if (found == resources_.end()) {
      throw std::invalid_argument("unknown cloud layer: " + key.second);
    }
    auto & resource = found->second;
    resource.demand = Clock::now();
    frame = resource.frame;
    header = summary(resource, now);
    header.erase("age");
    header.erase("stamp");
    header["layer"] = key.second;
    header["version"] = resource.version;
    generation = resource.generation;
    header["generation"] = generation;
    header["epoch"] = epoch_;
  }
  static const std::vector<float> empty;
  const auto & points = frame ? frame->points : empty;
  const bool triangles = frame && !frame->colors.empty();
  const auto content_type = binary ? "application/octet-stream" : "application/json; charset=utf-8";
  const auto encode = [&]() {
      if (binary) {
        header["format"] = triangles ? "triangles" : "points";
        return std::make_shared<const std::string>(
          cloud_response(header, points, triangles ? &frame->colors : nullptr));
      }
      if (triangles && !project_2d) {
        header["points"] = points;
        header["colors"] = frame->colors;
        header["format"] = "triangles";
      } else {
        header["points"] = path_points(points, project_2d);
      }
      return HttpResponse::json(std::move(header)).body;
    };
  if (!frame) {
    return HttpResponse::bytes(encode(), content_type);
  }

  // Each immutable frame owns its own cached encodings. Old requests may finish
  // after a clear without inserting obsolete data into the current frame.
  const std::string status = header.at("status");
  auto & cached = frame->responses[binary ? 0 : (project_2d ? 1 : 2)];
  std::lock_guard<std::mutex> lock(cached.mutex);
  if (cached.body && cached.generation == generation && cached.status == status) {
    return HttpResponse::bytes(cached.body, content_type);
  }
  auto body = encode();
  if (generation >= cached.generation) {
    cached.body = body;
    cached.generation = generation;
    cached.status = status;
  }
  return HttpResponse::bytes(std::move(body), content_type);
}

HttpResponse Scene::path(const std::string & layer) const
{
  json header;
  std::shared_ptr<const Frame> frame;
  const auto now = wall_time();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = resources_.find({"path", layer});
    if (found == resources_.end()) {
      throw std::invalid_argument("unknown path topic: " + layer);
    }
    frame = found->second.frame;
    header = summary(found->second, now);
    header.erase("age");
    header.erase("stamp");
    header["topic"] = layer;
    header["version"] = found->second.version;
    header["generation"] = found->second.generation;
    header["epoch"] = epoch_;
  }
  header["points"] = frame ? path_points(frame->points) : json::array();
  return HttpResponse::json(std::move(header));
}

std::vector<Scene::Key> Scene::event_keys(const Query & query) const
{
  const auto requested_group = trim(query_value(query, "group"));
  const std::string group = requested_group.empty() ? "nav" : requested_group;
  std::set<Key> keys;
  std::lock_guard<std::mutex> lock(mutex_);
  if (query_value(query, "clouds") != "0") {
    if (group != "nav" && group != "mapping") {
      throw std::invalid_argument("unknown cloud group: " + group);
    }
    const auto layers = split(query_value(query, "layers"));
    if (layers.size() > (group == "nav" ? 7 : 2)) {
      throw std::invalid_argument("too many cloud layers requested");
    }
    for (const auto & layer : layers) {
      keys.emplace(group, layer);
    }
    if (layers.empty()) {
      for (const auto & [key, resource] : resources_) {
        if (key.first == group) {
          keys.insert(key);
        }
      }
    }
  }
  for (const auto & topic : split(query_value(query, "paths"))) {
    keys.emplace("path", topic);
  }
  const auto poses = split(query_value(query, "pose"));
  for (const auto & pose : poses) {
    if (pose == "1" || pose == "true" || pose == "both") {
      keys.emplace("pose", "nav");
      keys.emplace("pose", "mapping");
    } else if (pose != "0" || poses.size() != 1) {
      keys.emplace("pose", pose);
    }
  }
  for (const auto & key : keys) {
    if (!resources_.count(key)) {
      throw std::invalid_argument("unknown metadata resource: " + key.second);
    }
  }
  if (keys.empty()) {
    throw std::invalid_argument("empty metadata subscription");
  }
  return {keys.begin(), keys.end()};
}

std::shared_ptr<void> Scene::watch(std::vector<Key> keys, std::function<void()> wake)
{
  auto watcher = std::make_shared<StreamWatcher>(StreamWatcher{std::move(keys), std::move(wake)});
  std::lock_guard<std::mutex> lock(watchers_mutex_);
  watchers_.erase(std::remove_if(watchers_.begin(), watchers_.end(),
    [](const auto & entry) {return entry.expired();}), watchers_.end());
  watchers_.push_back(watcher);
  return watcher;
}

void Scene::notify_watchers(const std::vector<Key> & changed)
{
  if (changed.empty()) {
    return;
  }
  std::vector<std::shared_ptr<StreamWatcher>> interested;
  {
    std::lock_guard<std::mutex> lock(watchers_mutex_);
    for (auto entry = watchers_.begin(); entry != watchers_.end();) {
      if (auto watcher = entry->lock()) {
        const bool matches = std::any_of(changed.begin(), changed.end(), [&](const auto & key) {
            return std::binary_search(watcher->keys.begin(), watcher->keys.end(), key);
          });
        if (matches) {
          interested.push_back(std::move(watcher));
        }
        ++entry;
      } else {
        entry = watchers_.erase(entry);
      }
    }
  }
  for (const auto & watcher : interested) {
    try {
      watcher->wake();
    } catch (const std::exception &) {
      // A disconnected stream must not interrupt an ROS callback.
    }
  }
}

HttpResponse Scene::events(const Query & query)
{
  const auto keys = event_keys(query);
  return HttpResponse::stream([this, keys, initial = true, seen = std::map<Key, json>()]() mutable
    {
      auto updates = json::array();
      const auto now = wall_time();
      const auto demand = Clock::now();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & key : keys) {
          auto & resource = resources_.at(key);
          if (key.first == "nav" || key.first == "mapping") {
            resource.demand = demand;
          }
          auto entry = summary(resource, now);
          const auto stamp = entry["stamp"];
          const auto age = entry["age"];
          entry.erase("stamp");
          entry.erase("age");
          entry["group"] = key.first;
          entry["layer"] = key.second;
          entry["version"] = resource.version;
          entry["generation"] = resource.generation;
          if (entry != seen[key]) {
            seen[key] = entry;
            entry["seq"] = ++metadata_sequence_;
            if (key.first == "pose" && resource.frame) {
              entry["pose"] = resource.frame->pose;
              entry["pose"]["stamp"] = stamp;
              entry["pose"]["age"] = age;
            }
            updates.push_back(std::move(entry));
          }
        }
      }
      if (!initial && updates.empty()) {
        return std::string();
      }
      const std::string event = initial ? "snapshot" : "cloud";
      initial = false;
      return "event: " + event + "\ndata: " +
        json{{"epoch", epoch_}, {"layers", std::move(updates)}}.dump(
        -1, ' ', false, json::error_handler_t::replace) + "\n\n";
    }, [this, keys](std::function<void()> wake) {
      return watch(keys, std::move(wake));
    });
}

std::optional<HttpResponse> Scene::route(const HttpRequest & request)
{
  if (request.method != "GET") {
    return std::nullopt;
  }
  const auto & endpoint = request.path;
  if (endpoint == "/api/cloud/events") {
    return events(request.query);
  }
  if (endpoint == "/api/map3d" || endpoint == "/api/map3d.bin" || endpoint == "/api/map2d" ||
    endpoint == "/api/mapping/map3d" || endpoint == "/api/mapping/map3d.bin")
  {
    const bool mapping = endpoint.find("/mapping/") != std::string::npos;
    const std::string & selection = query_value(request.query, "layer");
    return cloud({mapping ? "mapping" : "nav", selection.empty() ?
      (mapping ? "global_map" : "local_map") : selection},
      endpoint.find(".bin") != std::string::npos, endpoint == "/api/map2d");
  }
  if (endpoint == "/api/path" || endpoint == "/api/mapping/path") {
    return path(endpoint == "/api/mapping/path" ? "mapping_path" : query_value(request.query, "topic"));
  }
  if (endpoint == "/api/pose") {
    const auto now = wall_time();
    std::lock_guard<std::mutex> lock(mutex_);
    json payload{{"epoch", epoch_}, {"map_frame", map_frame_}, {"server_time", now}};
    for (const auto & key : {Key{"pose", "nav"}, Key{"pose", "mapping"}}) {
      const auto & resource = resources_.at(key);
      auto summary_payload = summary(resource, now);
      summary_payload["pose"] = resource.frame ? resource.frame->pose : json(nullptr);
      summary_payload["version"] = resource.version;
      summary_payload["generation"] = resource.generation;
      summary_payload["epoch"] = epoch_;
      payload[key.second == "nav" ? "odom" : "mapping_odom"] = std::move(summary_payload);
    }
    return HttpResponse::json(std::move(payload));
  }
  return std::nullopt;
}
}  // namespace web_control
