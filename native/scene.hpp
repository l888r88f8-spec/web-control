#pragma once

#include <array>
#include <map>
#include <mutex>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "http_server.hpp"

namespace web_control
{
class Scene
{
public:
  explicit Scene(rclcpp::Node & node);
  const rclcpp::CallbackGroup::SharedPtr & cloud_group() const {return cloud_group_;}
  std::optional<HttpResponse> route(const HttpRequest & request);
  nlohmann::json state(bool mapping) const;
  void clear(const std::string & target);

private:
  using Key = std::pair<std::string, std::string>;
  using Clock = std::chrono::steady_clock;
  using CloudMessage = std::variant<sensor_msgs::msg::PointCloud2::ConstSharedPtr,
    visualization_msgs::msg::MarkerArray::ConstSharedPtr>;

  struct PendingCloud
  {
    CloudMessage message;
    double stamp;
    std::size_t generation;
  };

  struct EncodedCloud
  {
    std::mutex mutex;
    std::shared_ptr<const std::string> body;
    std::string status;
    std::size_t generation = 0;
  };

  struct Frame
  {
    std::vector<float> points;
    std::vector<float> colors;
    std::string frame_id;
    nlohmann::json pose;
    double stamp = 0;
    mutable std::array<EncodedCloud, 3> responses;

    std::size_t count() const {return points.size() / 3;}
  };
  struct Resource
  {
    std::string topic;
    bool latched = false;
    bool keep_subscription = false;
    bool static_source = false;
    std::size_t generation = 0;
    std::size_t version = 0;
    std::shared_ptr<const Frame> frame;
    std::optional<Clock::time_point> demand;
    std::optional<Clock::time_point> last_cloud_decode;
    std::optional<Clock::time_point> subscribe_retry;
    std::optional<PendingCloud> pending_cloud;
    rclcpp::SubscriptionBase::SharedPtr subscription;
  };

  struct StreamWatcher
  {
    std::vector<Key> keys;
    std::function<void()> wake;
  };

  void add_cloud(Key key, const std::string & parameter, const std::string & topic,
    bool latched, bool keep_subscription = false, bool static_source = false);
  void add_pose(const std::string & layer, const std::string & topic);
  void add_path(const std::string & layer, const std::string & topic, bool pose_array);
  void reconcile();
  bool demand_active(const Resource & resource, Clock::time_point now) const;
  bool cloud_decode_ready(const Resource & resource, Clock::time_point now) const;
  rclcpp::SubscriptionBase::SharedPtr subscribe_cloud(const Key & key, const Resource & resource);
  void publish(const Key & key, std::size_t generation, std::shared_ptr<const Frame> frame);
  std::size_t generation(const Key & key) const;
  void receive_cloud(const Key & key, CloudMessage message);
  void decode_cloud(const Key & key, const PendingCloud & pending);
  nlohmann::json summary(const Resource & resource, double now) const;
  HttpResponse cloud(const Key & key, bool binary, bool project_2d);
  HttpResponse path(const std::string & layer) const;
  HttpResponse events(const Query & query);
  std::vector<Key> event_keys(const Query & query) const;
  std::shared_ptr<void> watch(std::vector<Key> keys, std::function<void()> wake);
  void notify_watchers(const std::vector<Key> & changed);

  rclcpp::Node & node_;
  rclcpp::CallbackGroup::SharedPtr cloud_group_;
  std::string epoch_;
  std::string map_frame_;
  std::string trajectory_;
  std::string probe_trajectory_;
  double stale_timeout_;
  double demand_ttl_;
  double idle_timeout_;
  mutable std::mutex mutex_;
  std::map<Key, Resource> resources_;
  std::uint64_t metadata_sequence_ = 0;
  std::mutex watchers_mutex_;
  std::vector<std::weak_ptr<StreamWatcher>> watchers_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
