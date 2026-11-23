#pragma once

#include <map>
#include <mutex>
#include <string>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace web_control
{
class TfTree
{
public:
  explicit TfTree(rclcpp::Node & node);
  nlohmann::json snapshot() const;

private:
  struct Edge
  {
    std::string parent;
    std::string child;
    bool static_transform;
    double stamp;
    double last_seen;
    std::uint64_t order;
  };

  void receive(const tf2_msgs::msg::TFMessage & message, bool static_transform);

  rclcpp::Node & node_;
  std::string topic_;
  std::string static_topic_;
  std::size_t limit_;
  double stale_timeout_;
  mutable std::mutex mutex_;
  std::map<std::string, Edge> edges_;
  std::uint64_t sequence_ = 0;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr dynamic_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_subscription_;
};
}
