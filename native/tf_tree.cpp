#include "tf_tree.hpp"
#include "wall_clock.hpp"

#include <algorithm>
#include <functional>
#include <set>
#include <tuple>

namespace web_control
{
using nlohmann::json;
using tf2_msgs::msg::TFMessage;

TfTree::TfTree(rclcpp::Node & node)
: node_(node),
  topic_(node.declare_parameter<std::string>("tf_topic", "/tf")),
  static_topic_(node.declare_parameter<std::string>("tf_static_topic", "/tf_static")),
  limit_(std::max<std::int64_t>(1, node.declare_parameter("tf_cache_size", 4096))),
  stale_timeout_(node.declare_parameter("tf_stale_timeout_sec", 2.0))
{
  if (!topic_.empty()) {
    dynamic_subscription_ = node_.create_subscription<TFMessage>(
      topic_, rclcpp::QoS(100).best_effort(), [this](TFMessage::ConstSharedPtr message) {
        receive(*message, false);
      });
  }
  if (!static_topic_.empty()) {
    static_subscription_ = node_.create_subscription<TFMessage>(
      static_topic_, rclcpp::QoS(100).reliable().transient_local(),
      [this](TFMessage::ConstSharedPtr message) {receive(*message, true);});
  }
}

void TfTree::receive(const TFMessage & message, bool static_transform)
{
  const auto now = wall_time();
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & transform : message.transforms) {
    const auto & child = transform.child_frame_id;
    const auto key = child.empty() ? "__malformed_" + std::to_string(sequence_ + 1) : child;
    auto previous = edges_.find(key);
    if (previous == edges_.end() && edges_.size() >= limit_) {
      const auto oldest = std::min_element(edges_.begin(), edges_.end(), [](const auto & a, const auto & b) {
        return std::tie(a.second.static_transform, a.second.order) <
               std::tie(b.second.static_transform, b.second.order);
      });
      edges_.erase(oldest);
    }
    const auto order = previous == edges_.end() ? ++sequence_ : previous->second.order;
    auto parent = transform.header.frame_id;
    if (parent.empty() && child.empty()) {
      parent = "?";
    }
    edges_.insert_or_assign(key, Edge{std::move(parent), child, static_transform,
        transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9, now, order});
  }
}

json TfTree::snapshot() const
{
  std::map<std::string, Edge> copied;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    copied = edges_;
  }
  const auto now = wall_time();
  const auto view = [now, this](const Edge & edge) {
      const auto age = std::max(0.0, now - edge.last_seen);
      return json{{"parent", edge.parent}, {"child", edge.child}, {"static", edge.static_transform},
        {"stamp", edge.stamp}, {"last_seen", edge.last_seen}, {"age", age},
        {"stale", !edge.static_transform && age > stale_timeout_}};
    };
  std::set<std::string> frames;
  std::map<std::string, Edge> parents;
  std::map<std::string, std::set<std::string>> children;
  json detached = json::array();
  for (const auto & [key, edge] : copied) {
    if (!edge.parent.empty()) {
      frames.insert(edge.parent);
    }
    if (!edge.child.empty()) {
      frames.insert(edge.child);
    }
    if (edge.parent.empty() || edge.child.empty() || edge.parent == edge.child) {
      auto row = view(edge);
      row["reason"] = edge.parent == edge.child ? "self cycle" : "malformed frame";
      detached.push_back(std::move(row));
    } else {
      parents.emplace(edge.child, edge);
      children[edge.parent].insert(edge.child);
    }
  }
  std::set<std::string> visited;
  std::set<std::string> stack;
  std::function<json(const std::string &, unsigned)> build =
    [&](const std::string & frame, unsigned depth) {
      visited.insert(frame);
      stack.insert(frame);
      json node{{"frame", frame}, {"children", json::array()}, {"parent", nullptr}, {"edge", nullptr}};
      for (const auto & child : children[frame]) {
        auto edge = view(parents.at(child));
        const char * reason = stack.count(child) ? "cycle" : visited.count(child) ? "extra parent" :
          depth >= 128 ? "tree too deep" : nullptr;
        if (reason) {
          edge["reason"] = reason;
          detached.push_back(std::move(edge));
        } else {
          auto next = build(child, depth + 1);
          next["parent"] = frame;
          next["edge"] = std::move(edge);
          node["children"].push_back(std::move(next));
        }
      }
      stack.erase(frame);
      return node;
    };
  json trees = json::array();
  for (const auto & frame : frames) {
    if (!parents.count(frame) && !visited.count(frame)) {
      trees.push_back(build(frame, 1));
    }
  }
  for (const auto & frame : frames) {
    if (!visited.count(frame)) {
      trees.push_back(build(frame, 1));
    }
  }
  return {{"ok", true}, {"status", frames.empty() ? "waiting" : "active"},
    {"message", frames.empty() ? json("waiting for /tf") : json(nullptr)},
    {"stamp", now}, {"frame_count", frames.size()}, {"edge_count", copied.size()},
    {"stale_timeout_sec", stale_timeout_}, {"trees", std::move(trees)},
    {"detached", std::move(detached)}, {"topic", topic_}, {"static_topic", static_topic_},
    {"ros_available", true}};
}
}
