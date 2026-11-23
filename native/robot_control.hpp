#pragma once

#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include <nlohmann/json.hpp>

namespace rclcpp
{
class Node;
}

namespace web_control
{
struct ControlResponse
{
  int status;
  nlohmann::json body;
};

class RobotControl
{
public:
  explicit RobotControl(rclcpp::Node & node);
  ~RobotControl();

  std::optional<ControlResponse> route(
    std::string_view path, const nlohmann::json & payload,
    const std::optional<std::string> & selected_map);
  nlohmann::json status(const std::optional<std::string> & selected_map) const;
  void map_selected(const std::string & name);
  void clear_action_status();
  void navigation_stopped();
  void shutdown();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace web_control
