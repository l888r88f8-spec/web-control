#pragma once

#include <filesystem>
#include <memory>

#include <nlohmann/json.hpp>

namespace rclcpp
{
class Node;
}

namespace web_control
{
class Diagnostics
{
public:
  Diagnostics(rclcpp::Node & node, const std::filesystem::path & log_directory, bool logging_enabled);
  ~Diagnostics();
  nlohmann::json snapshot() const;
  nlohmann::json session() const;
  void set_logging(bool enabled);
  void chassis_lifecycle(bool started);
  void shutdown();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace web_control
