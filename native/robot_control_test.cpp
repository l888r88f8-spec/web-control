#include "robot_control.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

namespace
{
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using Json = nlohmann::json;
using StatusArray = action_msgs::msg::GoalStatusArray;
using Cancel = action_msgs::srv::CancelGoal;
using Goal = geometry_msgs::msg::PoseStamped;
using Twist = geometry_msgs::msg::Twist;

void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

template<typename Predicate>
void wait_for(rclcpp::Executor & executor, Predicate predicate)
{
  const auto deadline = Clock::now() + 3s;
  while (!predicate() && Clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  require(predicate(), "timed out waiting for test condition");
}

struct TemporaryDirectory
{
  TemporaryDirectory()
  {
    char name[] = "/tmp/web-robot-control-test.XXXXXX";
    const auto result = mkdtemp(name);
    if (!result) {
      throw std::runtime_error("cannot create test fixture directory");
    }
    path = result;
  }

  ~TemporaryDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }

  std::filesystem::path path;
};

void test(rclcpp::Context::SharedPtr context)
{
  TemporaryDirectory directory;
  const auto record = (directory.path / "paused_navigation.json").string();
  const std::string prefix = "/web_control_test_" + std::to_string(getpid());
  rclcpp::NodeOptions options;
  options.context(context);
  auto adapter = std::make_shared<rclcpp::Node>("adapter", prefix, options);
  auto fixture = std::make_shared<rclcpp::Node>("fixture", prefix, options);
  adapter->declare_parameter("teleop_cmd_vel_topic", prefix + "/commands");
  adapter->declare_parameter("goal_topic", prefix + "/goals");
  adapter->declare_parameter("action_status_topic", prefix + "/status");
  adapter->declare_parameter("cancel_goal_service", prefix + "/cancel");
  adapter->declare_parameter("navigation_record_path", record);
  adapter->declare_parameter("teleop_command_timeout", 0.12);
  adapter->declare_parameter("teleop_publish_rate", 50.0);
  adapter->declare_parameter("viewer_timeout_sec", 0.10);

  std::vector<Twist> commands;
  std::vector<Goal> goals;
  auto command_listener = fixture->create_subscription<Twist>(
    prefix + "/commands", 10,
    [&](const Twist & message) {commands.push_back(message);});
  auto goal_listener = fixture->create_subscription<Goal>(
    prefix + "/goals", 5,
    [&](const Goal & message) {goals.push_back(message);});
  auto status_pub = fixture->create_publisher<StatusArray>(prefix + "/status", 10);
  int cancel_requests = 0;
  auto cancel_service = fixture->create_service<Cancel>(
    prefix + "/cancel",
    [&](const Cancel::Request::SharedPtr request, Cancel::Response::SharedPtr response) {
      require(request->goal_info.goal_id.uuid == action_msgs::msg::GoalInfo{}.goal_id.uuid,
        "expected a zero cancellation UUID");
      ++cancel_requests;
      response->goals_canceling.emplace_back();
    });
  web_control::RobotControl robot(*adapter);
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(adapter);
  executor.add_node(fixture);
  wait_for(executor, [&]() {
    return command_listener->get_publisher_count() && goal_listener->get_publisher_count() &&
           status_pub->get_subscription_count();
  });

  const std::optional<std::string> map = "demo";
  auto route = [&](std::string_view path, const Json & payload) {
      const auto response = robot.route(path, payload, map);
      require(response.has_value(), "route missing");
      return *response;
    };
  auto control = [&]() {return robot.status(map).at("navigation_control");};
  auto command = [&](int sequence, double throttle, double steering) {
      return route("/api/teleop",
        {{"client_id", "tab-1"}, {"seq", sequence}, {"throttle", throttle}, {"steering", steering}});
    };

  require(route("/api/viewer", {{"viewer_id", "tab-1"}}).body["viewers"] == 1, "viewer missing");
  wait_for(executor, [&]() {return robot.status(map).at("viewers") == 0;});
  auto reverse = command(1, -2.0, 1.0);
  require(reverse.status == 200 && reverse.body["teleop"]["linear_x"] == -1.5,
    "reverse command did not clamp");
  const double angular = reverse.body["teleop"]["angular_z"];
  require(std::abs(angular - (-1.5 / 0.550) * std::tan(0.3270520925102273)) < 1e-10,
    "reverse steering sign or magnitude changed");
  wait_for(executor, [&]() {return !commands.empty() && commands.back().linear.x < 0;});
  require(command(3, 0, 0).status == 200, "release rejected");
  require(command(2, 1, 1).body["stale"] == true, "late command rearmed after release");
  wait_for(executor, [&]() {return commands.back().linear.x == 0;});
  require(command(4, 1, 0).status == 200, "new command rejected");
  wait_for(executor, [&]() {return commands.back().linear.x > 0;});
  wait_for(executor, [&]() {return commands.back().linear.x == 0;});
  require(command(4, 1, 0).body["stale"] == true, "watchdog cleared command sequence");

  auto send_status = [&](int value, int seconds) {
      StatusArray message;
      auto & status = message.status_list.emplace_back();
      status.status = value;
      status.goal_info.stamp.sec = seconds;
      status_pub->publish(message);
      executor.spin_some();
    };
  require(route("/api/goal", {{"x", 1}, {"y", 2}, {"yaw", 1.0}}).status == 200,
    "goal rejected");
  wait_for(executor, [&]() {return goals.size() == 1;});
  require(goals[0].header.frame_id == "map" &&
    std::abs(goals[0].pose.orientation.z - std::sin(0.5)) < 1e-10, "invalid published pose");
  send_status(2, 1);
  wait_for(executor, [&]() {return control()["can_pause"] == true;});
  auto pause = std::async(std::launch::async, [&]() {
    return route("/api/navigation/pause", Json::object());
  });
  wait_for(executor, [&]() {return pause.wait_for(0ms) == std::future_status::ready;});
  require(pause.get().status == 200 && cancel_requests == 1, "pause did not request cancellation");
  send_status(5, 1);
  wait_for(executor, [&]() {return control()["state"] == "paused";});
  Json persisted;
  std::ifstream(record) >> persisted;
  require(persisted["phase"] == "paused" && persisted["map_name"] == "demo",
    "paused destination not persisted");
  require(route("/api/navigation/resume", Json::object()).status == 200, "resume rejected");
  wait_for(executor, [&]() {return goals.size() == 2;});
  send_status(5, 1);
  require(control()["state"] == "resuming", "old terminal status settled the new goal");
  send_status(2, 2);
  wait_for(executor, [&]() {return control()["state"] == "navigating";});
  require(!std::filesystem::exists(record), "confirmed resume retained the pause record");
  robot.clear_action_status();
  require(robot.status(map)["navigation_action"]["status"] == "waiting" &&
    control()["state"] == "navigating", "action display reset altered navigation control");
  robot.shutdown();
  require(command(5, 1, 0).status != 200, "shutdown allowed new movement");
}
}  // namespace

int main()
{
  try {
    auto context = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions options;
    options.set_domain_id(101 + getpid() % 30);
    context->init(0, nullptr, options);
    test(context);
    context->shutdown("test complete");
    return EXIT_SUCCESS;
  } catch (const std::exception & problem) {
    std::cerr << problem.what() << '\n';
    return EXIT_FAILURE;
  }
}
