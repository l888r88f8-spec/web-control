#include "diagnostics.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unistd.h>

namespace
{
using namespace std::chrono_literals;
using Json = nlohmann::json;
using DriverStats = diagnostic_msgs::msg::DiagnosticArray;
using Odom = nav_msgs::msg::Odometry;
using Imu = sensor_msgs::msg::Imu;

void require(bool condition, const char * problem)
{
  if (!condition) {
    throw std::runtime_error(problem);
  }
}

template<typename Predicate>
void wait_for(rclcpp::Executor & executor, Predicate predicate)
{
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  require(predicate(), "timed out waiting for diagnostics fixture");
}

struct Directory
{
  Directory()
  {
    char name[] = "/tmp/web-diagnostics-test.XXXXXX";
    const auto created = mkdtemp(name);
    require(created, "cannot create temporary test folder");
    path = created;
  }

  ~Directory()
  {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }

  std::filesystem::path path;
};

void run(rclcpp::Context::SharedPtr context)
{
  Directory directory;
  const auto prefix = "/web_diagnostics_test_" + std::to_string(getpid());
  rclcpp::NodeOptions options;
  options.context(context);
  auto node = std::make_shared<rclcpp::Node>("diagnostics", prefix, options);
  auto fixture = std::make_shared<rclcpp::Node>("fixture", prefix, options);
  node->declare_parameter("diagnostics_imu_topic", prefix + "/imu");
  node->declare_parameter("diagnostics_lidar_topic", prefix + "/cloud");
  node->declare_parameter("diagnostics_lidar_stats_topic", prefix + "/cloud_stats");
  node->declare_parameter("diagnostics_chassis_odom_topic", prefix + "/odom");
  node->declare_parameter("diagnostics_sample_period_sec", 0.1);
  node->declare_parameter("motion_debounce_sec", 0.0);

  const auto qos = rclcpp::QoS(5).best_effort();
  const auto raw = fixture->create_publisher<sensor_msgs::msg::PointCloud2>(prefix + "/cloud", qos);
  const auto stats = fixture->create_publisher<DriverStats>(prefix + "/cloud_stats", qos);
  const auto odom = fixture->create_publisher<Odom>(prefix + "/odom", qos);
  const auto imu = fixture->create_publisher<Imu>(prefix + "/imu", qos);
  web_control::Diagnostics diagnostics(*node, directory.path, true);
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(node);
  executor.add_node(fixture);
  wait_for(executor, [&]() {
    return stats->get_subscription_count() && odom->get_subscription_count() &&
           imu->get_subscription_count();
  });
  require(raw->get_subscription_count() == 0, "raw LiDAR subscription enabled by default");
  wait_for(executor, [&]() {
    return diagnostics.snapshot()["topics"][1]["status"] == "publisher_only";
  });
  require(diagnostics.snapshot()["topics"][1]["hz"].is_null(), "invented unmeasured LiDAR rate");

  DriverStats report;
  auto & status = report.status.emplace_back();
  status.name = "hesai_lidar_publish";
  status.hardware_id = prefix + "/cloud";
  const Json fields = {
    {"source_topic", prefix + "/cloud"}, {"publish_rate_hz", "12.5"},
    {"window_sec", "2"}, {"last_publish_age_sec", "0.1"}, {"frames_in_window", "25"},
    {"total_frames", "100"}, {"sequence", "3"}, {"session_id", "fixture"}
  };
  for (const auto & [key, value] : fields.items()) {
    auto & field = status.values.emplace_back();
    field.key = key;
    field.value = value.get<std::string>();
  }
  stats->publish(report);
  wait_for(executor, [&]() {return diagnostics.snapshot()["lidar_rate_source"] == "driver_publish";});
  require(diagnostics.snapshot()["topics"][1]["hz"] == 12.5, "driver LiDAR rate not used");
  stats->publish(report);
  wait_for(executor, [&]() {
    return diagnostics.snapshot()["lidar_publish_stats"]["duplicate_count"] == 1;
  });

  Imu observation;
  observation.linear_acceleration.x = 2.0;
  imu->publish(observation);
  wait_for(executor, [&]() {return diagnostics.snapshot()["imu_observation"].is_object();});
  auto snapshot = diagnostics.snapshot();
  require(snapshot["imu_observation"]["motion_proof"] == false &&
    snapshot["motion"]["total_distance_m"].is_null(), "IMU incorrectly established wheel motion");

  diagnostics.set_logging(false);
  Odom sample;
  sample.header.frame_id = "odom";
  sample.child_frame_id = "base";
  sample.header.stamp.sec = 2;
  sample.pose.pose.orientation.w = 1.0;
  sample.twist.twist.linear.x = 1.0;
  odom->publish(sample);
  wait_for(executor, [&]() {return diagnostics.snapshot()["motion"]["state"] == "moving";});
  sample.header.stamp.nanosec = 400000000;
  sample.twist.twist.linear.x = -1.0;
  odom->publish(sample);
  wait_for(executor, [&]() {return diagnostics.snapshot()["motion"]["direction"] == "reverse";});
  require(std::abs(diagnostics.snapshot()["motion"]["total_distance_m"].get<double>() - 0.2) < 1e-6,
    "distance did not integrate the absolute speed across a reversal");
  diagnostics.chassis_lifecycle(false);
  snapshot = diagnostics.snapshot();
  require(snapshot["motion"]["reason"] == "chassis_stop" &&
    snapshot["motion"]["validity"]["valid"] == false &&
    snapshot["topic_logging_enabled"] == false, "lifecycle or logging toggle not reported");

  const auto logfile = diagnostics.session()["path"].get<std::string>();
  diagnostics.shutdown();
  std::ifstream input(logfile);
  std::string text;
  bool disabled = false;
  bool motion = false;
  bool shutdown = false;
  while (std::getline(input, text)) {
    if (text.empty() || text.front() != '{') {
      continue;
    }
    const auto row = Json::parse(text);
    if (row["kind"] == "logging_toggle") {
      disabled = true;
    }
    if (disabled) {
      require(!row.contains("topics") && !row.contains("imu_observation") &&
        !row.contains("publisher_count") && !row.contains("lidar_stats_topic") &&
        row["kind"] != "topic_status",
        "topic metrics written after logging disabled");
      motion = motion || row["kind"] == "motion";
      shutdown = shutdown || row["kind"] == "shutdown";
    }
  }
  require(disabled && motion && shutdown, "expected diagnostics JSONL audit records missing");
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    auto context = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init;
    init.set_domain_id(101 + getpid() % 30);
    context->init(argc, argv, init);
    run(context);
    context->shutdown("test complete");
    return EXIT_SUCCESS;
  } catch (const std::exception & problem) {
    std::cerr << problem.what() << '\n';
    return EXIT_FAILURE;
  }
}
