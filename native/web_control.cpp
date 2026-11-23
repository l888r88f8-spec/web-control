#include "catalog.hpp"
#include "diagnostics.hpp"
#include "managed_processes.hpp"
#include "robot_control.hpp"
#include "scene.hpp"
#include "system_status.hpp"
#include "tf_tree.hpp"
#include "wall_clock.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <lio_sam_hesai/srv/save_map_options.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>

#include <atomic>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fcntl.h>
#include <stdexcept>
#include <thread>
#include <unistd.h>

namespace web_control
{
namespace
{
namespace fs = std::filesystem;
using nlohmann::json;
using namespace std::chrono_literals;
using SaveMap = lio_sam_hesai::srv::SaveMapOptions;
using GetParameters = rcl_interfaces::srv::GetParameters;

std::string content_type(const fs::path & file)
{
  static const std::map<std::string, std::string> types{
    {".html", "text/html; charset=utf-8"}, {".js", "application/javascript; charset=utf-8"},
    {".css", "text/css; charset=utf-8"}, {".json", "application/json"},
    {".svg", "image/svg+xml"}, {".png", "image/png"}, {".jpg", "image/jpeg"},
    {".ico", "image/x-icon"}, {".woff2", "font/woff2"}};
  const auto found = types.find(file.extension().string());
  return found == types.end() ? "application/octet-stream" : found->second;
}

HttpResponse problem(unsigned status, std::string message)
{
  return HttpResponse::json({{"ok", false}, {"message", std::move(message)}}, status);
}

HttpResponse static_file(const fs::path & root, const std::string & uri)
{
  const auto requested = uri == "/" ? "index.html" : uri.substr(1);
  std::error_code error;
  const auto resolved = fs::canonical(root / requested, error);
  if (error || root.empty() || !fs::is_regular_file(resolved, error)) {
    return problem(404, "file not found");
  }
  const auto relative = resolved.lexically_relative(root);
  if (relative.empty() || *relative.begin() == "..") {
    return problem(404, "file not found");
  }
  const int descriptor = ::open(resolved.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (descriptor < 0) {
    return problem(404, "file not found");
  }
  struct Close
  {
    int descriptor;
    ~Close() {::close(descriptor);}
  } close{descriptor};
  return HttpResponse::file(descriptor, content_type(resolved));
}

std::string default_static_directory()
{
  try {
    const auto installation = fs::path(ament_index_cpp::get_package_share_directory("web_control")) /
      "static";
    if (fs::is_directory(installation)) {
      // ament's symlink install can link individual assets back to the build
      // source tree. Recognize that known tree without allowing arbitrary
      // symlink targets outside the selected static root.
      std::error_code index_error;
      const auto index = fs::canonical(installation / "index.html", index_error);
      std::error_code source_error;
      const auto source = fs::canonical(WEB_CONTROL_STATIC_DIR, source_error);
      if (!index_error && !source_error && index == source / "index.html") {
        return source.string();
      }
      return installation.string();
    }
  } catch (const ament_index_cpp::PackageNotFoundError &) {
  }
  return WEB_CONTROL_STATIC_DIR;
}

std::int64_t integer_query(const Query & query, const std::string & key, std::int64_t fallback)
{
  const auto & value = query_value(query, key);
  if (value.empty()) {
    return fallback;
  }
  std::size_t length;
  try {
    const auto number = std::stoll(value, &length);
    if (length == value.size()) {
      return number;
    }
  } catch (const std::logic_error &) {
  }
  throw std::invalid_argument("invalid integer parameter: " + key);
}

class WebControlNative final : public rclcpp::Node
{
public:
  WebControlNative()
  : Node("web_control"), scene_(*this), tf_(*this), robot_(*this)
  {
    const auto host = declare_parameter<std::string>("host", "0.0.0.0");
    const auto port = declare_parameter<int>("port", 8080);
    const auto preview = declare_parameter<int>("preview_port", 0);
    if (port < 0 || port > 65535 || preview < 0 || preview > 65535) {
      throw std::invalid_argument("HTTP port must be in the range 0-65535");
    }
    std::error_code error;
    static_dir_ = fs::canonical(
      declare_parameter<std::string>("static_directory", default_static_directory()), error);
    ManagedProcessConfig managed_config;
    for (auto & [name, launch] : managed_config.launches) {
      launch.package = declare_parameter(name + "_launch_package", launch.package);
      launch.file = declare_parameter(name + "_launch_file", launch.file);
      launch.args = declare_parameter(name + "_launch_args", launch.args);
    }
    managed_config.max_lines = static_cast<std::size_t>(
      std::max<std::int64_t>(1, declare_parameter("managed_log_lines", 400)));
    managed_config.log_directory = managed_log_directory(
      declare_parameter<std::string>("managed_log_dir", ""));
    const auto save_map_service = declare_parameter<std::string>(
      "save_map_service", "/lio_sam/save_map_options");
    if (!save_map_service.empty()) {
      save_map_ = create_client<SaveMap>(save_map_service);
    }
    CatalogConfig catalog_config;
    catalog_config.clock = []() {return wall_time();};
    catalog_config.recording_root = declare_parameter<std::string>("recording_root", "/");
    catalog_config.topics = [this]() {
        const auto topics = get_topic_names_and_types();
        return CatalogConfig::TopicGraph(topics.begin(), topics.end());
      };
    catalog_config.managed_running = [this](const auto & target) {
        return processes_ && processes_->running(target);
      };
    catalog_config.map_selected = [this](const MapSelection & selection, bool has_name) {
        if (has_name) {
          scene_.clear("selection");
          robot_.clear_action_status();
          if (selection.name) {
            robot_.map_selected(*selection.name);
          }
        }
      };
    catalog_config.save_map_available = [this]() {
        if (!save_map_) {
          return false;
        }
        const bool ready = save_map_->service_is_ready();
        if (!ready) {
          save_map_->prune_pending_requests();
        }
        return ready;
      };
    catalog_config.save_map = [this](const SaveMapRequest & data, CatalogConfig::SaveMapCompletion done) {
        if (!save_map_ || !save_map_->service_is_ready()) {
          throw std::runtime_error("save map service unavailable");
        }
        auto request = std::make_shared<SaveMap::Request>();
        request->destination = data.destination;
        request->resolution = data.resolution;
        request->ground_resolution = data.ground_resolution;
        request->save_ground_map = data.save_ground_map;
        request->save_tiled_map = data.save_tiled_map;
        save_map_->async_send_request(request,
          [done = std::move(done)](rclcpp::Client<SaveMap>::SharedFuture future) {
            try {
              const auto reply = future.get();
              done(reply->success, reply->message);
            } catch (const std::exception & error) {
              done(false, error.what());
            }
          });
      };

    catalog_ = std::make_unique<Catalog>(std::move(catalog_config));
    diagnostics_ = std::make_unique<Diagnostics>(*this, managed_config.log_directory,
        catalog_->topic_logging_enabled());
    ManagedProcessHooks hooks;
    hooks.now = []() {return wall_time();};
    hooks.map_arguments = [this](const std::string & target) {return launch_arguments(target);};
    hooks.on_start = [this](const std::string & target) {
        if (target == "chassis") {
          diagnostics_->chassis_lifecycle(true);
        }
        if (target == "localization") {
          scene_.clear("localization_start");
        }
      };
    hooks.on_exit = [this](const std::string & target) {
        if (target == "chassis") {
          diagnostics_->chassis_lifecycle(false);
        }
        scene_.clear(target);
        if (target == "navigation") {
          robot_.navigation_stopped();
        } else if (target == "localization") {
          robot_.clear_action_status();
        }
      };
    hooks.diagnostics_session = [this]() {return diagnostics_->session();};
    processes_ = std::make_unique<ManagedProcesses>(std::move(managed_config), std::move(hooks));
    configure_perception_window();
    if (preview != 0 || port != 0) {
      const auto chosen_port = preview ? static_cast<std::uint16_t>(preview) : static_cast<std::uint16_t>(port);
      auto address = preview ? std::string("127.0.0.1") : host;
      http_ = std::make_unique<HttpServer>(address, chosen_port,
          [this](HttpRequest request) {return route(std::move(request));},
          std::chrono::duration<double>(get_parameter("teleop_command_timeout").as_double()));
      RCLCPP_INFO(get_logger(), "Web control serving %s on %s:%u",
        static_dir_.c_str(), address.c_str(), chosen_port);
    }
  }

  const rclcpp::CallbackGroup::SharedPtr & cloud_group() const {return scene_.cloud_group();}

  void stop()
  {
    stopping_ = true;
    // Fence motion and publish the final zero before waiting for HTTP workers.
    // The ROS context remains valid until both executors have stopped.
    robot_.shutdown();
    http_.reset();
    processes_->shutdown();
    catalog_.reset();
  }

  void finish_diagnostics()
  {
    diagnostics_->shutdown();
  }

private:
  ManagedLaunchArguments launch_arguments(const std::string & target)
  {
    if (target != "navigation" && target != "localization") {
      return {};
    }
    const auto selection = catalog_->selection();
    if (!selection.name || !selection.entry) {
      throw std::invalid_argument("no valid map selected; choose a map before starting " + target);
    }
    if (target == "navigation") {
      return {{"map_path", selection.entry->global_map_path.parent_path().string()},
        {"planner", catalog_->navigation_planner()}};
    }
    if (!selection.effective_mode) {
      throw std::invalid_argument(selection.mode_message.empty() ?
        "selected map cannot use the configured loading mode" : selection.mode_message);
    }
    const auto path = *selection.effective_mode == "tiled" ?
      selection.entry->tiled_path.value() : selection.entry->global_map_path;
    return {{"map_path", path.string()}, {"auto_relocalization",
        selection.effective_auto_relocalization ? "true" : "false"}};
  }

  void configure_perception_window()
  {
    const auto name = declare_parameter<std::string>(
      "perception_window_service", "/shared_cloud_preprocessor/get_parameters");
    const auto period = declare_parameter("perception_window_refresh_sec", 5.0);
    if (name.empty() || !std::isfinite(period) || period <= 0) {
      return;
    }
    perception_client_ = create_client<GetParameters>(name);
    perception_timer_ = create_wall_timer(std::chrono::duration<double>(period), [this]() {
      if (!perception_client_->service_is_ready()) {
        set_perception_window({{"available", false}});
        return;
      }
      if (perception_pending_) {
        return;
      }
      auto request = std::make_shared<GetParameters::Request>();
      request->names = {"perception_window_size", "min_obstacle_height", "marking_height", "robot_base_frame"};
      perception_pending_ = true;
      try {
        perception_client_->async_send_request(request, [this](rclcpp::Client<GetParameters>::SharedFuture future) {
          perception_pending_ = false;
          json result{{"available", false}};
          try {
            const auto & values = future.get()->values;
            if (values.size() == 4 &&
              values[0].type == 3 && values[1].type == 3 && values[2].type == 3)
            {
              const auto size = values[0].double_value;
              const auto low = values[1].double_value;
              const auto high = values[2].double_value;
              if (std::isfinite(size) && size > 0 && std::isfinite(low) &&
                std::isfinite(high) && low <= high)
              {
                result = {{"available", true}, {"window", size}, {"min_height", low},
                  {"max_height", high}, {"frame", values[3].string_value.empty() ?
                    "base_link" : values[3].string_value}};
              }
            }
          } catch (const std::exception &) {
          }
          set_perception_window(std::move(result));
        });
      } catch (const std::exception &) {
        perception_pending_ = false;
        set_perception_window({{"available", false}});
      }
    });
  }

  void set_perception_window(json status)
  {
    status["updated"] = wall_time();
    std::lock_guard<std::mutex> lock(perception_mutex_);
    perception_ = std::move(status);
  }

  HttpResponse route(HttpRequest request)
  {
    if (stopping_) {
      return problem(503, "web control is shutting down");
    }
    if (request.path.empty() || request.path.front() != '/') {
      return problem(400, "invalid request path");
    }
    if (auto response = scene_.route(request)) {
      return std::move(*response);
    }
    const auto & path = request.path;
    if (request.method == "GET") {
      if (path == "/api/state") {
        auto response = scene_.state(false);
        response.update(robot_.status(catalog_->selection().name));
        response["navigation_planner"] = catalog_->navigation_planner();
        std::lock_guard<std::mutex> lock(perception_mutex_);
        response["perception_window"] = perception_;
        return HttpResponse::json(std::move(response));
      }
      if (path == "/api/mapping/state") {
        auto response = scene_.state(true);
        response["save_map"] = catalog_->save_map_status();
        response["recording"] = catalog_->recording_status();
        return HttpResponse::json(std::move(response));
      }
      if (path == "/api/tf") {
        return HttpResponse::json(tf_.snapshot());
      }
      if (path == "/api/system") {
        return HttpResponse::json(system_.snapshot(wall_time()));
      }
      if (path == "/api/diagnostics") {
        return HttpResponse::json(diagnostics_->snapshot());
      }
      if (path == "/api/logs" || path.compare(0, 10, "/api/logs/") == 0) {
        return logs(request);
      }
      if (path.compare(0, 5, "/api/") != 0) {
        return static_file(static_dir_, path);
      }
    } else if (request.method == "POST") {
      if (!request.body.is_object()) {
        throw std::invalid_argument("request must be a JSON object");
      }
      if (path == "/api/diagnostics/logging") {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        bool enabled;
        try {
          enabled = catalog_->set_topic_logging_enabled(request.body);
        } catch (const std::runtime_error & error) {
          return problem(503, error.what());
        }
        diagnostics_->set_logging(enabled);
        auto response = diagnostics_->snapshot();
        response["message"] = enabled ? "topic logging enabled" : "topic logging disabled";
        return HttpResponse::json(std::move(response));
      }
      if (path == "/api/managed/start" || path == "/api/managed/stop") {
        const auto target = request.body.value("target", std::string());
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        return HttpResponse::json(path == "/api/managed/start" ?
          processes_->start(target) : processes_->stop(target));
      }
      if (path == "/api/teleop" || path == "/api/viewer" || path == "/api/initialpose") {
        if (auto response = robot_.route(path, request.body, std::nullopt)) {
          return HttpResponse::json(std::move(response->body), response->status);
        }
      } else if (path == "/api/goal" || path == "/api/navigation/pause" ||
        path == "/api/navigation/resume" || path == "/api/navigation/clear")
      {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (auto response = robot_.route(path, request.body, catalog_->selection().name)) {
          return HttpResponse::json(std::move(response->body), response->status);
        }
      }
    } else {
      return problem(405, "method not allowed");
    }
    if (path == "/api/maps/select" || path == "/api/navigation/planner") {
      std::lock_guard<std::mutex> lock(lifecycle_mutex_);
      return catalog_route(request);
    }
    return catalog_route(request);
  }

  HttpResponse catalog_route(const HttpRequest & request)
  {
    if (auto response = catalog_->route(request.method, request.path,
        query_value(request.query, "path"), request.body))
    {
      return HttpResponse::json(std::move(response->body), response->status);
    }
    return problem(404, "unknown endpoint");
  }

  HttpResponse logs(const HttpRequest & request)
  {
    try {
      if (request.path == "/api/logs") {
        std::map<std::string, std::int64_t> cursors;
        for (const auto target : managed_targets) {
          cursors.emplace(target, integer_query(request.query, target, 0));
        }
        return HttpResponse::json(processes_->terminal_logs(cursors));
      }
      const auto & target = query_value(request.query, "target");
      const auto & session = query_value(request.query, "session");
      if (request.path == "/api/logs/sessions") {
        return HttpResponse::json(processes_->saved_logs(target));
      }
      if (request.path == "/api/logs/tail") {
        auto [body, truncated] = processes_->saved_log_tail(
          target, session, integer_query(request.query, "max_bytes", 262144));
        auto response = HttpResponse::bytes(
          std::make_shared<const std::string>(std::move(body)), "text/plain; charset=utf-8");
        response.headers.emplace("X-Log-Truncated", truncated ? "1" : "0");
        return response;
      }
      if (request.path == "/api/logs/download") {
        auto file = processes_->open_saved_log(target, session);
        auto response = HttpResponse::file(file.fd, "text/plain; charset=utf-8");
        response.headers.emplace("Content-Disposition", "attachment; filename=\"" + file.name + "\"");
        return response;
      }
    } catch (const ManagedLogMissing & error) {
      return problem(404, error.what());
    } catch (const ManagedLogError & error) {
      return problem(503, error.what());
    }
    return problem(404, "unknown endpoint");
  }

  Scene scene_;
  std::atomic_bool stopping_{false};
  TfTree tf_;
  RobotControl robot_;
  SystemStatus system_;
  fs::path static_dir_;
  std::mutex lifecycle_mutex_;
  std::mutex perception_mutex_;
  json perception_{{"available", false}, {"updated", 0.0}};
  bool perception_pending_ = false;
  rclcpp::Client<SaveMap>::SharedPtr save_map_;
  rclcpp::Client<GetParameters>::SharedPtr perception_client_;
  rclcpp::TimerBase::SharedPtr perception_timer_;
  std::unique_ptr<Catalog> catalog_;
  std::unique_ptr<Diagnostics> diagnostics_;
  std::unique_ptr<ManagedProcesses> processes_;
  std::unique_ptr<HttpServer> http_;
};
}  // namespace
}  // namespace web_control

int main(int argc, char ** argv) try
{
  sigset_t signals;
  sigemptyset(&signals);
  sigaddset(&signals, SIGINT);
  sigaddset(&signals, SIGTERM);
  if (pthread_sigmask(SIG_BLOCK, &signals, nullptr) != 0) {
    throw std::runtime_error("could not configure shutdown signal handler");
  }

  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto node = std::make_shared<web_control::WebControlNative>();
  rclcpp::executors::MultiThreadedExecutor state_executor(rclcpp::ExecutorOptions(), 2);
  rclcpp::executors::SingleThreadedExecutor cloud_executor;
  state_executor.add_node(node);
  cloud_executor.add_callback_group(node->cloud_group(), node->get_node_base_interface());
  std::atomic_bool executor_failed{false};
  const auto spin = [&](rclcpp::Executor & executor) {
      try {
        executor.spin();
      } catch (const std::exception & error) {
        RCLCPP_ERROR(node->get_logger(), "executor failed: %s", error.what());
        executor_failed = true;
      } catch (...) {
        RCLCPP_ERROR(node->get_logger(), "executor failed");
        executor_failed = true;
      }
    };
  std::thread state_thread;
  std::thread cloud_thread;
  try {
    state_thread = std::thread([&]() {spin(state_executor);});
    cloud_thread = std::thread([&]() {spin(cloud_executor);});
    timespec wait{0, 100000000};
    while (!executor_failed && rclcpp::ok()) {
      if (sigtimedwait(&signals, nullptr, &wait) >= 0) {
        break;
      }
      if (errno != EAGAIN && errno != EINTR) {
        throw std::runtime_error("shutdown signal wait failed");
      }
    }
  } catch (const std::exception & error) {
    executor_failed = true;
    RCLCPP_ERROR(node->get_logger(), "%s", error.what());
  }
  try {
    node->stop();
  } catch (const std::exception & error) {
    executor_failed = true;
    RCLCPP_ERROR(node->get_logger(), "shutdown failed: %s", error.what());
  }
  try {
    state_executor.cancel();
    cloud_executor.cancel();
  } catch (const std::exception & error) {
    executor_failed = true;
    RCLCPP_ERROR(node->get_logger(), "executor cancellation failed: %s", error.what());
    rclcpp::shutdown();
  }
  if (state_thread.joinable()) {
    state_thread.join();
  }
  if (cloud_thread.joinable()) {
    cloud_thread.join();
  }
  node->finish_diagnostics();
  rclcpp::shutdown();
  return executor_failed ? 1 : 0;
}
catch (const std::exception & error)
{
  RCLCPP_ERROR(rclcpp::get_logger("web_control"), "%s", error.what());
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 1;
}
