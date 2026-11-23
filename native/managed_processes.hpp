#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace web_control
{
inline constexpr std::array<const char *, 6> managed_targets{
  "chassis", "imu", "lidar", "localization", "navigation", "mapping"};

struct ManagedLaunch
{
  std::string package;
  std::string file;
  std::string args;
};

struct ManagedProcessConfig
{
  std::map<std::string, ManagedLaunch> launches{
    {"chassis", {"hunter_base", "hunter_base.launch.py", ""}},
    {"imu", {"fdilink_ahrs", "ahrs_driver.launch.py", ""}},
    {"lidar", {"hesai_ros_driver", "start.py", ""}},
    {"localization", {"lio_sam_hesai", "localization_with_nonground.launch.py", "use_rviz:=false"}},
    {"navigation", {"p2p_move_base", "p2p_move_base_navigation.launch.py", ""}},
    {"mapping", {"lio_sam_hesai", "mapping.launch.py", "use_rviz:=false"}},
  };
  std::filesystem::path log_directory;
  std::size_t max_lines = 400;
};

using ManagedLaunchArguments = std::vector<std::pair<std::string, std::string>>;

struct ManagedProcessHooks
{
  std::function<double()> now;
  std::function<ManagedLaunchArguments(const std::string &)> map_arguments;
  std::function<void(const std::string &)> on_start;
  std::function<void(const std::string &)> on_exit;
  // Supplies the active diagnostic session and persist_error, if owned by the node.
  std::function<nlohmann::json()> diagnostics_session;
};

class ManagedLogMissing : public std::runtime_error
{
  using std::runtime_error::runtime_error;
};
class ManagedLogError : public std::runtime_error
{
  using std::runtime_error::runtime_error;
};

struct ManagedLogDownload
{
  ManagedLogDownload(int descriptor, std::string filename, std::uint64_t bytes);
  ~ManagedLogDownload();
  ManagedLogDownload(ManagedLogDownload && other) noexcept;
  ManagedLogDownload(const ManagedLogDownload &) = delete;
  ManagedLogDownload & operator=(const ManagedLogDownload &) = delete;
  int fd;
  std::string name;
  std::uint64_t size;
};

std::filesystem::path managed_log_directory(const std::filesystem::path & configured = {});

class ManagedProcesses
{
public:
  explicit ManagedProcesses(ManagedProcessConfig config = {}, ManagedProcessHooks hooks = {});
  ~ManagedProcesses();
  ManagedProcesses(const ManagedProcesses &) = delete;
  ManagedProcesses & operator=(const ManagedProcesses &) = delete;

  nlohmann::json start(const std::string & target);
  nlohmann::json stop(const std::string & target);
  bool running(const std::string & target) const;
  void shutdown();

  nlohmann::json terminal_logs(const std::map<std::string, std::int64_t> & cursors = {}) const;
  nlohmann::json saved_logs(const std::string & target) const;
  std::pair<std::string, bool> saved_log_tail(
    const std::string & target, const std::string & session_id,
    std::int64_t max_bytes = 262144) const;
  ManagedLogDownload open_saved_log(const std::string & target, const std::string & session_id) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace web_control
