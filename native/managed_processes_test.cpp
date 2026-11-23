#include "managed_processes.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace
{
using namespace std::chrono_literals;
using web_control::ManagedProcessConfig;
using web_control::ManagedProcessHooks;
using web_control::ManagedProcesses;

void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

class TempDirectory
{
public:
  TempDirectory()
  {
    char name[] = "/tmp/web-control-log-test-XXXXXX";
    const char * result = ::mkdtemp(name);
    require(result != nullptr, "mkdtemp failed");
    path = result;
  }

  ~TempDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }

  std::filesystem::path path;
};
}  // namespace

int main(int argc, char ** argv)
{
  const auto binary = std::filesystem::path(argv[0]).filename();
  if (binary == "sudo") {
    if (argc == 4 && std::string(argv[1]) == "-n" && std::string(argv[2]) == "modprobe" &&
      std::string(argv[3]) == "can")
    {
      return 0;
    }
    if (argc == 11 && std::string(argv[2]) == "ip" && std::string(argv[5]) == "can0" &&
      std::string(argv[10]) == "500000")
    {
      ::dprintf(STDERR_FILENO, "RTNETLINK answers: Device or resource busy\n");
      return 2;
    }
    return 42;
  }
  if (binary == "ip") {
    if (argc != 6 || std::string(argv[1]) != "-j" || std::string(argv[5]) != "can0") {
      return 42;
    }
    ::dprintf(STDOUT_FILENO,
      "[{\"ifname\":\"can0\",\"link_type\":\"can\",\"flags\":[\"UP\"],"
      "\"linkinfo\":{\"info_kind\":\"can\",\"info_data\":{\"bittiming\":{\"bitrate\":%s}}}}]\n",
      std::getenv("WEB_CONTROL_TEST_BITRATE"));
    return 0;
  }
  if (argc > 1 && std::string(argv[1]) == "launch") {
    if (argc == 4 && std::string(argv[2]) == "hunter_base") {
      ::dprintf(STDOUT_FILENO, "chassis fake launch\n");
      return 0;
    }
    if (argc != 6 || std::string(argv[4]) != "custom:=text with spaces" ||
      std::string(argv[5]) != "map_path:=/path with spaces;$(false)" ||
      std::string(std::getenv("PYTHONUNBUFFERED")) != "1" ||
      std::string(std::getenv("RCUTILS_COLORIZED_OUTPUT")) != "0")
    {
      return 42;
    }
    const auto descendant = ::fork();
    if (descendant == 0) {
      if (::setsid() < 0) {
        return 42;
      }
      ::dprintf(STDOUT_FILENO, "\033[32mready:%d\033[0m\n", static_cast<int>(::getpid()));
    } else if (descendant < 0) {
      return 42;
    }
    for (;;) {
      ::pause();
    }
  }

  TempDirectory directory;
  const auto executable = std::filesystem::canonical("/proc/self/exe");
  for (const auto * name : {"ros2", "sudo", "ip"}) {
    std::filesystem::create_symlink(executable, directory.path / name);
  }
  require(::prctl(PR_SET_CHILD_SUBREAPER, 1) == 0, "cannot collect fake descendant");
  ::setenv("PATH", directory.path.c_str(), 1);
  ManagedProcessConfig config;
  config.log_directory = directory.path / "logs";
  config.max_lines = 4;
  config.launches["navigation"] = {
    "test_package", "test_launch", "map_path:=old custom:='text with spaces'"};
  ManagedProcessHooks hooks;
  hooks.map_arguments = [](const std::string &) {
      return web_control::ManagedLaunchArguments{{"map_path", "/path with spaces;$(false)"}};
    };
  ManagedProcesses managed(config, hooks);

  const auto started = managed.start("navigation");
  require(started.at("ok").get<bool>(), "start failed");
  require(started.at("log_error").is_null(), "persistent log start failed");
  pid_t descendant = 0;
  for (int attempt = 0; attempt < 100 && descendant == 0; ++attempt) {
    const auto lines = managed.terminal_logs()["logs"]["navigation"]["lines"];
    for (const auto & line : lines) {
      const auto text = line.get<std::string>();
      if (text.compare(0, 6, "ready:") == 0 && text.find('\x1b') == std::string::npos) {
        descendant = std::stoi(text.substr(6));
      }
    }
    std::this_thread::sleep_for(10ms);
  }
  require(descendant > 0, "sanitized child output missing");
  const auto listed = managed.saved_logs("navigation").at("sessions");
  require(listed.size() == 1 && listed.at(0).at("active") == true, "active log missing");
  require(managed.start("navigation").at("ok") == true, "repeated start failed");
  require(managed.saved_logs("navigation").at("sessions").size() == 1, "duplicate session");
  require(managed.stop("navigation").at("ok") == true, "stop failed");
  require(!managed.running("navigation"), "stopped child still running");
  int status = 0;
  require(::waitpid(descendant, &status, WNOHANG) == descendant && WIFSIGNALED(status),
    "escaped child was not stopped");
  const auto log = managed.terminal_logs()["logs"]["navigation"];
  require(log["returncode"] == -SIGINT, "stop did not retain process exit code");
  const auto cursors = std::map<std::string, std::int64_t>{
    {"navigation", log["seq"].get<std::int64_t>()}};
  require(managed.terminal_logs(cursors)["logs"]["navigation"]["lines"].empty(), "delta not empty");

  const std::string session = listed.at(0).at("session");
  const auto tail = managed.saved_log_tail("navigation", session);
  require(tail.first.find("ready:") != std::string::npos, "persistent output missing");
  require(!tail.second, "short log marked truncated");
  const auto download = managed.open_saved_log("navigation", session);
  require(download.size > 0 && download.name == session + ".log", "download metadata wrong");

  const auto file = config.log_directory / "navigation" / (session + ".log");
  std::filesystem::remove(file);
  std::filesystem::create_symlink(executable, file);
  bool rejected = false;
  try {
    managed.open_saved_log("navigation", session);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  require(rejected, "saved-log file symlink was followed");

  ::setenv("WEB_CONTROL_TEST_BITRATE", "250000", 1);
  require(managed.start("chassis")["ok"] == true, "chassis preflight did not start");
  for (int attempt = 0; attempt < 100 && managed.running("chassis"); ++attempt) {
    std::this_thread::sleep_for(10ms);
  }
  auto chassis = managed.terminal_logs()["logs"]["chassis"];
  require(chassis["returncode"] == 2, "a mismatched CAN bitrate must reject the chassis launch");
  auto cursor = chassis["seq"].get<std::int64_t>();
  ::setenv("WEB_CONTROL_TEST_BITRATE", "500000", 1);
  require(managed.start("chassis")["ok"] == true, "chassis retry did not start");
  for (int attempt = 0; attempt < 100 && managed.running("chassis"); ++attempt) {
    std::this_thread::sleep_for(10ms);
  }
  chassis = managed.terminal_logs({{"chassis", cursor}})["logs"]["chassis"];
  require(chassis["returncode"] == 0, "verified CAN bitrate must permit the chassis launch");
  require(std::find(chassis["lines"].begin(), chassis["lines"].end(), "chassis fake launch") !=
    chassis["lines"].end(), "fake chassis launch output missing");
}
