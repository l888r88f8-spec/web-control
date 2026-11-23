#include "managed_processes.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <optional>
#include <poll.h>
#include <random>
#include <regex>
#include <spawn.h>
#include <sstream>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

extern char ** environ;

namespace web_control
{
namespace
{
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

constexpr std::size_t max_line_bytes = 262144;
constexpr auto can_verify_timeout = 2s;
constexpr std::array stop_signals{SIGINT, SIGTERM, SIGKILL};
constexpr std::array stop_timeouts{1500ms, 750ms, 250ms};
const std::vector<std::string> can_module{"sudo", "-n", "modprobe", "can"};
const std::vector<std::string> can_up{
  "sudo", "-n", "ip", "link", "set", "can0", "up", "type", "can", "bitrate", "500000"};
const std::vector<std::string> can_verify{"ip", "-j", "-details", "link", "show", "can0"};

struct Descriptor
{
  explicit Descriptor(int descriptor = -1) : fd(descriptor) {}
  ~Descriptor() {if (fd >= 0) {::close(fd);}}
  Descriptor(Descriptor && other) noexcept : fd(std::exchange(other.fd, -1)) {}
  Descriptor & operator=(Descriptor && other) noexcept
  {
    std::swap(fd, other.fd);
    return *this;
  }
  int fd;
};

double wall_time()
{
  return std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string local_time(double timestamp, const char * format)
{
  const auto seconds = static_cast<std::time_t>(timestamp);
  std::tm local{};
  ::localtime_r(&seconds, &local);
  std::array<char, 32> buffer{};
  std::strftime(buffer.data(), buffer.size(), format, &local);
  return buffer.data();
}

std::string session_name(double timestamp)
{
  thread_local std::mt19937_64 random(std::random_device{}());
  std::ostringstream id;
  id << local_time(timestamp, "%Y%m%d-%H%M%S") << "-" <<
    std::hex << std::setw(12) << std::setfill('0') << (random() & 0xffffffffffffULL);
  return id.str();
}

bool valid_session(const std::string & id)
{
  static const std::regex pattern("[0-9]{8}-[0-9]{6}-[0-9a-f]{8,32}");
  return id.size() >= 24 && id.size() <= 48 && std::regex_match(id, pattern);
}

std::string clean_text(const std::string & input)
{
  std::string text;
  text.reserve(input.size());
  for (std::size_t i = 0; i < input.size();) {
    if (input[i] == '\x1b' && i + 1 < input.size()) {
      const char next = input[i + 1];
      if ((next >= '@' && next <= 'Z') || (next >= '\\' && next <= '_')) {
        i += 2;
        continue;
      }
      if (next == '[') {
        auto end = i + 2;
        while (end < input.size() && input[end] >= '0' && input[end] <= '?') {
          ++end;
        }
        while (end < input.size() && input[end] >= ' ' && input[end] <= '/') {
          ++end;
        }
        if (end < input.size() && input[end] >= '@' && input[end] <= '~') {
          i = end + 1;
          continue;
        }
      }
    }
    text += input[i++];
  }
  // JSON's UTF-8 replacement matches the text API when output contains an incomplete code point.
  try {
    Json(text).dump();
    return text;
  } catch (const Json::type_error &) {
    return Json::parse(Json(text).dump(-1, ' ', false, Json::error_handler_t::replace)).get<std::string>();
  }
}

std::string join_command(const std::vector<std::string> & command)
{
  std::string result;
  for (const auto & argument : command) {
    result += (result.empty() ? "" : " ") + argument;
  }
  return result;
}

std::vector<std::string> split_command_arguments(const std::string & value)
{
  std::vector<std::string> arguments;
  std::string argument;
  char quote = 0;
  bool started = false;
  for (std::size_t i = 0; i < value.size(); ++i) {
    const char character = value[i];
    if (quote && character == quote) {
      quote = 0;
    } else if (!quote && (character == '"' || character == '\'')) {
      quote = character;
    } else if (!quote && std::string(" \t\n\r").find(character) != std::string::npos) {
      if (started) {
        arguments.push_back(std::move(argument));
        argument.clear();
        started = false;
      }
      continue;
    } else if (character == '\\' && quote != '\'') {
      if (i + 1 == value.size()) {
        throw std::invalid_argument("No escaped character in managed launch arguments");
      }
      const char next = value[i + 1];
      if (!quote || next == '"' || next == '\\') {
        argument += next;
        ++i;
      } else {
        argument += character;
      }
    } else {
      argument += character;
    }
    started = true;
  }
  if (quote) {
    throw std::invalid_argument("No closing quotation in managed launch arguments");
  }
  if (started) {
    arguments.push_back(std::move(argument));
  }
  return arguments;
}

std::vector<std::string> launch_command(
  const ManagedLaunch & launch, const ManagedLaunchArguments & overrides)
{
  std::vector<std::string> command{"ros2", "launch", launch.package, launch.file};
  for (auto argument : split_command_arguments(launch.args)) {
    const auto split = argument.find(":=");
    const auto overridden = split != std::string::npos && std::any_of(
      overrides.begin(), overrides.end(), [&argument, split](const auto & entry) {
        return argument.substr(0, split) == entry.first;
      });
    if (!overridden) {
      command.push_back(std::move(argument));
    }
  }
  for (const auto & [name, value] : overrides) {
    command.push_back(name + ":=" + value);
  }
  for (const auto & argument : command) {
    if (argument.find('\0') != std::string::npos) {
      throw std::invalid_argument("managed launch arguments cannot contain null bytes");
    }
  }
  return command;
}

// Open target directories and files relative to descriptors so replacing a symlink
// during a download cannot redirect it outside the configured log directory.
Descriptor open_target_directory(const fs::path & root, const std::string & target)
{
  Descriptor parent(::open(root.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC));
  if (parent.fd < 0) {
    if (errno == ENOENT || errno == ENOTDIR) {
      return Descriptor();
    }
    throw ManagedLogError("cannot open log directory: " + std::string(std::strerror(errno)));
  }
  Descriptor directory(::openat(
    parent.fd, target.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW));
  if (directory.fd >= 0 || errno == ENOENT) {
    return directory;
  }
  if (errno == ENOTDIR || errno == ELOOP) {
    throw std::invalid_argument("refusing to read a symlinked or invalid log target");
  }
  throw ManagedLogError("cannot open log target: " + std::string(std::strerror(errno)));
}

struct Session
{
  std::string id;
  fs::path path;
  double started_at;
  Descriptor file;
  std::uint64_t bytes = 0;
  std::string error;

  void write(const std::string & line)
  {
    if (file.fd < 0) {
      return;
    }
    const auto text = line + '\n';
    std::size_t offset = 0;
    while (offset < text.size()) {
      const auto count = ::write(file.fd, text.data() + offset, text.size() - offset);
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count <= 0) {
        error = "log write failed: " + std::string(std::strerror(errno));
        file = Descriptor();
        return;
      }
      offset += count;
      bytes += count;
    }
  }
};

struct Target
{
  std::mutex operation;
  mutable std::mutex mutex;
  std::thread worker;
  bool enabled = false;
  bool running = false;
  bool cancelled = false;
  std::optional<int> returncode;
  std::optional<Session> session;
  std::string persist_error;
  std::int64_t sequence = 0;
  std::deque<std::pair<std::int64_t, std::string>> lines;
};

struct Spawn
{
  pid_t pid = -1;
  Descriptor output;
  int error = 0;
};

Spawn spawn(const std::vector<std::string> & command)
{
  int output[2];
  if (::pipe2(output, O_CLOEXEC) < 0) {
    return {-1, Descriptor(), errno};
  }
  Descriptor reader(output[0]), writer(output[1]);
  if (::fcntl(reader.fd, F_SETFL, O_NONBLOCK) < 0) {
    return {-1, Descriptor(), errno};
  }

  std::vector<char *> argv;
  for (const auto & argument : command) {
    argv.push_back(const_cast<char *>(argument.c_str()));
  }
  argv.push_back(nullptr);

  char unbuffered[] = "PYTHONUNBUFFERED=1";
  char color[] = "RCUTILS_COLORIZED_OUTPUT=0";
  std::vector<char *> env;
  for (char ** item = environ; *item != nullptr; ++item) {
    if (std::strncmp(*item, "PYTHONUNBUFFERED=", 17) != 0 &&
      std::strncmp(*item, "RCUTILS_COLORIZED_OUTPUT=", 25) != 0)
    {
      env.push_back(*item);
    }
  }
  env.insert(env.end(), {unbuffered, color, nullptr});

  posix_spawn_file_actions_t actions;
  int error = ::posix_spawn_file_actions_init(&actions);
  if (error != 0) {
    return {-1, Descriptor(), error};
  }
  error = ::posix_spawn_file_actions_adddup2(&actions, writer.fd, STDOUT_FILENO);
  if (error == 0) {
    error = ::posix_spawn_file_actions_adddup2(&actions, writer.fd, STDERR_FILENO);
  }

  posix_spawnattr_t attr;
  const int attr_error = ::posix_spawnattr_init(&attr);
  if (error == 0) {
    error = attr_error;
  }
  if (error == 0) {
    sigset_t defaults, mask;
    ::sigemptyset(&defaults);
    for (const int signal : {SIGINT, SIGTERM, SIGHUP, SIGPIPE}) {
      ::sigaddset(&defaults, signal);
    }
    ::sigemptyset(&mask);
    error = ::posix_spawnattr_setsigdefault(&attr, &defaults);
    if (error == 0) {
      error = ::posix_spawnattr_setsigmask(&attr, &mask);
    }
    if (error == 0) {
      error = ::posix_spawnattr_setflags(
        &attr, POSIX_SPAWN_SETSID | POSIX_SPAWN_SETSIGDEF | POSIX_SPAWN_SETSIGMASK);
    }
  }
  pid_t pid = -1;
  if (error == 0) {
    error = ::posix_spawnp(&pid, argv.front(), &actions, &attr, argv.data(), env.data());
  }
  if (attr_error == 0) {
    ::posix_spawnattr_destroy(&attr);
  }
  ::posix_spawn_file_actions_destroy(&actions);
  return {pid, std::move(reader), error};
}

struct ProcessIdentity
{
  pid_t parent;
  pid_t group;
  unsigned long long start;
  char state;
};

std::optional<ProcessIdentity> process_identity(pid_t pid)
{
  std::ifstream stat("/proc/" + std::to_string(pid) + "/stat");
  std::string line;
  std::getline(stat, line);
  const auto end = line.rfind(')');
  if (end == std::string::npos || end + 2 >= line.size()) {
    return std::nullopt;
  }
  std::istringstream fields(line.substr(end + 2));
  ProcessIdentity identity{};
  if (!(fields >> identity.state >> identity.parent >> identity.group)) {
    return std::nullopt;
  }
  std::string ignored;
  for (int field = 6; field < 22; ++field) {
    fields >> ignored;
  }
  if (!(fields >> identity.start)) {
    return std::nullopt;
  }
  return identity;
}

struct Descendant
{
  unsigned long long start;
  Descriptor pidfd;
};

using Descendants = std::map<pid_t, Descendant>;

void observe_descendants(pid_t leader, Descendants & tracked)
{
  std::multimap<pid_t, std::pair<pid_t, unsigned long long>> children;
  std::error_code error;
  for (fs::directory_iterator it("/proc", error), end; !error && it != end; it.increment(error)) {
    const auto name = it->path().filename().string();
    if (name.empty() || name.find_first_not_of("0123456789") != std::string::npos) {
      continue;
    }
    const auto pid = static_cast<pid_t>(std::strtol(name.c_str(), nullptr, 10));
    const auto identity = process_identity(pid);
    if (identity && pid != leader) {
      const auto parent = identity->group == leader ? leader : identity->parent;
      children.emplace(parent, std::make_pair(pid, identity->start));
    }
  }
  std::vector<pid_t> parents{leader};
  for (auto entry = tracked.begin(); entry != tracked.end();) {
    const auto identity = process_identity(entry->first);
    if (identity && identity->start == entry->second.start) {
      parents.push_back(entry->first);
      ++entry;
    } else {
      entry = tracked.erase(entry);
    }
  }
  while (!parents.empty()) {
    const auto parent = parents.back();
    parents.pop_back();
    const auto [first, last] = children.equal_range(parent);
    for (auto child = first; child != last; ++child) {
      const auto [pid, start] = child->second;
      if (tracked.count(pid)) {
        continue;
      }
      Descriptor pidfd;
#ifdef SYS_pidfd_open
      pidfd = Descriptor(static_cast<int>(::syscall(SYS_pidfd_open, pid, 0)));
#endif
      const auto identity = process_identity(pid);
      if (identity && identity->start == start) {
        tracked.emplace(pid, Descendant{start, std::move(pidfd)});
        parents.push_back(pid);
      }
    }
  }
}

bool descendants_running(const Descendants & tracked)
{
  return std::any_of(tracked.begin(), tracked.end(), [](const auto & item) {
    const auto identity = process_identity(item.first);
    return identity && identity->start == item.second.start &&
           identity->state != 'Z' && identity->state != 'X';
  });
}

void signal_group(pid_t leader, const Descendants & tracked, int signal)
{
  ::kill(-leader, signal);
  for (const auto & [pid, known] : tracked) {
    const auto identity = process_identity(pid);
    if (!identity || identity->start != known.start || identity->group == leader) {
      continue;
    }
#ifdef SYS_pidfd_send_signal
    if (known.pidfd.fd >= 0) {
      ::syscall(SYS_pidfd_send_signal, known.pidfd.fd, signal, nullptr, 0);
      continue;
    }
#endif
    ::kill(pid, signal);
  }
}

bool verified_can(const std::string & output)
{
  const auto value = Json::parse(output, nullptr, false);
  if (!value.is_array() || value.size() != 1 || !value.front().is_object()) {
    return false;
  }
  try {
    const auto & link = value.front();
    const auto & flags = link.at("flags");
    const auto & info = link.at("linkinfo");
    if (link.at("ifname") != "can0" || !flags.is_array() ||
      std::find(flags.begin(), flags.end(), Json("UP")) == flags.end() ||
      link.at("link_type") != "can" || info.at("info_kind") != "can")
    {
      return false;
    }
    const auto & data = info.at("info_data");
    auto bitrate = data.value("bitrate", Json());
    if (bitrate.is_null()) {
      bitrate = data.at("bittiming").at("bitrate");
    }
    if (bitrate.is_string()) {
      const auto str = bitrate.get<std::string>();
      std::size_t end = 0;
      return std::stod(str, &end) == 500000 && end == str.size();
    }
    return bitrate.is_number() && bitrate == 500000;
  } catch (const std::exception &) {
    return false;
  }
}
}  // namespace

struct ManagedProcesses::Impl
{
  ManagedProcessConfig config;
  ManagedProcessHooks hooks;
  std::map<std::string, Target> targets;
  std::atomic<bool> shutting_down{false};

  Impl(ManagedProcessConfig value, ManagedProcessHooks callbacks)
  : config(std::move(value)), hooks(std::move(callbacks))
  {
    config.log_directory = managed_log_directory(config.log_directory);
    std::error_code error;
    fs::create_directories(config.log_directory, error);
    for (const auto * name : managed_targets) {
      launch_command(config.launches.at(name), {});
      targets.try_emplace(name);
    }
  }

  double now() const {return hooks.now ? hooks.now() : wall_time();}

  Json response(const std::string & message) const
  {
    return {{"ok", true}, {"message", message}, {"stamp", now()}};
  }

  Target & get(const std::string & name)
  {
    return const_cast<Target &>(std::as_const(*this).get(name));
  }

  const Target & get(const std::string & name) const
  {
    const auto target = targets.find(name);
    if (target == targets.end()) {
      throw std::invalid_argument("unknown managed target: " + name);
    }
    return target->second;
  }

  static Json possible_error(const std::string & message)
  {
    return message.empty() ? Json() : Json(message);
  }

  void append_line(Target & target, std::string line)
  {
    if (target.session) {
      target.session->write(line);
      target.persist_error = target.session->error;
    }
    ++target.sequence;
    if (config.max_lines > 0) {
      if (target.lines.size() == config.max_lines) {
        target.lines.pop_front();
      }
      target.lines.emplace_back(target.sequence, std::move(line));
    }
  }

  void append(Target & target, const std::string & line)
  {
    auto text = clean_text(line);
    const std::lock_guard lock(target.mutex);
    append_line(target, std::move(text));
  }

  void begin_session(const std::string & name, Target & target, const std::vector<std::string> & command)
  {
    target.persist_error.clear();
    try {
      fs::create_directories(config.log_directory / name);
      auto directory = open_target_directory(config.log_directory, name);
      if (directory.fd < 0) {
        throw ManagedLogError("log target directory is unavailable");
      }
      const double started_at = wall_time();
      auto id = session_name(started_at);
      const auto filename = id + ".log";
      Descriptor file(::openat(
        directory.fd, filename.c_str(),
        O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0644));
      if (file.fd < 0) {
        throw ManagedLogError("cannot create log file for " + name + ": " + std::strerror(errno));
      }
      target.session.emplace(Session{
        id, config.log_directory / name / filename, started_at, std::move(file), 0, {}});
      target.session->write(
        "# managed log target=" + name + " session=" + id + " started=" +
        local_time(started_at, "%Y-%m-%dT%H:%M:%S"));
      target.session->write("$ " + clean_text(join_command(command)));
      target.persist_error = target.session->error;
    } catch (const std::exception & error) {
      target.persist_error = "persistent log unavailable: " + std::string(error.what());
    }
  }

  Json session_info(const Target & target) const
  {
    Json info{{"persist_error", possible_error(target.persist_error)}};
    if (target.session) {
      const auto & session = *target.session;
      info.update({
        {"session", session.id}, {"started_at", session.started_at}, {"bytes", session.bytes},
        {"closed", session.file.fd < 0}, {"error", possible_error(session.error)},
        {"path", session.path.string()},
      });
    }
    return info;
  }

  void notify_exit(const std::string & name)
  {
    if (hooks.on_exit) {
      try {
        hooks.on_exit(name);
      } catch (...) {
        // A node callback must not terminate the supervisor or its worker.
      }
    }
  }

  void finish(
    const std::string & name, Target & target, std::optional<int> code,
    const std::string & message, bool notify = true)
  {
    {
      const std::lock_guard lock(target.mutex);
      append_line(target, "[web_control] " + name + " " + message);
      target.session.reset();
      target.running = false;
      target.returncode = code;
    }
    if (notify) {
      notify_exit(name);
    }
  }

  struct ChildResult
  {
    int code = 1;
    bool cancelled = false;
    bool timed_out = false;
    bool spawn_failed = false;
    bool launched = false;
    std::string output;
  };

  ChildResult run_child(
    Target & target, const std::vector<std::string> & command, bool capture = false,
    bool log = true, std::optional<Clock::duration> timeout = std::nullopt)
  {
    Spawn child;
    ChildResult result;
    {
      // Coordinate registration with stop so a cancelled sequence cannot spawn
      // its next prerequisite or the final ROS launch.
      const std::lock_guard lock(target.mutex);
      if (target.cancelled) {
        result.cancelled = true;
        return result;
      }
      if (log) {
        append_line(target, "$ " + clean_text(join_command(command)));
      }
      child = spawn(command);
    }
    if (child.error != 0) {
      result.code = child.error == ENOENT ? 127 : 1;
      result.spawn_failed = true;
      if (log) {
        append(target, "failed to start: " + std::string(std::strerror(child.error)));
      }
      return result;
    }
    result.launched = true;

    std::string pending;
    bool previous_cr = false;
    Descendants descendants;
    std::size_t signal_step = 0;
    auto deadline = Clock::now();
    auto next_proc_scan = deadline;
    const auto child_deadline = timeout ? deadline + *timeout : Clock::time_point::max();
    bool leader_exited = false;
    auto drain = [&]() {
        if (child.output.fd < 0) {
          return;
        }
        std::array<char, 8192> bytes;
        // Recheck stop and exit between batches even when the pipe never empties.
        for (int batch = 0; batch < 8; ++batch) {
          const auto count = ::read(child.output.fd, bytes.data(), bytes.size());
          if (count < 0 && errno == EINTR) {
            continue;
          }
          if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            break;
          }
          if (count <= 0) {
            child.output = Descriptor();
            break;
          }
          if (capture && result.output.size() < max_line_bytes) {
            result.output.append(
              bytes.data(), std::min<std::size_t>(count, max_line_bytes - result.output.size()));
          }
          if (!log) {
            continue;
          }
          for (ssize_t i = 0; i < count; ++i) {
            const auto character = bytes[i];
            if (character == '\n' && previous_cr) {
              previous_cr = false;
              continue;
            }
            const bool newline = character == '\r' || character == '\n';
            if (newline || pending.size() == max_line_bytes) {
              append(target, pending);
              pending.clear();
            }
            if (!newline) {
              pending += character;
            }
            previous_cr = character == '\r';
          }
        }
      };

    // Keep the leader as a waitable zombie until the group has been signalled,
    // so no newly allocated process can reuse its process-group identifier.
    while (true) {
      drain();
      const auto now = Clock::now();
      if (now >= next_proc_scan) {
        observe_descendants(child.pid, descendants);
        next_proc_scan = now + 500ms;
      }
      siginfo_t info{};
      if (::waitid(P_PID, child.pid, &info, WEXITED | WNOHANG | WNOWAIT) == 0) {
        leader_exited = info.si_pid == child.pid;
      } else if (errno != EINTR) {
        child.pid = -1;
        break;
      }
      {
        const std::lock_guard lock(target.mutex);
        result.cancelled = target.cancelled;
      }
      result.timed_out = result.timed_out || (!leader_exited && now >= child_deadline);

      if ((result.cancelled || result.timed_out || leader_exited) && signal_step == 0) {
        observe_descendants(child.pid, descendants);
        if (result.timed_out) {
          signal_step = stop_signals.size() - 1;
        }
        signal_group(child.pid, descendants, stop_signals[signal_step]);
        deadline = now + stop_timeouts[signal_step++];
      }
      if (leader_exited && child.output.fd < 0 && !descendants_running(descendants)) {
        // Catch same-group children that started and detached their output before a scan.
        signal_group(child.pid, descendants, SIGKILL);
        break;
      }
      if (signal_step > 0 && now >= deadline) {
        if (signal_step == stop_signals.size()) {
          break;
        }
        observe_descendants(child.pid, descendants);
        signal_group(child.pid, descendants, stop_signals[signal_step]);
        deadline = now + stop_timeouts[signal_step++];
      }
      pollfd output{child.output.fd, POLLIN, 0};
      ::poll(&output, 1, 25);
    }

    drain();
    if (!pending.empty()) {
      append(target, pending);
    }
    if (child.pid > 0) {
      int status = 0;
      pid_t reaped;
      do {
        reaped = ::waitpid(child.pid, &status, WNOHANG);
      } while (reaped < 0 && errno == EINTR);
      if (reaped > 0) {
        result.code = WIFEXITED(status) ? WEXITSTATUS(status) : -WTERMSIG(status);
      } else if (reaped == 0) {
        // A child in uninterruptible kernel sleep may outlast the stop budget.
        // Arrange eventual reaping without holding the HTTP request or node.
        std::thread([pid = child.pid]() {
          while (::waitpid(pid, nullptr, 0) < 0 && errno == EINTR) {}
        }).detach();
        result.code = -SIGKILL;
      }
    }
    return result;
  }

  void run_target(std::string name, Target & target, const std::vector<std::string> & command)
  {
    try {
      if (name == "chassis") {
        for (const auto & prerequisite : {can_module, can_up}) {
          const auto step = run_child(target, prerequisite, true);
          if (step.cancelled) {
            finish(name, target, std::nullopt, "stopped during preflight");
            return;
          }
          if (step.code == 0) {
            continue;
          }

          if (prerequisite == can_up && step.output.find("Device or resource busy") != std::string::npos) {
            const auto verification = run_child(target, can_verify, true, false, can_verify_timeout);
            if (verification.cancelled) {
              finish(name, target, std::nullopt, "stopped during preflight");
              return;
            }
            if (!verification.timed_out && verification.code == 0 && verified_can(verification.output)) {
              append(target, join_command(prerequisite) +
                ": can0 already up with verified UP/CAN/500000 bitrate; continuing");
              continue;
            }
            const std::string failure = verification.timed_out ? "read-only ip query timed out after 2s" :
              (verification.code == 0 ? "expected can0 UP/CAN/500000 bitrate" :
              "read-only ip query exited " + std::to_string(verification.code));
            append(target, join_command(prerequisite) + ": EBUSY is not enough (" + failure +
              "); not starting " + name);
          }
          if (!step.spawn_failed) {
            append(target, join_command(prerequisite) + " failed (exit " + std::to_string(step.code) +
              "); not starting " + name);
          }
          finish(name, target, step.code, "prerequisite sequence ended (exit " + std::to_string(step.code) + ")");
          return;
        }
      }

      const auto result = run_child(target, command);
      if (result.spawn_failed) {
        finish(name, target, result.code, "failed to start (exit " + std::to_string(result.code) + ")", false);
      } else if (!result.launched && result.cancelled) {
        finish(name, target, std::nullopt,
          name == "chassis" ? "stopped during preflight" : "stopped (exit None)");
      } else {
        finish(name, target, result.code, "exited with code " + std::to_string(result.code));
      }
    } catch (const std::exception & error) {
      append(target, "process supervisor failed: " + std::string(error.what()));
      finish(name, target, 1, "exited with code 1");
    }
  }

  void request_stop(const std::string & name, Target & target)
  {
    const std::lock_guard lock(target.mutex);
    if (target.running && !target.cancelled) {
      target.cancelled = true;
      append_line(target, "stopping " + name + "...");
    }
  }

  void stop_all()
  {
    std::vector<std::unique_lock<std::mutex>> locks;
    locks.reserve(targets.size());
    for (auto & [name, target] : targets) {
      locks.emplace_back(target.operation);
      request_stop(name, target);
    }
    for (auto & [name, target] : targets) {
      if (target.worker.joinable()) {
        target.worker.join();
      }
    }
  }

  const std::string & require_log_target(const std::string & name) const
  {
    if (name != "diagnostics") {
      get(name);
    }
    return name;
  }
};

ManagedLogDownload::ManagedLogDownload(int descriptor, std::string filename, std::uint64_t bytes)
: fd(descriptor), name(std::move(filename)), size(bytes) {}

ManagedLogDownload::~ManagedLogDownload()
{
  if (fd >= 0) {
    ::close(fd);
  }
}

ManagedLogDownload::ManagedLogDownload(ManagedLogDownload && other) noexcept
: fd(std::exchange(other.fd, -1)), name(std::move(other.name)), size(other.size) {}

fs::path managed_log_directory(const fs::path & configured)
{
  const auto value = configured.string();
  const auto first = value.find_first_not_of(" \t\r\n");
  const auto last = value.find_last_not_of(" \t\r\n");
  const auto text = first == std::string::npos ? std::string() : value.substr(first, last - first + 1);
  const auto * home_env = ::getenv("HOME");
  const fs::path home = home_env ? home_env : "";
  if (text.empty()) {
    const auto * xdg_env = ::getenv("XDG_STATE_HOME");
    const fs::path xdg = xdg_env ? xdg_env : "";
    const fs::path state = xdg.is_absolute() ? xdg : home / ".local" / "state";
    return state / "web_control" / "logs";
  }
  if (text == "~") {
    return home;
  }
  return text.compare(0, 2, "~/") == 0 ? home / text.substr(2) : fs::path(text);
}

ManagedProcesses::ManagedProcesses(ManagedProcessConfig config, ManagedProcessHooks hooks)
: impl_(std::make_unique<Impl>(std::move(config), std::move(hooks))) {}

ManagedProcesses::~ManagedProcesses()
{
  shutdown();
}

Json ManagedProcesses::start(const std::string & name)
{
  if (name == "all") {
    for (const auto * target : managed_targets) {
      start(target);
    }
    return impl_->response("managed processes starting");
  }

  auto & target = impl_->get(name);
  const std::lock_guard operation(target.operation);
  if (impl_->shutting_down) {
    throw std::runtime_error("managed processes are shutting down");
  }
  const auto arguments =
    impl_->hooks.map_arguments && (name == "localization" || name == "navigation") ?
    impl_->hooks.map_arguments(name) : ManagedLaunchArguments{};
  const auto command = launch_command(impl_->config.launches.at(name), arguments);
  Json persist_error;
  {
    const std::lock_guard lock(target.mutex);
    if (target.running) {
      auto response = impl_->response(name + " starting");
      response["log_error"] = Impl::possible_error(target.persist_error);
      return response;
    }
  }
  if (target.worker.joinable()) {
    target.worker.join();
  }
  if (impl_->hooks.on_start) {
    impl_->hooks.on_start(name);
  }
  {
    const std::lock_guard lock(target.mutex);
    target.enabled = target.running = true;
    target.cancelled = false;
    target.returncode.reset();
    impl_->begin_session(name, target, command);
    persist_error = Impl::possible_error(target.persist_error);
  }
  try {
    target.worker = std::thread([this, name, &target, command]() {
      impl_->run_target(name, target, command);
    });
  } catch (const std::system_error & error) {
    impl_->append(target, "failed to start: " + std::string(error.what()));
    impl_->finish(name, target, 1, "failed to start (exit 1)", false);
  }
  auto response = impl_->response(name + " starting");
  response["log_error"] = std::move(persist_error);
  return response;
}

Json ManagedProcesses::stop(const std::string & name)
{
  if (name == "all") {
    impl_->stop_all();
    return impl_->response("managed processes stopping");
  }
  auto & target = impl_->get(name);
  const std::lock_guard operation(target.operation);
  impl_->request_stop(name, target);
  if (target.worker.joinable()) {
    target.worker.join();
  }
  return impl_->response(name + " stopping");
}

bool ManagedProcesses::running(const std::string & name) const
{
  const auto & target = impl_->get(name);
  const std::lock_guard lock(target.mutex);
  return target.running;
}

void ManagedProcesses::shutdown()
{
  if (!impl_->shutting_down.exchange(true)) {
    impl_->stop_all();
  }
}

Json ManagedProcesses::terminal_logs(const std::map<std::string, std::int64_t> & cursors) const
{
  Json logs = Json::object();
  Json sessions = Json::object();
  for (const auto & [name, target] : impl_->targets) {
    const std::lock_guard lock(target.mutex);
    Json lines = Json::array();
    const auto cursor = cursors.find(name);
    const auto since = cursor != cursors.end() ? std::max<std::int64_t>(0, cursor->second) : 0;
    for (const auto & [sequence, line] : target.lines) {
      if (sequence > since) {
        lines.push_back(line);
      }
    }
    logs[name] = {
      {"target", name}, {"enabled", target.enabled}, {"running", target.running},
      {"returncode", target.returncode ? Json(*target.returncode) : Json()},
      {"lines", std::move(lines)}, {"seq", target.sequence},
    };
    sessions[name] = impl_->session_info(target);
  }
  return {
    {"logs", std::move(logs)}, {"sessions", std::move(sessions)},
    {"log_dir", impl_->config.log_directory.string()},
  };
}

Json ManagedProcesses::saved_logs(const std::string & name) const
{
  impl_->require_log_target(name);
  Json info = Json::object();
  if (name != "diagnostics") {
    const auto & target = impl_->get(name);
    const std::lock_guard lock(target.mutex);
    info = impl_->session_info(target);
  } else if (impl_->hooks.diagnostics_session) {
    info = impl_->hooks.diagnostics_session();
  }
  if (!info.is_object()) {
    info = Json::object();
  }
  const auto session_field = info.find("session");
  const auto active = session_field != info.end() && session_field->is_string() ?
    session_field->get<std::string>() : std::string();
  auto directory = open_target_directory(impl_->config.log_directory, name);
  std::vector<Json> rows;
  if (directory.fd >= 0) {
    const auto close_directory = [](DIR * value) {::closedir(value);};
    std::unique_ptr<DIR, decltype(close_directory)> stream(
      ::fdopendir(directory.fd), close_directory);
    if (stream) {
      directory.fd = -1;
      while (const auto * entry = ::readdir(stream.get())) {
        const std::string file = entry->d_name;
        if (file.size() < 5 || file.compare(file.size() - 4, 4, ".log") != 0) {
          continue;
        }
        const auto session = file.substr(0, file.size() - 4);
        struct stat stat {};
        if (!valid_session(session) ||
          ::fstatat(::dirfd(stream.get()), file.c_str(), &stat, AT_SYMLINK_NOFOLLOW) != 0 ||
          !S_ISREG(stat.st_mode))
        {
          continue;
        }
        const double mtime = stat.st_mtim.tv_sec + stat.st_mtim.tv_nsec / 1e9;
        rows.push_back({
          {"session", session}, {"started_at", mtime}, {"size", stat.st_size},
          {"modified", mtime}, {"active", session == active},
        });
      }
    }
  }
  std::sort(rows.begin(), rows.end(), [](const Json & left, const Json & right) {
    return left.at("started_at") > right.at("started_at");
  });
  return {
    {"ok", true}, {"target", name}, {"log_dir", impl_->config.log_directory.string()},
    {"sessions", std::move(rows)}, {"persist_error", info.value("persist_error", Json())},
    {"stamp", impl_->now()},
  };
}

ManagedLogDownload ManagedProcesses::open_saved_log(const std::string & name, const std::string & id) const
{
  impl_->require_log_target(name);
  if (!valid_session(id)) {
    throw std::invalid_argument("invalid log session id");
  }
  auto directory = open_target_directory(impl_->config.log_directory, name);
  if (directory.fd < 0) {
    throw ManagedLogMissing("log session " + id + " not found");
  }
  const auto filename = id + ".log";
  Descriptor file(::openat(directory.fd, filename.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW));
  if (file.fd < 0) {
    if (errno == ELOOP) {
      throw std::invalid_argument("refusing to read a symlinked log file");
    }
    if (errno == ENOENT) {
      throw ManagedLogMissing("log session " + id + " not found");
    }
    throw ManagedLogError("cannot open log file: " + std::string(std::strerror(errno)));
  }
  struct stat stat {};
  if (::fstat(file.fd, &stat) != 0) {
    throw ManagedLogError("cannot stat log file: " + std::string(std::strerror(errno)));
  }
  if (!S_ISREG(stat.st_mode)) {
    throw ManagedLogMissing("log session " + id + " not found");
  }
  return ManagedLogDownload(std::exchange(file.fd, -1), filename, stat.st_size);
}

std::pair<std::string, bool> ManagedProcesses::saved_log_tail(
  const std::string & name, const std::string & id, std::int64_t max_bytes) const
{
  const auto file = open_saved_log(name, id);
  const auto limit = static_cast<std::uint64_t>(std::clamp<std::int64_t>(max_bytes, 1024, 16 * 1024 * 1024));
  const auto length = std::min(file.size, limit);
  const auto start = file.size - length;
  std::string bytes(length, '\0');
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const auto count = ::pread(file.fd, bytes.data() + offset, bytes.size() - offset, start + offset);
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count < 0) {
      throw ManagedLogError("cannot read log file: " + std::string(std::strerror(errno)));
    }
    if (count == 0) {
      bytes.resize(offset);
      break;
    }
    offset += count;
  }
  if (start != 0) {
    const auto newline = bytes.find('\n');
    if (newline != std::string::npos) {
      bytes.erase(0, newline + 1);
    }
  }
  return {clean_text(bytes), start != 0};
}
}  // namespace web_control
