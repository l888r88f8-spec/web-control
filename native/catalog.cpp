#include "catalog.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <thread>

#include <fcntl.h>
#include <pwd.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_transport/recorder.hpp>
#include <yaml-cpp/yaml.h>

namespace web_control
{
namespace
{
namespace fs = std::filesystem;
using nlohmann::json;

constexpr std::array layers{
  "current_lidar", "global_map", "grid_terrain", "local_map", "mapcloud", "mapground", "obstacles"};
constexpr std::array roles{"throttle", "steering"};

class ApiError : public std::runtime_error
{
public:
  ApiError(int status, const std::string & message)
  : std::runtime_error(message), status(status) {}

  int status;
};

double wall_time()
{
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

json success(const std::string & message, double stamp)
{
  return {{"ok", true}, {"message", message}, {"stamp", stamp}};
}

json error(const std::string & message, double stamp)
{
  return {{"ok", false}, {"message", message}, {"stamp", stamp}};
}

const json & field(const json & object, const char * key)
{
  static const json missing = nullptr;
  const auto it = object.find(key);
  return it == object.end() ? missing : *it;
}

json object_or_empty(const json & value)
{
  return value.is_object() ? value : json::object();
}

json null_if_empty(const std::string & value)
{
  return value.empty() ? json(nullptr) : json(value);
}

template<typename T>
json null_if_missing(const std::optional<T> & value)
{
  return value ? json(*value) : json(nullptr);
}

std::string trim(const std::string & input)
{
  const auto first = input.find_first_not_of(" \t\n\r\f\v");
  if (first == std::string::npos) {
    return {};
  }
  return input.substr(first, input.find_last_not_of(" \t\n\r\f\v") - first + 1);
}

std::string quote(const std::string & value)
{
  return "'" + value + "'";
}

std::string require_string(const json & value, const std::string & message)
{
  if (!value.is_string()) {
    throw std::invalid_argument(message);
  }
  return value.get<std::string>();
}

std::string map_name(const json & value)
{
  const auto name = require_string(value, "map name must be a string");
  if (trim(name).empty()) {
    throw std::invalid_argument("map name must not be empty");
  }
  if (name == "." || name == "..") {
    throw std::invalid_argument("map name must not be " + quote(name));
  }
  if (name.find_first_of("/\\") != std::string::npos || name.find('\0') != std::string::npos) {
    throw std::invalid_argument("map name must be a single path component: " + quote(name));
  }
  return name;
}

std::string map_mode(const json & value)
{
  if (value == "auto" || value == "tiled" || value == "global") {
    return value.get<std::string>();
  }
  throw std::invalid_argument("mode must be one of: auto, tiled, global");
}

std::string planner(const json & value)
{
  if (value == "original" || value == "grid") {
    return value.get<std::string>();
  }
  throw std::invalid_argument("planner must be one of: original, grid");
}

bool require_bool(const json & value, const std::string & name)
{
  if (!value.is_boolean()) {
    throw std::invalid_argument(name + " must be a boolean");
  }
  return value.get<bool>();
}

bool saved_bool(const json & value, bool default_value = true)
{
  return value.is_boolean() ? value.get<bool>() : default_value;
}

fs::path runtime_home()
{
  const char * home = std::getenv("HOME");
  if (home && *home != '\0') {
    return home;
  }
  const auto * user = getpwuid(getuid());
  if (user && user->pw_dir) {
    return user->pw_dir;
  }
  throw std::runtime_error("home directory is required");
}

bool within(const fs::path & path, const fs::path & root)
{
  const auto [root_end, path_end] = std::mismatch(root.begin(), root.end(), path.begin(), path.end());
  (void)path_end;
  return root_end == root.end();
}

bool directory(const fs::path & path)
{
  std::error_code ec;
  return fs::is_directory(path, ec);
}

bool readable_file(const fs::path & path)
{
  std::error_code ec;
  return fs::is_regular_file(path, ec) && access(path.c_str(), R_OK) == 0;
}

bool lexists(const fs::path & path)
{
  std::error_code ec;
  return fs::exists(fs::symlink_status(path, ec));
}

fs::path resolve(const fs::path & path)
{
  auto resolved = fs::weakly_canonical(fs::absolute(path));
  if (resolved != resolved.root_path() && resolved.filename().empty()) {
    resolved = resolved.parent_path();
  }
  return resolved;
}

fs::path resolve_inside(const fs::path & path, const fs::path & root, const std::string & label)
{
  fs::path resolved;
  try {
    resolved = resolve(path);
  } catch (const fs::filesystem_error & exception) {
    throw std::invalid_argument("unable to resolve " + label + ": " + exception.code().message());
  }
  if (!within(resolved, root)) {
    throw std::invalid_argument(label + " escapes the map root");
  }
  return resolved;
}

fs::path required_artifact(
  const fs::path & child, const fs::path & resolved_child, const char * filename)
{
  const auto path = resolve_inside(child / filename, resolved_child, filename);
  if (!readable_file(path)) {
    throw std::invalid_argument(
      std::string(filename) + " is not a readable file for map " + quote(child.filename().string()));
  }
  return path;
}

bool optional_artifact(const fs::path & child, const fs::path & resolved_child, const char * filename)
{
  try {
    return readable_file(resolve_inside(child / filename, resolved_child, filename));
  } catch (const std::invalid_argument &) {
    return false;
  }
}

void validate_tile(const YAML::Node & tile, const fs::path & root)
{
  if (!tile.IsMap()) {
    throw std::invalid_argument("tile entry must be a mapping");
  }
  const auto value = tile["path"];
  if (!value.IsScalar() || value.Scalar().empty() ||
    (value.Tag() == "?" &&
    (std::isfinite(value.as<double>(NAN)) || value.as<bool>(false) == value.as<bool>(true))))
  {
    throw std::invalid_argument("tile entry is missing a path");
  }
  const fs::path relative = value.Scalar();
  if (relative.is_absolute()) {
    throw std::invalid_argument("tile path must be relative: " + quote(relative.string()));
  }
  if (relative.string().find('\0') != std::string::npos) {
    throw std::invalid_argument("tile entry is missing a path");
  }
  const auto resolved = resolve(root / relative);
  if (!within(resolved, root)) {
    throw std::invalid_argument("tile path escapes the tiled directory: " + quote(relative.string()));
  }
  std::error_code ec;
  if (!fs::is_regular_file(resolved, ec)) {
    throw std::invalid_argument("tile file is missing: " + quote(relative.string()));
  }
}

void validate_tiles(const YAML::Node & document, const fs::path & root)
{
  if (!document.IsMap()) {
    throw std::invalid_argument("manifest must be a mapping");
  }
  const auto version = document["schema_version"];
  if (!version.IsScalar() || version.Tag() == "!" || version.Tag() == "tag:yaml.org,2002:str" ||
    (version.as<double>(NAN) != 1.0 && version.as<int>(0) != 1))
  {
    throw std::invalid_argument(
      "unsupported schema_version " + (version.IsScalar() ? version.Scalar() : "None"));
  }
  const auto tiles = document["tiles"];
  if (!tiles.IsMap()) {
    throw std::invalid_argument("manifest must declare tiles");
  }
  for (const auto * kind : {"global", "ground"}) {
    const auto entries = tiles[kind];
    if (!entries.IsSequence()) {
      throw std::invalid_argument(std::string("tiles.") + kind + " must be a list");
    }
    for (const auto & entry : entries) {
      validate_tile(entry, root);
    }
  }
  const auto overview = document["overview"];
  if (overview && !overview.IsNull()) {
    if (!overview.IsMap()) {
      throw std::invalid_argument("overview must be a mapping");
    }
    validate_tile(overview, root);
  }
}

void find_tiled(MapEntry & entry, const fs::path & child, const fs::path & resolved_child)
{
  const auto tiled = child / "tiled";
  if (!lexists(tiled)) {
    return;
  }
  fs::path resolved;
  try {
    resolved = resolve_inside(tiled, resolved_child, "tiled");
  } catch (const std::invalid_argument & exception) {
    entry.warning = std::string("tiled directory is unusable: ") + exception.what();
    return;
  }
  if (!directory(resolved)) {
    entry.warning = "tiled path is not a directory";
    return;
  }
  fs::path manifest;
  try {
    manifest = resolve_inside(resolved / "manifest.yaml", resolved, "manifest.yaml");
  } catch (const std::invalid_argument & exception) {
    entry.warning = std::string("tiled manifest is unusable: ") + exception.what();
    return;
  }
  if (!readable_file(manifest)) {
    entry.warning = "tiled manifest is missing";
    return;
  }
  try {
    validate_tiles(YAML::LoadFile(manifest.string()), resolved);
    entry.tiled_path = resolved;
  } catch (const YAML::Exception & exception) {
    entry.warning = std::string("tiled manifest is unreadable: ") + exception.what();
  } catch (const std::exception & exception) {
    entry.warning = std::string("tiled manifest is invalid: ") + exception.what();
  }
}

MapEntry describe_map(const fs::path & root, const std::string & name)
{
  const auto child = root / name;
  const auto resolved = resolve_inside(child, root, "map " + quote(name));
  if (!directory(child)) {
    throw std::invalid_argument("map " + quote(name) + " is not a directory");
  }
  MapEntry entry;
  entry.name = name;
  entry.global_map_path = required_artifact(child, resolved, "GlobalMap.pcd");
  entry.ground_map_path = required_artifact(child, resolved, "GroundMap.pcd");
  entry.has_scan_context = optional_artifact(child, resolved, "sc_db.bin");
  find_tiled(entry, child, resolved);
  return entry;
}

json public_entry(const MapEntry & entry)
{
  return {
    {"name", entry.name}, {"tiled", entry.tiled_path.has_value()},
    {"has_scan_context", entry.has_scan_context}, {"warning", null_if_empty(entry.warning)}};
}

void resolve_mode(MapSelection & selection)
{
  const auto & entry = *selection.entry;
  if (selection.mode == "tiled" && !entry.tiled_path) {
    const auto detail = entry.warning.empty() ? "no usable tiled map" : entry.warning;
    selection.mode_message =
      "map " + quote(entry.name) + " has no usable tiled map (" + detail +
      "); choose Auto or Global before starting localization";
  } else {
    selection.effective_mode = selection.mode == "global" || !entry.tiled_path ? "global" : "tiled";
  }
  selection.effective_auto_relocalization =
    selection.requested_auto_relocalization && entry.has_scan_context;
  if (!entry.has_scan_context) {
    selection.auto_relocalization_message =
      "selected map has no sc_db.bin; automatic relocalization is unavailable";
  }
}

json public_selection(const MapSelection & selection)
{
  return {
    {"name", null_if_missing(selection.name)}, {"valid", selection.entry.has_value()},
    {"entry", selection.entry ? public_entry(*selection.entry) : json(nullptr)},
    {"message", null_if_empty(selection.message)}, {"mode", selection.mode},
    {"effective_mode", null_if_missing(selection.effective_mode)},
    {"mode_valid", selection.effective_mode.has_value()},
    {"mode_message", null_if_empty(selection.mode_message)},
    {"auto_relocalization", selection.requested_auto_relocalization},
    {"auto_relocalization_available", selection.entry && selection.entry->has_scan_context},
    {"effective_auto_relocalization", selection.effective_auto_relocalization},
    {"auto_relocalization_message", null_if_empty(selection.auto_relocalization_message)}};
}

void atomic_write(const fs::path & destination, const json & value)
{
  const auto text = value.dump(-1, ' ', true) + "\n";
  if (!destination.parent_path().empty()) {
    fs::create_directories(destination.parent_path());
  }
  std::string temporary = destination.string() + ".XXXXXX";
  const int fd = mkostemp(temporary.data(), O_CLOEXEC);
  if (fd == -1) {
    throw std::system_error(errno, std::generic_category(), "create preference file");
  }
  int failure = 0;
  for (std::size_t offset = 0; offset < text.size();) {
    const auto count = write(fd, text.data() + offset, text.size() - offset);
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count <= 0) {
      failure = count < 0 ? errno : EIO;
      break;
    }
    offset += static_cast<std::size_t>(count);
  }
  if (!failure && fsync(fd) == -1) {
    failure = errno;
  }
  if (close(fd) == -1 && !failure) {
    failure = errno;
  }
  if (!failure && rename(temporary.c_str(), destination.c_str()) == -1) {
    failure = errno;
  }
  if (failure) {
    unlink(temporary.c_str());
    throw std::system_error(failure, std::generic_category(), "write preference file");
  }
}

double checked_number(const json & value, const std::string & key)
{
  if (!value.is_number()) {
    throw std::invalid_argument(key + " must be a number");
  }
  const auto number = value.get<double>();
  if (!std::isfinite(number)) {
    throw std::invalid_argument(key + " must be finite");
  }
  return number;
}

json stick(const json & value)
{
  if (!value.is_object()) {
    throw std::invalid_argument("stick preferences must be an object");
  }
  json result = json::object();
  for (const char * key : {"size", "x", "y"}) {
    if (!value.contains(key)) {
      throw std::invalid_argument(std::string("missing ") + key);
    }
    const double number = checked_number(value.at(key), key);
    if (std::string_view(key) == "size") {
      if (number < 140 || number > 260) {
        throw std::invalid_argument("size must be within [140, 260]");
      }
    } else if (number < 0.0) {
      throw std::invalid_argument(std::string(key) + " must be nonnegative");
    }
    result[key] = number;
  }
  return result;
}

json saved_stick(const json & value, bool legacy)
{
  json result = {{"size", 180.0}, {"x", nullptr}, {"y", nullptr}};
  const auto & size = field(value, "size");
  if (size.is_number() && size.get<double>() >= 140 && size.get<double>() <= 260) {
    result["size"] = size.get<double>();
  }
  if (!legacy) {
    for (const char * key : {"x", "y"}) {
      const auto & offset = field(value, key);
      if (offset.is_number() && std::isfinite(offset.get<double>()) && offset.get<double>() >= 0) {
        result[key] = offset.get<double>();
      }
    }
  }
  return result;
}

json saved_joystick(const json & value)
{
  const bool legacy = !value.contains("throttle") && !value.contains("steering");
  json result = json::object();
  for (const auto * role : roles) {
    result[role] = saved_stick(legacy ? value : field(value, role), legacy);
  }
  return result;
}

json saved_layers(const json & value)
{
  json result = json::object();
  for (const auto * layer : layers) {
    result[layer] = saved_bool(field(value, layer));
  }
  return result;
}

json validate_layers(const json & value)
{
  if (!value.is_object()) {
    throw std::invalid_argument("layer preferences must be an object");
  }
  if (value.empty()) {
    throw std::invalid_argument("layer preferences must not be empty");
  }
  for (const auto & [key, enabled] : value.items()) {
    if (std::find(layers.begin(), layers.end(), key) == layers.end()) {
      throw std::invalid_argument("unknown layer: " + key);
    }
    require_bool(enabled, "layer " + key);
  }
  return value;
}

fs::path path_argument(const std::string & value, const std::string & label)
{
  if (value.find('\0') != std::string::npos) {
    throw std::invalid_argument(label + " must not contain NUL bytes");
  }
  const fs::path path(value);
  if (!path.is_absolute()) {
    throw std::invalid_argument(label + " must be an absolute path");
  }
  if (std::find(path.begin(), path.end(), "..") != path.end()) {
    throw std::invalid_argument(label + " must not contain '..' path components");
  }
  return path;
}

std::string folded(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char letter) {
    return static_cast<char>(std::tolower(letter));
  });
  return text;
}

json browse(const std::string & path, const fs::path & root, const fs::path & home, bool save)
{
  const auto scope = save ? "save map" : "recording";
  const auto value = trim(path);
  const auto requested = value.empty() ? root : path_argument(value, "browse path");
  if (!directory(requested)) {
    throw std::invalid_argument("browse path is not a directory");
  }
  const auto current = fs::canonical(requested);
  if (!within(current, root)) {
    throw std::invalid_argument(std::string("browse path is outside the allowed ") + scope + " root");
  }
  std::error_code ec;
  fs::directory_iterator iterator(current, ec);
  std::vector<fs::path> directories;
  for (; !ec && iterator != fs::directory_iterator(); iterator.increment(ec)) {
    const auto & candidate = iterator->path();
    if (!directory(candidate)) {
      continue;
    }
    std::error_code candidate_error;
    const auto resolved = fs::canonical(candidate, candidate_error);
    if (!candidate_error && within(resolved, root)) {
      directories.push_back(candidate);
    }
  }
  if (ec) {
    throw std::invalid_argument("directory is not readable");
  }
  std::sort(directories.begin(), directories.end(), [](const fs::path & lhs, const fs::path & rhs) {
    const auto a = lhs.filename().string();
    const auto b = rhs.filename().string();
    const auto folded_a = folded(a);
    const auto folded_b = folded(b);
    return folded_a == folded_b ? a < b : folded_a < folded_b;
  });
  json entries = json::array();
  for (const auto & child : directories) {
    entries.push_back({
      {"name", child.filename().string()}, {"path", child.string()},
      {"writable", access(child.c_str(), W_OK) == 0}});
  }
  auto default_directory = within(home, root) ? home : root;
  if (save) {
    const auto home_maps = resolve(home / "map");
    if (within(home_maps, root) && directory(home_maps)) {
      default_directory = home_maps;
    } else if (!directory(default_directory)) {
      default_directory = root;
    }
  }
  return {
    {"path", current.string()},
    {"parent", current == root ? json(nullptr) : json(current.parent_path().string())},
    {"writable", access(current.c_str(), W_OK) == 0},
    {"entries", entries},
    {"default_directory", default_directory.string()}};
}

fs::path bag_output(const json & payload, const fs::path & root)
{
  if (!payload.is_object()) {
    throw std::invalid_argument("payload must be an object");
  }
  const auto directory_name = trim(require_string(
    field(payload, "directory"), "save directory must be a string path"));
  if (directory_name.empty()) {
    throw std::invalid_argument("save directory is required");
  }
  const auto requested = path_argument(directory_name, "save directory");
  const auto name = trim(require_string(field(payload, "name"), "bag name must be a string"));
  const auto alphanumeric = [](char letter) {
    return (letter >= 'a' && letter <= 'z') || (letter >= 'A' && letter <= 'Z') ||
           (letter >= '0' && letter <= '9');
  };
  if (name.empty() || name.size() > 128 || !alphanumeric(name.front()) ||
    !std::all_of(name.begin(), name.end(), [&](char letter) {
      return alphanumeric(letter) || letter == '.' || letter == '_' || letter == '-';
    }))
  {
    throw std::invalid_argument(
      "bag name must start with a letter or number and use only letters, numbers, dot, underscore or dash");
  }
  if (!fs::exists(requested)) {
    throw std::invalid_argument("save directory does not exist");
  }
  if (!directory(requested)) {
    throw std::invalid_argument("save path is not a directory");
  }
  const auto resolved = fs::canonical(requested);
  if (!within(resolved, root)) {
    throw std::invalid_argument("save directory is outside the allowed recording root");
  }
  if (access(resolved.c_str(), W_OK) != 0) {
    throw std::invalid_argument("save directory is not writable");
  }
  const auto output = resolved / name;
  if (lexists(output)) {
    throw ApiError(409, "a bag already exists at " + output.string());
  }
  return output;
}

std::vector<std::string> selected_topics(const json & payload, const CatalogConfig::TopicGraph & graph)
{
  const auto & topics = field(payload, "topics");
  if (!topics.is_array() || topics.empty()) {
    throw std::invalid_argument("select at least one topic to record");
  }
  if (topics.size() > 64) {
    throw std::invalid_argument("select no more than 64 topics");
  }
  std::set<std::string> available;
  for (const auto & [name, types] : graph) {
    (void)types;
    available.insert(name);
  }
  std::set<std::string> seen;
  std::vector<std::string> selected;
  for (const auto & topic : topics) {
    const auto value = trim(require_string(topic, "topic names must be non-empty strings"));
    if (value.empty()) {
      throw std::invalid_argument("topic names must be non-empty strings");
    }
    if (!seen.insert(value).second) {
      throw std::invalid_argument("duplicate topic: " + value);
    }
    if (available.count(value) == 0) {
      throw std::invalid_argument("topic is not available: " + value);
    }
    selected.push_back(value);
  }
  return selected;
}

double save_resolution(const json & payload, const char * key)
{
  const auto & value = field(payload, key);
  if (value.is_null()) {
    return -1.0;
  }
  double number;
  if (value.is_number()) {
    number = value.get<double>();
  } else if (value.is_boolean()) {
    number = value.get<bool>() ? 1.0 : 0.0;
  } else {
    try {
      const auto text = trim(value.get<std::string>());
      std::size_t length = 0;
      number = std::stod(text, &length);
      if (length != text.size()) {
        throw std::invalid_argument("incomplete numeric value");
      }
    } catch (const std::out_of_range &) {
      throw std::invalid_argument(std::string(key) + " must be finite");
    } catch (const std::exception &) {
      throw std::invalid_argument(std::string(key) + " must be numeric");
    }
  }
  if (!std::isfinite(number)) {
    throw std::invalid_argument(std::string(key) + " must be finite");
  }
  return number;
}

SaveMapRequest save_request(const json & payload, const fs::path & root, const fs::path & home)
{
  if (!payload.is_object()) {
    throw std::invalid_argument("payload must be an object");
  }
  for (const auto * key : {"use_dufomap", "dufomap_mode"}) {
    if (payload.contains(key)) {
      throw std::invalid_argument(
        std::string(key) + " is no longer supported; DUFOMap is controlled by the mapping config dufomap.enable");
    }
  }
  const auto destination = trim(require_string(
    field(payload, "destination"), "destination must be a string path"));
  if (destination.empty()) {
    throw std::invalid_argument("destination is required");
  }
  auto path = path_argument(destination, "destination").lexically_normal();
  if (path == path.root_path()) {
    throw std::invalid_argument("destination must not be the filesystem root /");
  }
  if (path.filename().empty()) {
    path = path.parent_path();
  }
  if (fs::is_symlink(fs::symlink_status(path))) {
    throw std::invalid_argument("destination must not be a symlink");
  }
  if (fs::exists(path) && !directory(path)) {
    throw std::invalid_argument("destination exists and is not a directory");
  }
  auto ancestor = path.parent_path();
  while (!lexists(ancestor)) {
    ancestor = ancestor.parent_path();
  }
  if (!directory(ancestor)) {
    throw std::invalid_argument("destination parent is not a directory");
  }
  const auto resolved = resolve(path);
  if (resolved == home) {
    throw std::invalid_argument("destination must not be the home directory; choose a subdirectory");
  }
  if (!within(resolved, root)) {
    throw std::invalid_argument("destination is outside the allowed save root");
  }
  return {
    path.string(), save_resolution(payload, "resolution"),
    save_resolution(payload, "ground_resolution"),
    !payload.contains("save_ground_map") || require_bool(payload.at("save_ground_map"), "save_ground_map"),
    !payload.contains("save_tiled_map") || require_bool(payload.at("save_tiled_map"), "save_tiled_map")};
}

struct SaveMapJob
{
  std::mutex mutex;
  unsigned token = 0;
  bool running = false;
  std::optional<bool> succeeded;
  std::string destination;
  std::string message;
  std::optional<double> started_at;
  std::optional<double> finished_at;

  std::string state() const
  {
    if (running) {
      return "running";
    }
    if (succeeded.has_value()) {
      return *succeeded ? "succeeded" : "failed";
    }
    return "idle";
  }

  void finish(unsigned completed_token, bool success, std::string result, double stamp)
  {
    if (!running || completed_token != token) {
      return;
    }
    running = false;
    succeeded = success;
    message = result.empty() ? (success ? "map saved" : "map save failed") : std::move(result);
    finished_at = stamp;
  }
};

class HeadlessBagKeyboard final : public KeyboardHandler
{
public:
  HeadlessBagKeyboard()
  : KeyboardHandler(::read, [](int) {return 0;}, ::tcgetattr, ::tcsetattr, false) {}
};

class NativeRecording
{
public:
  NativeRecording(const fs::path & path, const std::vector<std::string> & topics)
  {
    rosbag2_storage::StorageOptions storage;
    storage.uri = path.string();
    storage.storage_id = "sqlite3";
    storage.max_cache_size = 100 * 1024 * 1024;
    rosbag2_transport::RecordOptions options;
    options.topics = topics;
    options.rmw_serialization_format = rmw_get_serialization_format();
    rclcpp::NodeOptions node_options;
    node_options.use_global_arguments(false);
    recorder_ = std::make_shared<rosbag2_transport::Recorder>(
      std::make_shared<rosbag2_cpp::Writer>(), std::make_shared<HeadlessBagKeyboard>(),
      storage, options, "rosbag2_recorder", node_options);
    executor_.add_node(recorder_);
    recorder_->record();
    thread_ = std::thread([this] {
      try {
        while (!stopping_ && rclcpp::ok()) {
          executor_.spin_once(std::chrono::milliseconds{100});
        }
      } catch (const std::exception & exception) {
        failure_ = exception.what();
      }
      exited_ = true;
    });
  }

  ~NativeRecording()
  {
    try {
      stop();
    } catch (...) {
      // Shutdown is best effort after the executor has joined.
    }
  }

  bool exited() const
  {
    return exited_;
  }

  void stop()
  {
    if (!recorder_) {
      return;
    }
    stopping_ = true;
    if (thread_.joinable()) {
      thread_.join();
    }
    auto recorder = std::move(recorder_);
    recorder->stop();
    executor_.remove_node(recorder);
    recorder.reset();
    if (!failure_.empty()) {
      throw std::runtime_error(failure_);
    }
  }

private:
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<rosbag2_transport::Recorder> recorder_;
  std::atomic<bool> stopping_{false};
  std::atomic<bool> exited_{false};
  std::thread thread_;
  std::string failure_;
};
}

class Catalog::Implementation
{
public:
  explicit Implementation(CatalogConfig config)
  : config_(std::move(config)),
    home_(resolve(config_.home.empty() ? runtime_home() : config_.home)),
    maps_root_(resolve(config_.map_root.empty() ? home_ / "map" : config_.map_root)),
    recording_root_(resolve(root_argument(config_.recording_root))),
    selection_path_(config_.selection_path.empty() ?
      home_ / ".config/web_control/selected_map.json" : config_.selection_path),
    clock_(config_.clock ? config_.clock : wall_time)
  {
    if (!directory(recording_root_)) {
      throw std::invalid_argument("recording root does not exist");
    }
  }

  ~Implementation()
  {
    try {
      stop_recording();
    } catch (...) {
      // Destruction must complete if the recorder has already shut down.
    }
  }

  std::optional<CatalogResponse> route(
    const std::string & method, const std::string & path, const std::string & browse_path,
    const json & payload)
  {
    try {
      if (method == "GET") {
        if (path == "/api/maps") {
          return CatalogResponse{200, maps()};
        }
        if (path == "/api/ui/preferences") {
          return CatalogResponse{200, preferences()};
        }
        if (path == "/api/mapping/save_map") {
          return CatalogResponse{200, save_map_status()};
        }
        if (path == "/api/mapping/record") {
          return CatalogResponse{200, recording_status()};
        }
        if (path == "/api/mapping/record/topics") {
          return CatalogResponse{200, recording_topics()};
        }
        if (path == "/api/mapping/save_map/browse" || path == "/api/mapping/record/browse") {
          const bool save = path == "/api/mapping/save_map/browse";
          auto body = success(save ? "save map directories" : "recording directories", clock_());
          body.update(browse(browse_path, recording_root_, home_, save));
          return CatalogResponse{200, body};
        }
      }
      if (method == "POST") {
        if (path == "/api/maps/select") {
          return CatalogResponse{200, select_map(payload)};
        }
        if (path == "/api/ui/preferences") {
          return CatalogResponse{200, save_preferences(payload)};
        }
        if (path == "/api/navigation/planner") {
          return CatalogResponse{200, select_planner(payload)};
        }
        if (path == "/api/mapping/save_map") {
          return CatalogResponse{200, start_save_map(payload)};
        }
        if (path == "/api/mapping/record/start") {
          return CatalogResponse{200, start_recording(payload)};
        }
        if (path == "/api/mapping/record/stop") {
          return CatalogResponse{200, stop_recording()};
        }
      }
      return std::nullopt;
    } catch (const ApiError & exception) {
      return CatalogResponse{exception.status, error(exception.what(), clock_())};
    } catch (const std::invalid_argument & exception) {
      return CatalogResponse{400, error(exception.what(), clock_())};
    }
  }

  MapSelection selection() const
  {
    std::lock_guard lock(store_mutex_);
    return selection(read());
  }

  std::string navigation_planner() const
  {
    std::lock_guard lock(store_mutex_);
    const auto saved = read();
    try {
      return planner(field(field(saved, "ui"), "navigation_planner"));
    } catch (const std::invalid_argument &) {
      return "original";
    }
  }

  bool topic_logging_enabled() const
  {
    std::lock_guard lock(store_mutex_);
    const auto saved = read();
    return saved_bool(field(field(saved, "ui"), "topic_logging_enabled"));
  }

  bool set_topic_logging_enabled(const json & payload)
  {
    if (!payload.is_object()) {
      throw std::invalid_argument("topic logging payload must be an object");
    }
    if (payload.size() != 1 || !payload.contains("enabled")) {
      throw std::invalid_argument("topic logging payload must contain only enabled");
    }
    const auto enabled = require_bool(payload.at("enabled"), "enabled");
    std::lock_guard lock(store_mutex_);
    auto saved = read();
    auto ui = object_or_empty(field(saved, "ui"));
    if (enabled == saved_bool(field(ui, "topic_logging_enabled"))) {
      return enabled;
    }
    ui["topic_logging_enabled"] = enabled;
    saved["ui"] = std::move(ui);
    persist(saved, "could not save topic logging preference");
    return enabled;
  }

  json save_map_status() const
  {
    const bool available = save_map_available();
    const auto stamp = clock_();
    std::lock_guard lock(save_job_->mutex);
    if (!available && save_job_->running) {
      save_job_->finish(save_job_->token, false, "save map service unavailable", stamp);
    }
    return {
      {"available", available}, {"state", save_job_->state()}, {"running", save_job_->running},
      {"success", null_if_missing(save_job_->succeeded)}, {"destination", save_job_->destination},
      {"message", save_job_->message}, {"started_at", null_if_missing(save_job_->started_at)},
      {"finished_at", null_if_missing(save_job_->finished_at)}, {"stamp", stamp}};
  }

  json recording_status()
  {
    std::unique_lock lock(recording_mutex_);
    if (recording_ && recording_->exited() && !recording_finalizing_) {
      finish_recording(lock);
    }
    return recording_snapshot();
  }

private:
  static fs::path root_argument(const fs::path & path)
  {
    const fs::path root = trim(path.string());
    if (!root.is_absolute()) {
      throw std::invalid_argument("recording root must be an absolute directory");
    }
    return root;
  }

  json read() const
  {
    std::ifstream input(selection_path_);
    if (!input) {
      return json::object();
    }
    return object_or_empty(json::parse(input, nullptr, false));
  }

  void persist(const json & saved, const std::string & message) const
  {
    try {
      atomic_write(selection_path_, saved);
    } catch (const std::exception & exception) {
      throw ApiError(503, message + ": " + exception.what());
    }
  }

  MapSelection selection(const json & saved) const
  {
    MapSelection result;
    try {
      result.mode = map_mode(field(saved, "mode"));
    } catch (const std::invalid_argument &) {
      // Older or invalid configurations use the default loading mode.
    }
    result.requested_auto_relocalization = saved_bool(field(saved, "auto_relocalization"));
    try {
      result.name = map_name(field(saved, "name"));
    } catch (const std::invalid_argument &) {
      result.message = "no map selected";
      result.mode_message = result.message;
      result.auto_relocalization_message = result.message;
      return result;
    }
    try {
      result.entry = describe_map(maps_root_, *result.name);
      resolve_mode(result);
    } catch (const std::invalid_argument & exception) {
      result.message = exception.what();
      result.auto_relocalization_message = "selected map " + quote(*result.name) + " is unavailable";
      result.mode_message = result.auto_relocalization_message + "; choose another map";
    }
    return result;
  }

  json maps() const
  {
    std::vector<std::string> names;
    std::error_code ec;
    fs::directory_iterator iterator(maps_root_, ec);
    for (; !ec && iterator != fs::directory_iterator(); iterator.increment(ec)) {
      names.push_back(iterator->path().filename().string());
    }
    std::sort(names.begin(), names.end());
    json entries = json::array();
    for (const auto & name : names) {
      try {
        entries.push_back(public_entry(describe_map(maps_root_, map_name(name))));
      } catch (const std::invalid_argument &) {
        // Invalid catalog entries are skipped independently.
      }
    }
    return {
      {"ok", true}, {"map_root", "~/map"}, {"maps", entries},
      {"selection", public_selection(selection())}, {"stamp", clock_()}};
  }

  json select_map(const json & payload)
  {
    if (!payload.is_object()) {
      throw std::invalid_argument("payload must be an object");
    }
    const bool has_name = payload.contains("name");
    const bool has_mode = payload.contains("mode");
    const bool has_auto = payload.contains("auto_relocalization");
    if (!has_name && !has_mode && !has_auto) {
      throw std::invalid_argument("payload must include name, mode, or auto_relocalization");
    }
    std::unique_lock lock(store_mutex_);
    auto saved = read();
    auto result = selection(saved);
    if (has_name) {
      const auto name = map_name(payload.at("name"));
      const auto entry = describe_map(maps_root_, name);
      result.name = name;
      result.entry = entry;
    } else if (!result.name) {
      throw std::invalid_argument(
        "no map selected; choose a map before changing the loading mode or automatic relocalization");
    } else if (!result.entry) {
      throw std::invalid_argument(
        "selected map " + quote(*result.name) + " is no longer available: " + result.message);
    }
    if (has_mode) {
      result.mode = map_mode(payload.at("mode"));
    }
    if (has_auto) {
      result.requested_auto_relocalization = require_bool(
        payload.at("auto_relocalization"), "auto_relocalization");
    }
    result.message.clear();
    result.mode_message.clear();
    result.effective_mode.reset();
    result.auto_relocalization_message.clear();
    resolve_mode(result);
    if (!result.effective_mode) {
      throw std::invalid_argument(result.mode_message);
    }
    if (has_auto && result.requested_auto_relocalization && !result.entry->has_scan_context) {
      throw std::invalid_argument(
        "automatic relocalization is unavailable: selected map has no sc_db.bin");
    }

    const bool localization = is_managed_running("localization");
    const bool navigation = is_managed_running("navigation");
    if (localization || navigation) {
      const std::string running = localization ?
        (navigation ? "localization and navigation" : "localization") : "navigation";
      throw ApiError(
        409, "stop " + running + " before changing the map, loading mode, or automatic relocalization");
    }
    if (has_name || has_mode) {
      saved["name"] = *result.name;
    }
    if (has_mode) {
      saved["mode"] = result.mode;
    }
    if (has_auto) {
      saved["auto_relocalization"] = result.requested_auto_relocalization;
    }
    persist(saved, "unable to save map selection");
    lock.unlock();
    if (config_.map_selected) {
      config_.map_selected(result, has_name);
    }
    auto response = success("selected map " + *result.name, clock_());
    response["selection"] = public_selection(result);
    return response;
  }

  bool is_managed_running(const std::string & target) const
  {
    return config_.managed_running && config_.managed_running(target);
  }

  json select_planner(const json & payload)
  {
    if (!payload.is_object() || payload.size() != 1 || !payload.contains("planner")) {
      throw std::invalid_argument("planner payload must contain only planner");
    }
    const auto value = planner(payload.at("planner"));
    std::lock_guard lock(store_mutex_);
    if (is_managed_running("navigation")) {
      throw ApiError(409, "stop navigation before changing the planner");
    }
    auto saved = read();
    auto ui = object_or_empty(field(saved, "ui"));
    ui["navigation_planner"] = value;
    saved["ui"] = std::move(ui);
    persist(saved, "unable to save navigation planner");
    auto response = success("navigation planner set to " + value, clock_());
    response["planner"] = value;
    return response;
  }

  json preferences() const
  {
    std::lock_guard lock(store_mutex_);
    const auto saved = read();
    const auto & ui = field(saved, "ui");
    return {
      {"ok", true}, {"joystick", saved_joystick(field(ui, "joystick"))},
      {"layers", saved_layers(field(ui, "layers"))}, {"stamp", clock_()}};
  }

  json save_preferences(const json & payload)
  {
    if (!payload.is_object()) {
      throw std::invalid_argument("ui preferences must be an object");
    }
    const bool has_joystick = payload.contains("throttle") || payload.contains("steering");
    const bool has_layers = payload.contains("layers");
    if (!has_joystick && !has_layers) {
      throw std::invalid_argument("no ui preference in payload");
    }
    auto response = success("ui preferences saved", clock_());
    if (has_joystick) {
      auto joystick = json::object();
      for (const auto * role : roles) {
        if (!payload.contains(role)) {
          throw std::invalid_argument(std::string("missing ") + role);
        }
        joystick[role] = stick(payload.at(role));
      }
      response["joystick"] = std::move(joystick);
    }
    if (has_layers) {
      response["layers"] = validate_layers(payload.at("layers"));
    }
    std::lock_guard lock(store_mutex_);
    auto saved = read();
    auto ui = object_or_empty(field(saved, "ui"));
    if (has_joystick) {
      ui["joystick"] = response["joystick"];
    }
    if (has_layers) {
      auto layers = object_or_empty(field(ui, "layers"));
      layers.update(response["layers"]);
      ui["layers"] = std::move(layers);
    }
    saved["ui"] = std::move(ui);
    persist(saved, "unable to save UI preferences");
    return response;
  }

  bool save_map_available() const
  {
    try {
      return config_.save_map && config_.save_map_available && config_.save_map_available();
    } catch (const std::exception &) {
      return false;
    }
  }

  json start_save_map(const json & payload)
  {
    const auto request = save_request(payload, recording_root_, home_);
    if (!save_map_available()) {
      throw ApiError(503, "save map service unavailable");
    }
    const auto clock = clock_;
    const auto stamp = clock();
    unsigned token;
    {
      std::lock_guard lock(save_job_->mutex);
      if (save_job_->running) {
        throw ApiError(409, "save map already running");
      }
      token = ++save_job_->token;
      save_job_->destination = request.destination;
      save_job_->message.clear();
      save_job_->running = true;
      save_job_->succeeded.reset();
      save_job_->started_at = stamp;
      save_job_->finished_at.reset();
    }
    try {
      std::weak_ptr<SaveMapJob> weak_job = save_job_;
      config_.save_map(request, [weak_job, token, clock](bool success, std::string message) {
        if (const auto job = weak_job.lock()) {
          const auto stamp = clock();
          std::lock_guard lock(job->mutex);
          job->finish(token, success, std::move(message), stamp);
        }
      });
    } catch (const std::exception & exception) {
      const std::string message = std::string("save map request failed: ") + exception.what();
      const auto finished_at = clock();
      std::lock_guard lock(save_job_->mutex);
      save_job_->finish(token, false, message, finished_at);
      throw ApiError(503, message);
    }
    auto response = success("map save started", clock());
    std::lock_guard lock(save_job_->mutex);
    response.update({
      {"available", true}, {"state", save_job_->state()}, {"destination", request.destination}});
    return response;
  }

  CatalogConfig::TopicGraph topic_graph() const
  {
    return config_.topics ? config_.topics() : CatalogConfig::TopicGraph{};
  }

  json recording_topics() const
  {
    auto graph = topic_graph();
    std::sort(graph.begin(), graph.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.first < rhs.first;
    });
    auto topics = json::array();
    for (const auto & [name, types] : graph) {
      topics.push_back({{"name", name}, {"types", types}});
    }
    auto response = success("recording topics", clock_());
    response["topics"] = std::move(topics);
    return response;
  }

  json recording_snapshot() const
  {
    return {
      {"available", rclcpp::ok()}, {"state", recording_state_},
      {"running", recording_ && !recording_->exited()},
      {"topics", recording_topics_},
      {"directory", recording_output_.empty() ? "" : recording_output_.parent_path().string()},
      {"output_dir", recording_output_.string()}, {"message", recording_message_},
      {"returncode", null_if_missing(recording_returncode_)},
      {"started_at", null_if_missing(recording_started_at_)},
      {"finished_at", null_if_missing(recording_finished_at_)},
      {"log_tail", recording_log_}, {"stamp", clock_()}};
  }

  json start_recording(const json & payload)
  {
    const auto output = bag_output(payload, recording_root_);
    const auto topics = selected_topics(payload, topic_graph());
    std::unique_lock lock(recording_mutex_);
    if (recording_ && recording_->exited() && !recording_finalizing_) {
      finish_recording(lock);
    }
    if (recording_) {
      throw ApiError(409, "a recording is already in progress");
    }
    if (lexists(output)) {
      throw ApiError(409, "a bag already exists at " + output.string());
    }
    recording_output_ = output;
    recording_topics_ = topics;
    recording_started_at_ = clock_();
    recording_finished_at_.reset();
    recording_returncode_.reset();
    recording_log_.clear();
    try {
      if (!rclcpp::ok()) {
        throw std::runtime_error("ROS recording context is unavailable");
      }
      recording_ = std::make_unique<NativeRecording>(output, topics);
    } catch (const std::exception & exception) {
      recording_state_ = "failed";
      recording_message_ = std::string("failed to start rosbag recording: ") + exception.what();
      recording_log_.push_back(exception.what());
      recording_finished_at_ = clock_();
      throw ApiError(503, recording_message_);
    }
    recording_state_ = "recording";
    recording_message_ = "Recording " + std::to_string(topics.size()) + " topics.";
    auto response = success("recording started", clock_());
    response["recording"] = recording_snapshot();
    return response;
  }

  void finish_recording(std::unique_lock<std::mutex> & lock)
  {
    const bool requested = recording_state_ == "stopping";
    recording_finalizing_ = true;
    lock.unlock();
    std::string failure;
    try {
      recording_->stop();
    } catch (const std::exception & exception) {
      failure = exception.what();
    }
    lock.lock();
    recording_.reset();
    recording_finished_at_ = clock_();
    recording_returncode_ = failure.empty() ? 0 : 1;
    std::error_code ec;
    const auto metadata = recording_output_ / "metadata.yaml";
    const bool finalized = fs::is_regular_file(metadata, ec) && fs::file_size(metadata, ec) > 0 && !ec;
    if (!failure.empty()) {
      recording_state_ = "failed";
      recording_message_ = "rosbag recording failed: " + failure;
      recording_log_.push_back(std::move(failure));
    } else if (!finalized) {
      recording_state_ = "failed";
      recording_message_ = requested ?
        "Recording stopped but metadata.yaml was not finalized." :
        "rosbag recording exited without metadata.yaml.";
    } else {
      recording_state_ = "stopped";
      recording_message_ = requested ?
        "Recording stopped; bag finalized at " + recording_output_.string() + "." :
        "Recording finished; bag saved at " + recording_output_.string() + ".";
    }
    recording_finalizing_ = false;
    recording_finalized_.notify_all();
  }

  json stop_recording()
  {
    std::unique_lock lock(recording_mutex_);
    recording_finalized_.wait(lock, [this] {return !recording_finalizing_;});
    if (recording_) {
      if (!recording_->exited()) {
        recording_state_ = "stopping";
        recording_message_ = "Stopping recording; waiting for rosbag to finalize.";
      }
      finish_recording(lock);
    }
    auto response = success("recording stop requested", clock_());
    response["recording"] = recording_snapshot();
    return response;
  }

  const CatalogConfig config_;
  const fs::path home_;
  const fs::path maps_root_;
  const fs::path recording_root_;
  const fs::path selection_path_;
  const std::function<double()> clock_;
  inline static std::mutex store_mutex_;
  const std::shared_ptr<SaveMapJob> save_job_ = std::make_shared<SaveMapJob>();
  std::mutex recording_mutex_;
  std::condition_variable recording_finalized_;
  std::unique_ptr<NativeRecording> recording_;
  bool recording_finalizing_ = false;
  std::string recording_state_ = "idle";
  std::string recording_message_ = "No recording in progress.";
  std::vector<std::string> recording_topics_;
  fs::path recording_output_;
  std::optional<int> recording_returncode_;
  std::optional<double> recording_started_at_;
  std::optional<double> recording_finished_at_;
  std::vector<std::string> recording_log_;
};

Catalog::Catalog(CatalogConfig config)
: implementation_(std::make_unique<Implementation>(std::move(config))) {}

Catalog::~Catalog() = default;

std::optional<CatalogResponse> Catalog::route(
  const std::string & method, const std::string & path, const std::string & browse_path,
  const json & payload)
{
  return implementation_->route(method, path, browse_path, payload);
}

MapSelection Catalog::selection() const
{
  return implementation_->selection();
}

std::string Catalog::navigation_planner() const
{
  return implementation_->navigation_planner();
}

bool Catalog::topic_logging_enabled() const
{
  return implementation_->topic_logging_enabled();
}

bool Catalog::set_topic_logging_enabled(const json & payload)
{
  return implementation_->set_topic_logging_enabled(payload);
}

json Catalog::recording_status() const
{
  return implementation_->recording_status();
}

json Catalog::save_map_status() const
{
  return implementation_->save_map_status();
}
}
