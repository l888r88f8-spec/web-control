#pragma once

#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace web_control
{
struct CatalogResponse
{
  int status;
  nlohmann::json body;
};

struct MapEntry
{
  std::string name;
  std::filesystem::path global_map_path;
  std::filesystem::path ground_map_path;
  std::optional<std::filesystem::path> tiled_path;
  bool has_scan_context = false;
  std::string warning;
};

struct MapSelection
{
  std::optional<std::string> name;
  std::optional<MapEntry> entry;
  std::string message;
  std::string mode = "auto";
  std::optional<std::string> effective_mode;
  std::string mode_message;
  bool requested_auto_relocalization = true;
  bool effective_auto_relocalization = false;
  std::string auto_relocalization_message;
};

struct SaveMapRequest
{
  std::string destination;
  double resolution;
  double ground_resolution;
  bool save_ground_map;
  bool save_tiled_map;
};

struct CatalogConfig
{
  using SaveMapCompletion = std::function<void(bool, std::string)>;
  using TopicGraph = std::vector<std::pair<std::string, std::vector<std::string>>>;

  std::filesystem::path home;
  std::filesystem::path map_root;
  std::filesystem::path recording_root = "/";
  std::filesystem::path selection_path;
  std::function<double()> clock;
  std::function<TopicGraph()> topics;
  std::function<bool(const std::string &)> managed_running;
  std::function<void(const MapSelection &, bool)> map_selected;
  std::function<bool()> save_map_available;
  std::function<void(const SaveMapRequest &, SaveMapCompletion)> save_map;
};

class Catalog
{
public:
  explicit Catalog(CatalogConfig config = {});
  ~Catalog();

  Catalog(const Catalog &) = delete;
  Catalog & operator=(const Catalog &) = delete;

  std::optional<CatalogResponse> route(
    const std::string & method, const std::string & path, const std::string & browse_path,
    const nlohmann::json & payload);

  MapSelection selection() const;
  std::string navigation_planner() const;
  bool topic_logging_enabled() const;
  bool set_topic_logging_enabled(const nlohmann::json & payload);
  nlohmann::json recording_status() const;
  nlohmann::json save_map_status() const;

private:
  class Implementation;
  std::unique_ptr<Implementation> implementation_;
};
}
