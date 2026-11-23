#include "catalog.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace fs = std::filesystem;
using web_control::Catalog;
using web_control::CatalogConfig;
using nlohmann::json;

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

struct TestDirectory
{
  TestDirectory()
  {
    std::string pattern = (fs::temp_directory_path() / "web-control-catalog-XXXXXX").string();
    const auto created = mkdtemp(pattern.data());
    require(created != nullptr, "could not create test directory");
    path = created;
  }

  ~TestDirectory()
  {
    std::error_code error;
    fs::remove_all(path, error);
  }

  fs::path path;
};
}

int main()
{
  TestDirectory temp;
  const auto root = temp.path / "map";
  const auto map = root / "site";
  fs::create_directories(map / "tiled");
  std::ofstream(map / "GlobalMap.pcd") << "global";
  std::ofstream(map / "GroundMap.pcd") << "ground";
  std::ofstream(map / "tiled" / "manifest.yaml") <<
    "schema_version: 1\ntiles: {global: [], ground: []}\n";
  fs::create_directory_symlink(temp.path, root / "escape");
  const auto settings = temp.path / "selected_map.json";
  std::ofstream(settings) << R"({"other":42,"ui":{"joystick":{"size":200},"unknown":true}})";

  CatalogConfig config;
  config.home = temp.path;
  config.map_root = root;
  config.recording_root = root;
  config.selection_path = settings;
  config.clock = [] {return 5.0;};
  bool save_available = true;
  web_control::CatalogConfig::SaveMapCompletion complete;
  config.save_map_available = [&] {return save_available;};
  config.save_map = [&](const web_control::SaveMapRequest & request, auto callback) {
      require(request.destination == (root / "new_map").string(), "save destination was rewritten");
      require(request.resolution == 0.1 && request.ground_resolution == -1.0,
        "save resolutions were not forwarded");
      require(request.save_ground_map && !request.save_tiled_map, "save toggles were not forwarded");
      complete = std::move(callback);
    };
  config.topics = [] {
      return CatalogConfig::TopicGraph{{"/pose", {"geometry_msgs/msg/PoseStamped"}}};
    };
  Catalog catalog(std::move(config));
  const auto get = [&](const std::string & path, const std::string & browse = "") {
    return catalog.route("GET", path, browse, nullptr).value();
  };
  const auto post = [&](const std::string & path, const json & value) {
    return catalog.route("POST", path, "", value).value();
  };

  const auto maps = get("/api/maps").body.at("maps");
  require(maps.size() == 1 && maps[0].at("tiled") == true, "tiled map not listed");
  require(!maps[0].contains("global_map_path"), "map artifact path was exposed");
  const auto preferences = get("/api/ui/preferences").body;
  require(preferences.at("joystick").at("throttle").at("size") == 200, "legacy size lost");
  require(preferences.at("joystick").at("steering").at("x").is_null(), "legacy x leaked");

  require(post("/api/maps/select", {{"name", "site"}}).status == 200, "map selection rejected");
  require(catalog.selection().effective_mode == "tiled", "selected map mode not resolved");
  require(post("/api/maps/select", {{"auto_relocalization", true}}).status == 400,
    "auto-relocalization enabled without a database");
  require(post("/api/ui/preferences", {{"layers", {{"global_map", false}}}}).status == 200,
    "layer update rejected");
  const auto persisted = json::parse(std::ifstream(settings));
  require(persisted.at("name") == "site" && persisted.at("other") == 42, "selection data lost");
  require(persisted.at("ui").at("unknown") == true, "unknown UI preference lost");

  const auto browse_route = "/api/mapping/record/browse";
  require(get(browse_route).body.at("entries").size() == 1, "escaping symlink was listed");
  require(get(browse_route, (root / "escape").string()).status == 400, "escaping browse accepted");
  require(get(browse_route, (map / "..").string()).status == 400, "parent traversal accepted");

  const auto save_route = "/api/mapping/save_map";
  const auto save = json{
    {"destination", (root / "new_map").string()}, {"resolution", "0.1"}, {"save_tiled_map", false}};
  require(post(save_route, save).body.at("state") == "running", "save did not start");
  require(post(save_route, save).status == 409, "second save accepted during export");
  auto old_completion = complete;
  save_available = false;
  require(get(save_route).body.at("state") == "failed", "missing service left save running");
  save_available = true;
  require(post(save_route, save).status == 200, "could not retry failed save");
  old_completion(true, "stale result");
  require(get(save_route).body.at("state") == "running", "stale save callback modified new job");
  complete(true, "saved");
  require(get(save_route).body.at("success") == true, "completed save did not report success");
  require(post(save_route, {{"destination", (root / "escape" / "new_map").string()}}).status == 400,
    "escaping save destination accepted");

  const auto record_route = "/api/mapping/record/start";
  require(post(record_route, {
    {"directory", root.string()}, {"name", "bag"}, {"topics", {"/pose", "/pose"}}}).status == 400,
    "duplicate recording topic accepted");
  require(post(record_route, {
    {"directory", root.string()}, {"name", "site"}, {"topics", {"/pose"}}}).status == 409,
    "existing recording destination accepted");
}
