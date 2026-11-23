#include <limits>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "cloud_response.hpp"
#include "terrain.hpp"

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

std::uint32_t word(const std::string & bytes, std::size_t offset)
{
  std::uint32_t value = 0;
  for (unsigned int i = 0; i < 4; ++i) {
    value |= static_cast<std::uint32_t>(static_cast<unsigned char>(bytes.at(offset + i))) << (i * 8);
  }
  return value;
}
}

int main()
{
  using visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray message;
  Marker faces;
  faces.header.frame_id = "map";
  faces.ns = "terrain_cells";
  faces.type = Marker::TRIANGLE_LIST;
  faces.points.resize(6);
  faces.points[1].x = 1;
  faces.points[2].y = 2;
  faces.points[5].x = std::numeric_limits<double>::infinity();
  faces.colors.resize(1);
  faces.colors.front().r = 1;
  faces.color.g = 1;

  Marker edges = faces;
  edges.ns = "terrain_grid_edges";
  edges.type = Marker::LINE_LIST;
  message.markers = {edges, faces};
  auto mesh = web_control::convert_terrain(message);
  require(mesh.frame_id == "map", "terrain frame was lost");
  require(mesh.positions == std::vector<float>{0, 0, 0, 1, 0, 0, 0, 2, 0}, "wrong face vertices");
  require(mesh.colors == std::vector<float>{1, 0, 0, 0, 1, 0, 0, 1, 0}, "wrong face colors");

  const auto body = web_control::cloud_response(
    {{"count", 3}, {"version", 3}, {"format", "triangles"}, {"epoch", "test-epoch"}},
    mesh.positions, &mesh.colors);
  require(body.substr(0, 4) == "WCB1", "wrong cloud binary magic");
  const auto header_size = word(body, 4);
  const auto header = nlohmann::json::parse(body.substr(8, header_size));
  const auto data_offset = 8 + header_size;
  require(header.at("count") == 3 && header.at("version") == 3 &&
    header.at("format") == "triangles" && header.at("epoch") == "test-epoch",
    "wrong terrain metadata");
  require(body.size() == data_offset + 72, "wrong cloud binary array length");
  require(word(body, data_offset + 12) == 0x3f800000, "wrong little-endian position");
  require(word(body, data_offset + 36) == 0x3f800000, "wrong little-endian color");

  Marker clear;
  clear.action = Marker::DELETEALL;
  message.markers.push_back(clear);
  mesh = web_control::convert_terrain(message);
  require(mesh.positions.empty() && mesh.colors.empty(), "DELETEALL did not clear the terrain");
}
