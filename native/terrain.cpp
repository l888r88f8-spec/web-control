#include "terrain.hpp"

#include <cmath>
#include <limits>

namespace web_control
{
namespace
{
bool valid_coordinate(double value)
{
  return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
}

bool valid_position(const geometry_msgs::msg::Point & point)
{
  return valid_coordinate(point.x) && valid_coordinate(point.y) && valid_coordinate(point.z);
}
}  // namespace

TerrainMesh convert_terrain(const visualization_msgs::msg::MarkerArray & message)
{
  using visualization_msgs::msg::Marker;
  TerrainMesh mesh;
  for (const auto & marker : message.markers) {
    if (!marker.header.frame_id.empty()) {
      mesh.frame_id = marker.header.frame_id;
    }
    if (marker.action == Marker::DELETEALL) {
      mesh.positions.clear();
      mesh.colors.clear();
      continue;
    }
    if (marker.action != Marker::ADD || marker.type != Marker::TRIANGLE_LIST ||
      marker.ns != "terrain_cells")
    {
      continue;
    }
    for (std::size_t i = 0; i + 2 < marker.points.size(); i += 3) {
      if (!valid_position(marker.points[i]) || !valid_position(marker.points[i + 1]) ||
        !valid_position(marker.points[i + 2]))
      {
        continue;
      }
      for (std::size_t vertex = i; vertex < i + 3; ++vertex) {
        const auto & point = marker.points[vertex];
        const auto & color = vertex < marker.colors.size() ? marker.colors[vertex] : marker.color;
        mesh.positions.push_back(static_cast<float>(point.x));
        mesh.positions.push_back(static_cast<float>(point.y));
        mesh.positions.push_back(static_cast<float>(point.z));
        mesh.colors.push_back(color.r);
        mesh.colors.push_back(color.g);
        mesh.colors.push_back(color.b);
      }
    }
  }
  return mesh;
}
}  // namespace web_control
