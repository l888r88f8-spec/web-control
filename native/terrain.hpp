#pragma once

#include <string>
#include <vector>

#include <visualization_msgs/msg/marker_array.hpp>

namespace web_control
{
struct TerrainMesh
{
  std::string frame_id;
  std::vector<float> positions;
  std::vector<float> colors;
};

TerrainMesh convert_terrain(const visualization_msgs::msg::MarkerArray & message);
}
