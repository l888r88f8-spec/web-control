#pragma once

#include <cstdint>
#include <string>

#include <nlohmann/json.hpp>

#include <vector>

namespace web_control
{
std::string cloud_response(const nlohmann::json & header, const std::vector<float> & positions,
  const std::vector<float> * colors);

}
