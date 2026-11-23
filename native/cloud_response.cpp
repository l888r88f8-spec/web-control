#include "cloud_response.hpp"

#include <cstring>
#include <limits>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace web_control
{
namespace
{
void append_word(std::string & output, std::uint32_t value)
{
  for (unsigned int shift = 0; shift < 32; shift += 8) {
    output.push_back(static_cast<char>((value >> shift) & 0xff));
  }
}

void append_floats(std::string & output, const std::vector<float> & values)
{
  static_assert(sizeof(float) == sizeof(std::uint32_t));
  static_assert(std::numeric_limits<float>::is_iec559);
  for (const float value : values) {
    std::uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    append_word(output, bits);
  }
}
}  // namespace

std::string cloud_response(const nlohmann::json & metadata, const std::vector<float> & positions,
  const std::vector<float> * colors)
{
  if (positions.size() % 3 != 0 || (colors && colors->size() != positions.size())) {
    throw std::invalid_argument("cloud coordinates and colors must contain matching XYZ/RGB triples");
  }
  const auto header = metadata.dump(-1, ' ', false, nlohmann::json::error_handler_t::replace);
  if (header.size() > std::numeric_limits<std::uint32_t>::max()) {
    throw std::length_error("terrain response header is too large");
  }
  std::string response = "WCB1";
  response.reserve(8 + header.size() + (positions.size() + (colors ? colors->size() : 0)) * sizeof(float));
  append_word(response, static_cast<std::uint32_t>(header.size()));
  response.append(header);
  append_floats(response, positions);
  if (colors) {
    append_floats(response, *colors);
  }
  return response;
}
}  // namespace web_control
