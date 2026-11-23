#include "system_status.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace web_control
{
namespace
{
double round_hundredths(double value)
{
  return std::round(value * 100) / 100;
}
}

nlohmann::json SystemStatus::snapshot(double now)
{
  using nlohmann::json;
  std::lock_guard<std::mutex> lock(mutex_);
  std::ifstream cpu_stream("/proc/stat");
  std::map<std::string, Cpu> current;
  json cores = json::array();
  json cpu_total = nullptr;
  for (std::string line; std::getline(cpu_stream, line); ) {
    std::istringstream input(line);
    std::string name;
    input >> name;
    if (name.compare(0, 3, "cpu") != 0) {
      break;
    }
    Cpu ticks{};
    std::size_t field = 0;
    for (std::uint64_t counter; input >> counter; ++field) {
      ticks.total += counter;
      if (field == 3 || field == 4) {
        ticks.idle += counter;
      }
    }
    if (field == 0) {
      continue;
    }
    current.emplace(name, ticks);
    json percent = nullptr;
    const auto previous = previous_.find(name);
    if (previous != previous_.end() && ticks.total > previous->second.total &&
      ticks.idle >= previous->second.idle)
    {
      const auto total = ticks.total - previous->second.total;
      const auto idle = std::min(total, ticks.idle - previous->second.idle);
      percent = round_hundredths(100.0 * (total - idle) / total);
    }
    if (name == "cpu") {
      cpu_total = percent;
    } else {
      cores.push_back({{"id", std::stoi(name.substr(3))}, {"percent", percent}});
    }
  }
  if (!current.count("cpu")) {
    throw std::runtime_error("missing aggregate CPU statistics");
  }
  previous_ = std::move(current);

  std::ifstream memory_stream("/proc/meminfo");
  std::map<std::string, std::uint64_t> memory;
  for (std::string line; std::getline(memory_stream, line); ) {
    std::istringstream input(line);
    std::string name;
    std::uint64_t value;
    if (input >> name >> value && !name.empty() && name.back() == ':') {
      name.pop_back();
      memory[name] = value;
    }
  }
  const auto used = [](auto total, auto free) {
      const auto consumed = total - std::min(total, free);
      return json{{"total_kb", total}, {"used_kb", consumed},
        {"percent", total ? round_hundredths(100.0 * consumed / total) : 0.0}};
    };
  return {{"stamp", now}, {"cpu", {{"total_percent", cpu_total}, {"cores", std::move(cores)}}},
    {"memory", used(memory["MemTotal"], memory.count("MemAvailable") ?
        memory["MemAvailable"] : memory["MemFree"])},
    {"swap", used(memory["SwapTotal"], memory["SwapFree"])}};
}
}  // namespace web_control
