#pragma once

#include <map>
#include <mutex>
#include <string>

#include <nlohmann/json.hpp>

namespace web_control
{
class SystemStatus
{
public:
  nlohmann::json snapshot(double now);

private:
  struct Cpu
  {
    std::uint64_t total;
    std::uint64_t idle;
  };
  std::mutex mutex_;
  std::map<std::string, Cpu> previous_;
};
}
