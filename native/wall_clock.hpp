#pragma once

#include <chrono>

namespace web_control
{
inline double wall_time()
{
  return std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}
}
