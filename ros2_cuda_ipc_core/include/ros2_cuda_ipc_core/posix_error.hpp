#pragma once

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>

namespace ros2_cuda_ipc_core {

inline std::string errno_to_string(int err) noexcept {
  char buffer[128];
#if defined(__GLIBC__) && defined(_GNU_SOURCE)
  char *msg = ::strerror_r(err, buffer, sizeof(buffer));
  if (msg != nullptr) {
    return std::string(msg);
  }
#else
  if (::strerror_r(err, buffer, sizeof(buffer)) == 0) {
    return std::string(buffer);
  }
#endif
  std::snprintf(buffer, sizeof(buffer), "Unknown error %d", err);
  return std::string(buffer);
}

inline std::string errno_to_string() noexcept { return errno_to_string(errno); }

}  // namespace ros2_cuda_ipc_core
