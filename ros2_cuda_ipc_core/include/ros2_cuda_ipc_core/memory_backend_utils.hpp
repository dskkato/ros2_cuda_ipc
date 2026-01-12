#pragma once

#include <algorithm>
#include <cctype>
#include <string>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/memory_types.hpp"

namespace ros2_cuda_ipc_core {

inline MemoryBackendKind parse_memory_backend(std::string name,
                                              const rclcpp::Logger &logger) {
  std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (name == "cuda" || name == "cuda_ipc" || name == "ipc") {
    return MemoryBackendKind::CUDA_IPC;
  }
  if (name == "vmm_fd" || name == "vmm-fd" || name == "vmm" || name == "fd") {
    return MemoryBackendKind::VMM_FD;
  }

  RCLCPP_WARN(logger, "Unknown memory_backend='%s'; defaulting to CUDA IPC",
              name.c_str());
  return MemoryBackendKind::CUDA_IPC;
}

}  // namespace ros2_cuda_ipc_core
