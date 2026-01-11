#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace ros2_cuda_ipc_core {

/// Maximum payload size for the mem_handle field in BufferCore.
constexpr std::size_t kMemoryHandleSize = 64;

/// Identifier for the memory backend embedded in BufferCore.backend.
enum class MemoryBackendKind : uint8_t {
  CUDA_IPC = 0,
  VMM_FD = 1,
};

inline constexpr uint8_t to_backend_byte(MemoryBackendKind kind) noexcept {
  return static_cast<uint8_t>(kind);
}

inline constexpr MemoryBackendKind backend_from_byte(uint8_t value) noexcept {
  switch (value) {
    case static_cast<uint8_t>(MemoryBackendKind::CUDA_IPC):
      return MemoryBackendKind::CUDA_IPC;
    case static_cast<uint8_t>(MemoryBackendKind::VMM_FD):
      return MemoryBackendKind::VMM_FD;
    default:
      return MemoryBackendKind::CUDA_IPC;
  }
}

inline bool is_safe_uuid_char(char c) noexcept {
  // Allow only alphanumeric characters and hyphens to avoid path traversal.
  return (c >= '0' && c <= '9') ||
         (c >= 'A' && c <= 'Z') ||
         (c >= 'a' && c <= 'z') ||
         (c == '-');
}

inline std::string build_memory_socket_path(std::string_view uuid) {
  std::string path = "/tmp/cuda_memory_pool_";
  for (char c : uuid) {
    if (is_safe_uuid_char(c)) {
      path.push_back(c);
    }
  }
  path.append(".sock");
  return path;
}

using MemoryHandlePayload = std::array<uint8_t, kMemoryHandleSize>;

}  // namespace ros2_cuda_ipc_core
