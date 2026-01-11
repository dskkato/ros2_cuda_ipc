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

inline std::string build_memory_socket_path(std::string_view uuid) {
  std::string path = "/tmp/cuda_memory_pool_";
  path.append(uuid.begin(), uuid.end());
  path.append(".sock");
  return path;
}

using MemoryHandlePayload = std::array<uint8_t, kMemoryHandleSize>;

}  // namespace ros2_cuda_ipc_core
