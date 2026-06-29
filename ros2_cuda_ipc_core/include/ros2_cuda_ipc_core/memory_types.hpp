// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <uuid/uuid.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>

namespace ros2_cuda_ipc_core {

/// Maximum payload size for the mem_handle field in BufferCore.
constexpr std::size_t kMemoryHandleSize = 64;
constexpr std::size_t kVmmUuidMaxLength = 36;

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
  // Validate UUID format
  uuid_t tmp;
  if (uuid_parse_range(uuid.begin(), uuid.end(), tmp) != 0) {
    throw std::invalid_argument("Invalid UUID format");
  }

  std::string path = "/tmp/cuda_memory_pool_";
  path.append(uuid.begin(), uuid.end());
  path.append(".sock");
  return path;
}

using MemoryHandlePayload = std::array<uint8_t, kMemoryHandleSize>;

inline uint32_t load_u32_le(const uint8_t* data) {
  uint32_t value = 0;
  for (int i = 3; i >= 0; --i) {
    value = (value << 8) | data[i];
  }
  return value;
}

inline void store_u32_le(uint8_t* dest, uint32_t value) {
  for (int i = 0; i < 4; ++i) {
    dest[i] = static_cast<uint8_t>(value & 0xFF);
    value >>= 8;
  }
}

inline bool encode_uuid_payload(const std::string& uuid,
                                MemoryHandlePayload& payload) {
  if (uuid.size() > kVmmUuidMaxLength) {
    return false;
  }
  payload.fill(0);
  store_u32_le(payload.data(), static_cast<uint32_t>(uuid.size()));
  std::copy(uuid.begin(), uuid.end(), payload.begin() + 4);
  return true;
}

}  // namespace ros2_cuda_ipc_core
