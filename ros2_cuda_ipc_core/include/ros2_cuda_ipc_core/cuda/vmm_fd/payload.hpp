// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <uuid/uuid.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

#include "ros2_cuda_ipc_core/memory_types.hpp"

namespace ros2_cuda_ipc_core::cuda::vmm_fd {

constexpr std::size_t kUuidMaxLength = 36;

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

inline std::string build_socket_path(std::string_view uuid) {
  uuid_t tmp;
  if (uuid_parse_range(uuid.begin(), uuid.end(), tmp) != 0) {
    throw std::invalid_argument("Invalid UUID format");
  }

  std::string path = "/tmp/cuda_memory_pool_";
  path.append(uuid.begin(), uuid.end());
  path.append(".sock");
  return path;
}

inline bool encode_uuid_payload(const std::string& uuid,
                                MemoryHandlePayload& payload) {
  if (uuid.size() > kUuidMaxLength) {
    return false;
  }
  uuid_t tmp;
  if (uuid_parse_range(uuid.data(), uuid.data() + uuid.size(), tmp) != 0) {
    return false;
  }
  payload.fill(0);
  store_u32_le(payload.data(), static_cast<uint32_t>(uuid.size()));
  std::copy(uuid.begin(), uuid.end(), payload.begin() + 4);
  return true;
}

inline std::optional<std::string> decode_uuid_payload(
    const MemoryHandlePayload& payload) {
  const uint32_t length = load_u32_le(payload.data());
  if (length == 0 || length > kUuidMaxLength || length + 4 > payload.size()) {
    return std::nullopt;
  }

  uuid_t uuid;
  const char* uuid_str = reinterpret_cast<const char*>(payload.data() + 4);
  if (uuid_parse_range(uuid_str, uuid_str + length, uuid) != 0) {
    return std::nullopt;
  }

  return std::string(uuid_str, length);
}

}  // namespace ros2_cuda_ipc_core::cuda::vmm_fd
