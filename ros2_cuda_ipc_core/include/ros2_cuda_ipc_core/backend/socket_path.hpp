// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <uuid/uuid.h>

#include <stdexcept>
#include <string>
#include <string_view>

namespace ros2_cuda_ipc_core::backend {

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

}  // namespace ros2_cuda_ipc_core::backend
