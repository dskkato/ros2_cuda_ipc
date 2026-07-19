// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <string>

#include "ros2_cuda_ipc_core/publisher_instance_id.hpp"

namespace ros2_cuda_ipc_core::test {

inline PublisherInstanceId publisher_instance_id(const std::string& seed) {
  PublisherInstanceId id{};
  uint32_t state = 2166136261u;
  for (uint8_t byte : seed) {
    state = (state ^ byte) * 16777619u;
  }
  for (auto& byte : id) {
    state = state * 1664525u + 1013904223u;
    byte = static_cast<uint8_t>(state >> 24);
  }
  if (is_nil(id)) {
    id.back() = 1;
  }
  return id;
}

}  // namespace ros2_cuda_ipc_core::test
