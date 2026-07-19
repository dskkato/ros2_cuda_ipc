// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

namespace ros2_cuda_ipc_core {

/// Protocol identity of one publisher initialisation.
using PublisherInstanceId = std::array<uint8_t, 16>;

inline bool is_nil(const PublisherInstanceId& id) noexcept {
  return std::all_of(id.begin(), id.end(),
                     [](uint8_t byte) { return byte == 0; });
}

}  // namespace ros2_cuda_ipc_core
