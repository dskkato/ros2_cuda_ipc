// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <cstdint>

namespace ros2_cuda_ipc_core::transport {

/// Fixed-size wire payload for a CUDA IPC event handle.
using EventHandlePayload = std::array<uint8_t, 64>;

}  // namespace ros2_cuda_ipc_core::transport
