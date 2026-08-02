// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace ros2_cuda_ipc_core::transport {

/// Maximum payload size for the mem_handle field in BufferCore.
constexpr std::size_t kMemoryHandleSize = 64;

using MemoryHandlePayload = std::array<uint8_t, kMemoryHandleSize>;

/// Fixed-size wire payload for a CUDA IPC event handle.
using EventHandlePayload = std::array<uint8_t, 64>;

}  // namespace ros2_cuda_ipc_core::transport
