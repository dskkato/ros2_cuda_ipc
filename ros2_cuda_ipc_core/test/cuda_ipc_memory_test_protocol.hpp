// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <type_traits>

#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::test {

struct CudaIpcMemoryTestPayload {
  uint32_t device_id = 0;
  uint64_t byte_size = 0;
  uint8_t expected_value = 0;
  transport::MemoryHandlePayload memory_handle{};
  transport::EventHandlePayload event_handle{};
};

static_assert(std::is_trivially_copyable_v<CudaIpcMemoryTestPayload>);

}  // namespace ros2_cuda_ipc_core::test
