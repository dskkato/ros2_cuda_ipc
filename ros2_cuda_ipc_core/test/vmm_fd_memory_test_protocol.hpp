// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::test {

constexpr std::size_t kVmmSocketPathStorageSize = 64;

// This is intentionally a fixed-size raw test transport buffer, not a mirror
// of BufferCore.vmm_socket_path.  The ROS message uses string, while this
// payload is sent as a trivially copyable struct over a pipe to test the
// process boundary without involving ROS message transport.
struct VmmFdMemoryTestPayload {
  uint32_t device_id = 0;
  uint64_t byte_size = 0;
  uint8_t expected_value = 0;
  std::array<char, kVmmSocketPathStorageSize> vmm_socket_path{};
  transport::EventHandlePayload event_handle{};
};

static_assert(std::is_trivially_copyable_v<VmmFdMemoryTestPayload>);

}  // namespace ros2_cuda_ipc_core::test
