// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <string>

#include "ros2_cuda_ipc_core/transport/memory_types.hpp"
#include "ros2_cuda_ipc_msgs/msg/buffer_core.hpp"

namespace ros2_cuda_ipc_core::transport {

// Transport-facing description of a publisher-owned GPU buffer.  This value
// does not own GPU resources and does not acquire or release a BufferRef.
struct BufferDescriptor {
  uint32_t publisher_pid = 0;
  uint32_t block_id = 0;
  uint64_t uid = 0;
  int device_id = -1;
  uint64_t byte_size = 0;
  std::string vmm_socket_path;
  EventHandlePayload ready_event_handle{};
};

}  // namespace ros2_cuda_ipc_core::transport
