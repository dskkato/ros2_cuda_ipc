// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <string>

#include "ros2_cuda_ipc_core/publisher_instance_id.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"
#include "ros2_cuda_ipc_msgs/msg/buffer_core.hpp"

namespace ros2_cuda_ipc_core::transport {

// Transport-facing description of a publisher-owned GPU buffer.  This value
// does not own GPU resources and does not acquire or release a BufferRef.
struct BufferDescriptor {
  std::string buffer_metadata_shm_name;
  PublisherInstanceId publisher_instance_id{};
  uint32_t slot_id = 0;
  uint32_t generation = 0;
  int device_id = -1;
  uint64_t byte_size = 0;
  std::string vmm_socket_path;
  EventHandlePayload ready_event_handle{};
};

}  // namespace ros2_cuda_ipc_core::transport
