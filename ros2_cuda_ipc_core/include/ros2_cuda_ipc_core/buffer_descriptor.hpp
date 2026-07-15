// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda_runtime_api.h>

#include <cstdint>
#include <string>

#include "ros2_cuda_ipc_core/memory_types.hpp"
#include "ros2_cuda_ipc_msgs/msg/buffer_core.hpp"

namespace ros2_cuda_ipc_core {

// Transport-facing description of a publisher-owned GPU buffer.  This value
// does not own GPU resources and does not acquire or release a lease.
struct BufferDescriptor {
  std::string lease_shm_name;
  uint32_t slot_id = 0;
  uint32_t generation = 0;
  int device_id = -1;
  uint64_t byte_size = 0;
  MemoryBackendKind backend = MemoryBackendKind::CUDA_IPC;
  MemoryHandlePayload memory_handle{};
  cudaIpcEventHandle_t ready_event_handle{};
};

// Copies transport metadata into the ROS wire message.  Publisher-local CUDA
// pointers and leases are intentionally not part of BufferCore.
void fill_buffer_core_message(const BufferDescriptor& descriptor,
                              ros2_cuda_ipc_msgs::msg::BufferCore& message);

}  // namespace ros2_cuda_ipc_core
