// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/transport/message_utils.hpp"

#include <cstring>

namespace ros2_cuda_ipc_core::transport {

namespace {

using BufferCoreMessage = ros2_cuda_ipc_msgs::msg::BufferCore;

static_assert(sizeof(BufferCoreMessage::_mem_handle_type) ==
                  sizeof(MemoryHandlePayload),
              "BufferCore.mem_handle must match MemoryHandlePayload");
static_assert(sizeof(BufferCoreMessage::_event_handle_type) ==
                  sizeof(cudaIpcEventHandle_t),
              "BufferCore.event_handle must match cudaIpcEventHandle_t");

}  // namespace

void fill_buffer_core_message(const BufferDescriptor& descriptor,
                              ros2_cuda_ipc_msgs::msg::BufferCore& message) {
  message.shm_name = descriptor.lease_shm_name;
  message.device_id = static_cast<uint32_t>(descriptor.device_id);
  message.slot_id = descriptor.slot_id;
  message.generation = descriptor.generation;
  message.byte_size = descriptor.byte_size;
  message.backend = to_backend_byte(descriptor.backend);
  std::memcpy(message.mem_handle.data(), descriptor.memory_handle.data(),
              sizeof(message.mem_handle));
  std::memcpy(message.event_handle.data(), &descriptor.ready_event_handle,
              sizeof(message.event_handle));
}

}  // namespace ros2_cuda_ipc_core::transport
