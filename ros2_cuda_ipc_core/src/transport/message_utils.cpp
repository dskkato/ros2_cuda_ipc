// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/transport/message_utils.hpp"

#include <cuda.h>

#include <cstring>

namespace ros2_cuda_ipc_core::transport {

namespace {

using BufferCoreMessage = ros2_cuda_ipc_msgs::msg::BufferCore;

static_assert(sizeof(BufferCoreMessage::_mem_handle_type) ==
                  sizeof(MemoryHandlePayload),
              "BufferCore.mem_handle must match MemoryHandlePayload");
static_assert(sizeof(CUipcEventHandle) == EventHandlePayload{}.size(),
              "CUDA IPC event handle payload size changed");
static_assert(sizeof(BufferCoreMessage::_event_handle_type) ==
                  EventHandlePayload{}.size(),
              "BufferCore.event_handle must remain a 64-byte payload");

}  // namespace

void fill_buffer_core_message(const BufferDescriptor& descriptor,
                              ros2_cuda_ipc_msgs::msg::BufferCore& message) {
  message.shm_name = descriptor.buffer_metadata_shm_name;
  message.publisher_instance_id = descriptor.publisher_instance_id;
  message.device_id = static_cast<uint32_t>(descriptor.device_id);
  message.slot_id = descriptor.slot_id;
  message.generation = descriptor.generation;
  message.byte_size = descriptor.byte_size;
  std::memcpy(message.mem_handle.data(), descriptor.memory_handle.data(),
              sizeof(message.mem_handle));
  std::memcpy(message.event_handle.data(), &descriptor.ready_event_handle,
              sizeof(message.event_handle));
}

}  // namespace ros2_cuda_ipc_core::transport
