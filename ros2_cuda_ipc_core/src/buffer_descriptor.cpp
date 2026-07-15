// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/buffer_descriptor.hpp"

#include <cstring>

namespace ros2_cuda_ipc_core {

void fill_buffer_core_message(const BufferDescriptor& descriptor,
                              ros2_cuda_ipc_msgs::msg::BufferCore& message) {
  message.shm_name = descriptor.lease_shm_name;
  message.device_id = static_cast<uint32_t>(descriptor.device_id);
  message.slot_id = descriptor.slot_id;
  message.generation = descriptor.generation;
  message.byte_size = descriptor.byte_size;
  message.backend = to_backend_byte(descriptor.backend);
  std::memcpy(message.mem_handle.data(), descriptor.memory_handle.data(),
              descriptor.memory_handle.size());
  std::memcpy(message.event_handle.data(), &descriptor.ready_event_handle,
              sizeof(descriptor.ready_event_handle));
}

}  // namespace ros2_cuda_ipc_core
