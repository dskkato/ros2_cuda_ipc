// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/buffer_descriptor.hpp"

namespace ros2_cuda_ipc_core {

view::BufferView BufferDescriptor::to_publisher_view(void* device_ptr) const {
  view::BufferView result;
  result.dev_ptr = device_ptr;
  result.device_id = device_id;
  result.byte_size = byte_size;
  result.slot_id = slot_id;
  result.generation = generation;
  result.shm_name = lease_shm_name;
  result.set_ipc_handles(backend, memory_handle.data(), memory_handle.size(),
                         ready_event_handle);
  return result;
}

}  // namespace ros2_cuda_ipc_core
