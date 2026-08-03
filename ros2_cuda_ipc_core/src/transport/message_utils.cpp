// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/transport/message_utils.hpp"

#include <cuda.h>

#include <cstring>

namespace ros2_cuda_ipc_core::transport {

namespace {

using BufferCoreMessage = ros2_cuda_ipc_msgs::msg::BufferCore;

static_assert(sizeof(CUipcEventHandle) == EventHandlePayload{}.size(),
              "CUDA IPC event handle payload size changed");
static_assert(sizeof(BufferCoreMessage::_event_handle_type) ==
                  EventHandlePayload{}.size(),
              "BufferCore.event_handle must remain a 64-byte payload");

}  // namespace

void fill_buffer_core_message(const BlockDescriptor& descriptor,
                              ros2_cuda_ipc_msgs::msg::BufferCore& message) {
  message.publisher_pid = descriptor.publisher_pid;
  message.block_id = descriptor.block_id;
  message.uid = descriptor.uid;
  message.device_id = static_cast<uint32_t>(descriptor.device_id);
  message.byte_size = descriptor.byte_size;
  message.vmm_socket_path = descriptor.vmm_socket_path;
  std::memcpy(message.event_handle.data(), &descriptor.ready_event_handle,
              sizeof(message.event_handle));
}

}  // namespace ros2_cuda_ipc_core::transport
