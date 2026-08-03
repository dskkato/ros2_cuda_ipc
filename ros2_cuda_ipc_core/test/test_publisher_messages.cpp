// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cstring>

#include "ros2_cuda_ipc_core/transport/buffer_descriptor.hpp"
#include "ros2_cuda_ipc_core/transport/message_utils.hpp"

namespace {

ros2_cuda_ipc_core::transport::BufferDescriptor make_descriptor() {
  ros2_cuda_ipc_core::transport::BufferDescriptor descriptor;
  descriptor.publisher_pid = 4242;
  descriptor.block_id = 7;
  descriptor.uid = 11;
  descriptor.device_id = 2;
  descriptor.byte_size = 120;
  descriptor.vmm_socket_path =
      "/tmp/cuda_memory_pool_12345678-1234-5678-1234-567812345678.sock";
  std::memset(&descriptor.ready_event_handle, 0x34,
              sizeof(descriptor.ready_event_handle));
  return descriptor;
}

TEST(PublisherMessagesTest,
     DescriptorConvertsToBufferCoreWithoutDevicePointer) {
  const auto descriptor = make_descriptor();
  ros2_cuda_ipc_msgs::msg::BufferCore message;

  ros2_cuda_ipc_core::transport::fill_buffer_core_message(descriptor, message);

  EXPECT_EQ(message.publisher_pid, descriptor.publisher_pid);
  EXPECT_EQ(message.block_id, descriptor.block_id);
  EXPECT_EQ(message.uid, descriptor.uid);
  EXPECT_EQ(message.device_id, 2u);
  EXPECT_EQ(message.byte_size, descriptor.byte_size);
  EXPECT_EQ(message.vmm_socket_path,
            "/tmp/cuda_memory_pool_12345678-1234-5678-1234-567812345678.sock");
  EXPECT_EQ(message.event_handle[0], 0x34);
}

}  // namespace
