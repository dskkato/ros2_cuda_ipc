// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cstring>

#include "ros2_cuda_ipc_core/buffer_descriptor.hpp"
#include "ros2_cuda_ipc_core/view/image_view.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_image.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace {

ros2_cuda_ipc_core::BufferDescriptor make_descriptor() {
  ros2_cuda_ipc_core::BufferDescriptor descriptor;
  descriptor.lease_shm_name = "/publisher_messages";
  descriptor.slot_id = 7;
  descriptor.generation = 11;
  descriptor.device_id = 2;
  descriptor.byte_size = 120;
  descriptor.backend = ros2_cuda_ipc_core::MemoryBackendKind::CUDA_IPC;
  std::memset(descriptor.memory_handle.data(), 0x12,
              descriptor.memory_handle.size());
  std::memset(&descriptor.ready_event_handle, 0x34,
              sizeof(descriptor.ready_event_handle));
  return descriptor;
}

TEST(PublisherMessagesTest,
     DescriptorConvertsToBufferCoreWithoutDevicePointer) {
  const auto descriptor = make_descriptor();
  ros2_cuda_ipc_msgs::msg::BufferCore message;

  ros2_cuda_ipc_core::fill_buffer_core_message(descriptor, message);

  EXPECT_EQ(message.shm_name, descriptor.lease_shm_name);
  EXPECT_EQ(message.slot_id, descriptor.slot_id);
  EXPECT_EQ(message.generation, descriptor.generation);
  EXPECT_EQ(message.device_id, 2u);
  EXPECT_EQ(message.byte_size, descriptor.byte_size);
  EXPECT_EQ(message.backend, ros2_cuda_ipc_msgs::msg::BufferCore::CUDA_IPC);
  EXPECT_EQ(message.mem_handle[0], 0x12);
  EXPECT_EQ(message.event_handle[0], 0x34);
}

TEST(PublisherMessagesTest,
     ConstructsGpuImageFromDescriptorAndApplicationMetadata) {
  const auto descriptor = make_descriptor();
  ros2_cuda_ipc_msgs::msg::GpuImage message;

  ros2_cuda_ipc_core::fill_buffer_core_message(descriptor, message.core);
  message.header.frame_id = "camera";
  message.dtype = static_cast<uint8_t>(ros2_cuda_ipc_core::view::DType::U8);
  message.shape = {1, 10, 3};
  message.strides = {30, 3, 1};
  message.encoding = "rgb8";

  EXPECT_EQ(message.header.frame_id, "camera");
  EXPECT_EQ(message.shape[1], 10u);
  EXPECT_EQ(message.strides[0], 30u);
  EXPECT_EQ(message.encoding, "rgb8");
  EXPECT_EQ(message.core.shm_name, descriptor.lease_shm_name);
}

TEST(PublisherMessagesTest,
     ConstructsGpuPointCloud2FromDescriptorAndApplicationMetadata) {
  const auto descriptor = make_descriptor();
  ros2_cuda_ipc_msgs::msg::GpuPointCloud2 message;

  ros2_cuda_ipc_core::fill_buffer_core_message(descriptor, message.core);
  message.header.frame_id = "lidar";
  message.height = 1;
  message.width = 10;
  message.point_step = 12;
  message.row_step = 120;
  message.is_dense = true;
  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  message.fields = {field};

  EXPECT_EQ(message.header.frame_id, "lidar");
  EXPECT_EQ(message.width, 10u);
  ASSERT_EQ(message.fields.size(), 1u);
  EXPECT_EQ(message.fields[0].name, "x");
  EXPECT_EQ(message.core.slot_id, descriptor.slot_id);
}

}  // namespace
