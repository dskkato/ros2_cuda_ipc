// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cstring>

#include "ros2_cuda_ipc_core/image/image_read_handle.hpp"
#include "ros2_cuda_ipc_core/image/message_utils.hpp"
#include "ros2_cuda_ipc_core/pointcloud2/message_utils.hpp"
#include "ros2_cuda_ipc_core/pointcloud2/pointcloud2_read_handle.hpp"
#include "ros2_cuda_ipc_core/transport/buffer_descriptor.hpp"
#include "ros2_cuda_ipc_core/transport/message_utils.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_image.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

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

TEST(PublisherMessagesTest,
     ConstructsGpuImageFromDescriptorAndApplicationMetadata) {
  const auto descriptor = make_descriptor();
  ros2_cuda_ipc_msgs::msg::GpuImage message;
  ros2_cuda_ipc_core::image::ImageReadHandle metadata;

  metadata.header.frame_id = "camera";
  metadata.dtype = ros2_cuda_ipc_core::image::DType::U8;
  metadata.shape = {1, 10, 3};
  metadata.strides = {30, 3, 1};
  metadata.encoding = "rgb8";
  ros2_cuda_ipc_core::image::fill_gpu_image_message(descriptor, metadata,
                                                    message);

  EXPECT_EQ(message.header.frame_id, "camera");
  EXPECT_EQ(message.shape[1], 10u);
  EXPECT_EQ(message.strides[0], 30u);
  EXPECT_EQ(message.encoding, "rgb8");
  EXPECT_EQ(message.core.publisher_pid, descriptor.publisher_pid);
}

TEST(PublisherMessagesTest,
     ConstructsGpuPointCloud2FromDescriptorAndApplicationMetadata) {
  const auto descriptor = make_descriptor();
  ros2_cuda_ipc_msgs::msg::GpuPointCloud2 message;
  ros2_cuda_ipc_core::pointcloud2::PointCloud2ReadHandle metadata;

  metadata.header.frame_id = "lidar";
  metadata.height = 1;
  metadata.width = 10;
  metadata.point_step = 12;
  metadata.row_step = 120;
  metadata.is_dense = true;
  ros2_cuda_ipc_core::pointcloud2::PointCloud2ReadHandle::Field field;
  field.name = "x";
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  metadata.fields = {field};
  ros2_cuda_ipc_core::pointcloud2::fill_gpu_pointcloud2_message(
      descriptor, metadata, message);

  EXPECT_EQ(message.header.frame_id, "lidar");
  EXPECT_EQ(message.width, 10u);
  ASSERT_EQ(message.fields.size(), 1u);
  EXPECT_EQ(message.fields[0].name, "x");
  EXPECT_EQ(message.core.block_id, descriptor.block_id);
}

}  // namespace
