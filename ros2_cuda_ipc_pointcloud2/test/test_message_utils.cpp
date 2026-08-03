#include <gtest/gtest.h>

#include <cstring>

#include "ros2_cuda_ipc_core/transport/buffer_descriptor.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_point_cloud2.hpp"
#include "ros2_cuda_ipc_pointcloud2/message_utils.hpp"
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

TEST(PointCloud2MessageUtilsTest,
     ConstructsGpuPointCloud2FromDescriptorAndMetadata) {
  const auto descriptor = make_descriptor();
  ros2_cuda_ipc_msgs::msg::GpuPointCloud2 message;
  ros2_cuda_ipc_pointcloud2::PointCloud2ReadHandle metadata;

  metadata.header.frame_id = "lidar";
  metadata.height = 1;
  metadata.width = 10;
  metadata.point_step = 12;
  metadata.row_step = 120;
  metadata.is_dense = true;
  ros2_cuda_ipc_pointcloud2::PointCloud2ReadHandle::Field field;
  field.name = "x";
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  metadata.fields = {field};
  ros2_cuda_ipc_pointcloud2::fill_gpu_pointcloud2_message(descriptor, metadata,
                                                          message);

  EXPECT_EQ(message.header.frame_id, "lidar");
  EXPECT_EQ(message.width, 10u);
  ASSERT_EQ(message.fields.size(), 1u);
  EXPECT_EQ(message.fields[0].name, "x");
  EXPECT_EQ(message.core.block_id, descriptor.block_id);
}

}  // namespace
