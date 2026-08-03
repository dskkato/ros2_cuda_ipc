#include <gtest/gtest.h>

#include <cstring>

#include "ros2_cuda_ipc_core/transport/buffer_descriptor.hpp"
#include "ros2_cuda_ipc_image/message_utils.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_image.hpp"

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

TEST(ImageMessageUtilsTest, ConstructsGpuImageFromDescriptorAndMetadata) {
  const auto descriptor = make_descriptor();
  ros2_cuda_ipc_msgs::msg::GpuImage message;
  ros2_cuda_ipc_image::ImageReadHandle metadata;

  metadata.header.frame_id = "camera";
  metadata.dtype = ros2_cuda_ipc_image::DType::U8;
  metadata.shape = {1, 10, 3};
  metadata.strides = {30, 3, 1};
  metadata.encoding = "rgb8";
  ros2_cuda_ipc_image::fill_gpu_image_message(descriptor, metadata, message);

  EXPECT_EQ(message.header.frame_id, "camera");
  EXPECT_EQ(message.shape[1], 10u);
  EXPECT_EQ(message.strides[0], 30u);
  EXPECT_EQ(message.encoding, "rgb8");
  EXPECT_EQ(message.core.publisher_pid, descriptor.publisher_pid);
}

}  // namespace
