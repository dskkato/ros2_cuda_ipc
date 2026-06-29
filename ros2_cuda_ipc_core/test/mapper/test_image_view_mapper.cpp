// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <sys/mman.h>

#include "../test_mapper_utils.hpp"
#include "ros2_cuda_ipc_core/mapper/image_view_mapper.hpp"

namespace {

class ImageViewMapperTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    ros2_cuda_ipc_core::test::RclcppScope::SetUp();
  }
  static void TearDownTestSuite() {
    ros2_cuda_ipc_core::test::RclcppScope::TearDown();
  }
};

TEST_F(ImageViewMapperTest, InvalidCoreReturnsDefaultImageView) {
  const std::string shm_name =
      ros2_cuda_ipc_core::test::make_unique_shm_name("image_mapper_invalid");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));

  ros2_cuda_ipc_msgs::msg::GpuImage msg;
  msg.header.frame_id = "frame";
  msg.dtype = static_cast<uint8_t>(ros2_cuda_ipc_core::DType::U8);
  msg.shape = {4, 5, 3};
  msg.strides = {15, 3, 1};
  msg.encoding = "rgb8";
  msg.core.shm_name = shm_name;
  msg.core.slot_id = 0;
  msg.core.device_id = 0;
  msg.core.generation = 42;
  msg.core.byte_size = 60;
  msg.core.backend = ros2_cuda_ipc_msgs::msg::BufferCore::CUDA_IPC;

  ros2_cuda_ipc_core::mapper::ImageViewMapper mapper;
  auto view = mapper.map(msg);
  EXPECT_FALSE(view.valid());
  EXPECT_TRUE(view.header.frame_id.empty());

  ::shm_unlink(shm_name.c_str());
}

TEST_F(ImageViewMapperTest, CopiesMetadataWhenCoreIsValid) {
  auto core = ros2_cuda_ipc_core::test::make_seeded_buffer_core_message(
      "image_mapper_valid", 21);

  ros2_cuda_ipc_msgs::msg::GpuImage msg;
  msg.header.frame_id = "camera_frame";
  msg.dtype = static_cast<uint8_t>(ros2_cuda_ipc_core::DType::U16);
  msg.shape = {4, 5, 3};
  msg.strides = {30, 6, 2};
  msg.encoding = "mono16";
  msg.core = core;

  ros2_cuda_ipc_core::mapper::ImageViewMapper mapper;
  auto view = mapper.map(msg);
  ASSERT_TRUE(view.core.valid());
  EXPECT_EQ(view.header.frame_id, "camera_frame");
  EXPECT_EQ(view.dtype, ros2_cuda_ipc_core::DType::U16);
  EXPECT_EQ(view.shape[1], 5u);
  EXPECT_EQ(view.strides[0], 30u);
  EXPECT_EQ(view.encoding, "mono16");

  view.core.reset();
  ::shm_unlink(core.shm_name.c_str());
}

TEST_F(ImageViewMapperTest, FillGpuImageMessageCopiesMetadataAndCore) {
  ros2_cuda_ipc_core::ImageView view;
  view.header.frame_id = "frame";
  view.dtype = ros2_cuda_ipc_core::DType::U8;
  view.shape = {4, 5, 3};
  view.strides = {15, 3, 1};
  view.encoding = "rgb8";
  view.core.device_id = 1;
  view.core.byte_size = 60;
  view.core.slot_id = 3;
  view.core.generation = 7;
  view.core.shm_name = "/demo";
  cudaIpcMemHandle_t mem_handle{};
  cudaIpcEventHandle_t event_handle{};
  view.core.set_ipc_handles(ros2_cuda_ipc_core::MemoryBackendKind::CUDA_IPC,
                            reinterpret_cast<const uint8_t*>(&mem_handle),
                            sizeof(mem_handle), event_handle);

  ros2_cuda_ipc_msgs::msg::GpuImage msg;
  ros2_cuda_ipc_core::mapper::fill_gpu_image_message(view, msg);

  EXPECT_EQ(msg.header.frame_id, "frame");
  EXPECT_EQ(msg.encoding, "rgb8");
  EXPECT_EQ(msg.shape[1], 5u);
  EXPECT_EQ(msg.core.slot_id, 3u);
  EXPECT_EQ(msg.core.shm_name, "/demo");
}

}  // namespace
