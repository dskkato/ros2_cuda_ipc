// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <sys/mman.h>

#include "ros2_cuda_ipc_core/image/image_view_mapper.hpp"
#include "test_mapper_utils.hpp"

namespace ros2_cuda_ipc_core {

class ImageViewMapperTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { test::RclcppScope::SetUp(); }
  static void TearDownTestSuite() { test::RclcppScope::TearDown(); }
};

TEST_F(ImageViewMapperTest, InvalidCoreReturnsDefaultImageView) {
  ros2_cuda_ipc_msgs::msg::GpuImage msg;
  msg.header.frame_id = "frame";
  msg.dtype = static_cast<uint8_t>(image::DType::U8);
  msg.shape = {4, 5, 3};
  msg.strides = {15, 3, 1};
  msg.encoding = "rgb8";
  msg.core = test::make_cached_buffer_core_message(
      test::test_publisher_pid(), test::next_test_block_id(), 42, 1);
  image::ImageViewMapper mapper;
  const auto view = mapper.map(msg);
  EXPECT_FALSE(view.valid());
  EXPECT_TRUE(view.header.frame_id.empty());
}

TEST_F(ImageViewMapperTest, CopiesMetadataWhenPublicationIsValid) {
  const auto core = test::make_seeded_buffer_core_message(21);
  const auto name = test::metadata_shm_name(core);
  auto mapping = buffer_metadata::BufferMetadata::attach(name);
  ASSERT_TRUE(mapping);

  ros2_cuda_ipc_msgs::msg::GpuImage msg;
  msg.header.frame_id = "camera_frame";
  msg.dtype = static_cast<uint8_t>(image::DType::U16);
  msg.shape = {4, 5, 3};
  msg.strides = {30, 6, 2};
  msg.encoding = "mono16";
  msg.core = core;
  image::ImageViewMapper mapper;
  auto view = mapper.map(msg);
  ASSERT_TRUE(view.valid());
  EXPECT_EQ(buffer_metadata::BufferRef::current_refcount(mapping), 1u);
  EXPECT_EQ(view.header.frame_id, "camera_frame");
  EXPECT_EQ(view.dtype, image::DType::U16);
  view = image::ImageView{};
  EXPECT_EQ(buffer_metadata::BufferRef::current_refcount(mapping), 0u);
  ::shm_unlink(name.c_str());
}

}  // namespace ros2_cuda_ipc_core
