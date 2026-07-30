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
  const std::string shm_name =
      test::make_unique_shm_name("image_mapper_invalid");
  auto mapping = buffer_metadata::BufferMetadata::create(
      shm_name, test::publisher_instance_id(shm_name), 1);
  ASSERT_TRUE(mapping);

  ros2_cuda_ipc_msgs::msg::GpuImage msg;
  msg.header.frame_id = "frame";
  msg.dtype = static_cast<uint8_t>(image::DType::U8);
  msg.shape = {4, 5, 3};
  msg.strides = {15, 3, 1};
  msg.encoding = "rgb8";
  msg.core.shm_name = shm_name;
  msg.core.publisher_instance_id = test::publisher_instance_id(shm_name);
  msg.core.slot_id = 0;
  msg.core.device_id = 0;
  msg.core.generation = 42;
  msg.core.byte_size = 60;
  msg.core.backend = ros2_cuda_ipc_msgs::msg::BufferCore::CUDA_IPC;

  image::ImageViewMapper mapper;
  auto view = mapper.map(msg);
  EXPECT_FALSE(view.valid());
  EXPECT_TRUE(view.header.frame_id.empty());

  ::shm_unlink(shm_name.c_str());
}

TEST_F(ImageViewMapperTest, CopiesMetadataWhenPublicationIsValid) {
  auto core = test::make_seeded_buffer_core_message("image_mapper_valid", 21);

  ros2_cuda_ipc_msgs::msg::GpuImage msg;
  msg.header.frame_id = "camera_frame";
  msg.dtype = static_cast<uint8_t>(image::DType::U16);
  msg.shape = {4, 5, 3};
  msg.strides = {30, 6, 2};
  msg.encoding = "mono16";
  msg.core = core;

  auto mapping = buffer_metadata::BufferMetadata::attach(
      core.shm_name, core.publisher_instance_id);
  ASSERT_TRUE(mapping);
  auto before =
      buffer_metadata::BufferRef::current_refcount(mapping, core.slot_id);
  ASSERT_TRUE(before.has_value());
  EXPECT_EQ(before.value(), 0u);

  image::ImageViewMapper mapper;
  auto view = mapper.map(msg);
  ASSERT_TRUE(view.valid());
  EXPECT_FALSE(view.core.valid());
  auto during =
      buffer_metadata::BufferRef::current_refcount(mapping, core.slot_id);
  ASSERT_TRUE(during.has_value());
  EXPECT_EQ(during.value(), 1u);
  EXPECT_EQ(view.header.frame_id, "camera_frame");
  EXPECT_EQ(view.dtype, image::DType::U16);
  EXPECT_EQ(view.shape[1], 5u);
  EXPECT_EQ(view.strides[0], 30u);
  EXPECT_EQ(view.encoding, "mono16");

  view = image::ImageView{};
  auto after =
      buffer_metadata::BufferRef::current_refcount(mapping, core.slot_id);
  ASSERT_TRUE(after.has_value());
  EXPECT_EQ(after.value(), 0u);
  ::shm_unlink(core.shm_name.c_str());
}

TEST_F(ImageViewMapperTest, DlpackMappingKeepsPublicationUnbound) {
  auto core = test::make_seeded_buffer_core_message("image_mapper_dlpack", 41);

  ros2_cuda_ipc_msgs::msg::GpuImage msg;
  msg.dtype = static_cast<uint8_t>(image::DType::U8);
  msg.shape = {2, 3, 4};
  msg.strides = {12, 4, 1};
  msg.core = core;

  auto mapping = buffer_metadata::BufferMetadata::attach(
      core.shm_name, core.publisher_instance_id);
  ASSERT_TRUE(mapping);

  image::ImageViewMapper mapper;
  auto view = mapper.map_for_dlpack(msg);
  ASSERT_TRUE(view.valid());
  EXPECT_FALSE(view.core.valid());
  auto during =
      buffer_metadata::BufferRef::current_refcount(mapping, core.slot_id);
  ASSERT_TRUE(during.has_value());
  EXPECT_EQ(during.value(), 1u);

  view = image::ImageView{};
  auto after =
      buffer_metadata::BufferRef::current_refcount(mapping, core.slot_id);
  ASSERT_TRUE(after.has_value());
  EXPECT_EQ(after.value(), 0u);
  ::shm_unlink(core.shm_name.c_str());
}

}  // namespace ros2_cuda_ipc_core
