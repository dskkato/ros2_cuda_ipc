// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <sys/mman.h>

#include "ros2_cuda_ipc_core/pointcloud2/pointcloud2_view_mapper.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "test_mapper_utils.hpp"

namespace ros2_cuda_ipc_core {

class PointCloud2ViewMapperTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { test::RclcppScope::SetUp(); }
  static void TearDownTestSuite() { test::RclcppScope::TearDown(); }
};

TEST_F(PointCloud2ViewMapperTest, InvalidCorePreservesHeaderOnlyBehavior) {
  const std::string shm_name =
      test::make_unique_shm_name("pointcloud_mapper_invalid");
  auto mapping = buffer_metadata::BufferMetadata::create(
      shm_name, test::publisher_instance_id(shm_name), 1);
  ASSERT_TRUE(mapping);

  ros2_cuda_ipc_msgs::msg::GpuPointCloud2 msg;
  msg.header.frame_id = "frame_pc";
  msg.height = 1;
  msg.width = 2;
  msg.point_step = 12;
  msg.row_step = 24;
  msg.is_dense = true;
  msg.core.shm_name = shm_name;
  msg.core.publisher_instance_id = test::publisher_instance_id(shm_name);
  msg.core.slot_id = 0;
  msg.core.device_id = 0;
  msg.core.generation = 42;
  msg.core.byte_size = 24;

  pointcloud2::PointCloud2ViewMapper mapper;
  auto view = mapper.map(msg);
  EXPECT_FALSE(view.core.valid());
  EXPECT_EQ(view.header.frame_id, "frame_pc");
  EXPECT_TRUE(view.fields.empty());

  ::shm_unlink(shm_name.c_str());
}

TEST_F(PointCloud2ViewMapperTest, CopiesLayoutWhenCoreIsValid) {
  auto core =
      test::make_seeded_buffer_core_message("pointcloud_mapper_valid", 31);

  ros2_cuda_ipc_msgs::msg::GpuPointCloud2 msg;
  msg.header.frame_id = "frame_pc";
  msg.height = 1;
  msg.width = 2;
  msg.point_step = 12;
  msg.row_step = 24;
  msg.is_dense = true;
  msg.core = core;
  sensor_msgs::msg::PointField field_x;
  field_x.name = "x";
  field_x.offset = 0;
  field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_x.count = 1;
  sensor_msgs::msg::PointField field_y = field_x;
  field_y.name = "y";
  field_y.offset = 4;
  msg.fields = {field_x, field_y};

  pointcloud2::PointCloud2ViewMapper mapper;
  auto view = mapper.map(msg);
  ASSERT_TRUE(view.core.valid());
  EXPECT_EQ(view.header.frame_id, "frame_pc");
  EXPECT_EQ(view.width, 2u);
  EXPECT_EQ(view.point_step, 12u);
  ASSERT_EQ(view.fields.size(), 2u);
  EXPECT_EQ(view.fields[1].name, "y");
  EXPECT_EQ(view.fields[1].offset, 4u);

  view.core = subscriber::ReadHandle{};
  ::shm_unlink(core.shm_name.c_str());
}

}  // namespace ros2_cuda_ipc_core
