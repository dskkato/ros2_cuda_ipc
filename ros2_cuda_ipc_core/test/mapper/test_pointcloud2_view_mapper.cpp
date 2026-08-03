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
  ros2_cuda_ipc_msgs::msg::GpuPointCloud2 msg;
  msg.header.frame_id = "frame_pc";
  msg.height = 1;
  msg.width = 2;
  msg.point_step = 12;
  msg.row_step = 24;
  msg.is_dense = true;
  msg.core = test::make_cached_buffer_core_message(
      test::test_publisher_pid(), test::next_test_block_id(), 42, 1);
  pointcloud2::PointCloud2ViewMapper mapper;
  const auto view = mapper.map(msg);
  EXPECT_FALSE(view.core.valid());
  EXPECT_EQ(view.header.frame_id, "frame_pc");
}

TEST_F(PointCloud2ViewMapperTest, CopiesLayoutWhenCoreIsValid) {
  const auto core = test::make_seeded_buffer_core_message(31);
  const auto name = test::metadata_shm_name(core);
  ros2_cuda_ipc_msgs::msg::GpuPointCloud2 msg;
  msg.header.frame_id = "frame_pc";
  msg.height = 1;
  msg.width = 2;
  msg.point_step = 12;
  msg.row_step = 24;
  msg.is_dense = true;
  msg.core = core;
  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  msg.fields = {field};
  pointcloud2::PointCloud2ViewMapper mapper;
  auto view = mapper.map(msg);
  ASSERT_TRUE(view.core.valid());
  EXPECT_EQ(view.header.frame_id, "frame_pc");
  EXPECT_EQ(view.width, 2u);
  view.core = subscriber::ReadHandle{};
  ::shm_unlink(name.c_str());
}

}  // namespace ros2_cuda_ipc_core
