// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <sys/mman.h>

#include "../test_mapper_utils.hpp"
#include "ros2_cuda_ipc_core/mapper/pointcloud2_view_mapper.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace {

class PointCloud2ViewMapperTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    ros2_cuda_ipc_core::test::RclcppScope::SetUp();
  }
  static void TearDownTestSuite() {
    ros2_cuda_ipc_core::test::RclcppScope::TearDown();
  }
};

TEST_F(PointCloud2ViewMapperTest, InvalidCorePreservesHeaderOnlyBehavior) {
  const std::string shm_name = ros2_cuda_ipc_core::test::make_unique_shm_name(
      "pointcloud_mapper_invalid");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));

  ros2_cuda_ipc_msgs::msg::GpuPointCloud2 msg;
  msg.header.frame_id = "frame_pc";
  msg.height = 1;
  msg.width = 2;
  msg.point_step = 12;
  msg.row_step = 24;
  msg.is_dense = true;
  msg.core.shm_name = shm_name;
  msg.core.slot_id = 0;
  msg.core.device_id = 0;
  msg.core.generation = 42;
  msg.core.byte_size = 24;
  msg.core.backend = ros2_cuda_ipc_msgs::msg::BufferCore::CUDA_IPC;

  ros2_cuda_ipc_core::mapper::PointCloud2ViewMapper mapper;
  auto view = mapper.map(msg);
  EXPECT_FALSE(view.core.valid());
  EXPECT_EQ(view.header.frame_id, "frame_pc");
  EXPECT_TRUE(view.fields.empty());

  ::shm_unlink(shm_name.c_str());
}

TEST_F(PointCloud2ViewMapperTest, CopiesLayoutWhenCoreIsValid) {
  auto core = ros2_cuda_ipc_core::test::make_seeded_buffer_core_message(
      "pointcloud_mapper_valid", 31);

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

  ros2_cuda_ipc_core::mapper::PointCloud2ViewMapper mapper;
  auto view = mapper.map(msg);
  ASSERT_TRUE(view.core.valid());
  EXPECT_EQ(view.header.frame_id, "frame_pc");
  EXPECT_EQ(view.width, 2u);
  EXPECT_EQ(view.point_step, 12u);
  ASSERT_EQ(view.fields.size(), 2u);
  EXPECT_EQ(view.fields[1].name, "y");
  EXPECT_EQ(view.fields[1].offset, 4u);

  view.core.reset();
  ::shm_unlink(core.shm_name.c_str());
}

TEST_F(PointCloud2ViewMapperTest,
       FillGpuPointCloud2MessageCopiesLayoutAndCore) {
  ros2_cuda_ipc_core::view::PointCloud2View view;
  view.header.frame_id = "frame";
  view.core.device_id = 1;
  view.core.byte_size = 120;
  view.core.slot_id = 5;
  view.core.generation = 9;
  view.core.shm_name = "/pc_demo";
  cudaIpcMemHandle_t mem_handle{};
  cudaIpcEventHandle_t event_handle{};
  view.core.set_ipc_handles(ros2_cuda_ipc_core::MemoryBackendKind::CUDA_IPC,
                            reinterpret_cast<const uint8_t*>(&mem_handle),
                            sizeof(mem_handle), event_handle);
  view.height = 1;
  view.width = 10;
  view.point_step = 12;
  view.row_step = 120;
  view.is_dense = true;
  view.fields = {{"x", 0u, sensor_msgs::msg::PointField::FLOAT32, 1u},
                 {"y", 4u, sensor_msgs::msg::PointField::FLOAT32, 1u}};

  ros2_cuda_ipc_msgs::msg::GpuPointCloud2 msg;
  ros2_cuda_ipc_core::mapper::fill_gpu_pointcloud2_message(view, msg);

  EXPECT_EQ(msg.header.frame_id, "frame");
  EXPECT_EQ(msg.width, 10u);
  EXPECT_EQ(msg.fields.size(), 2u);
  EXPECT_EQ(msg.core.slot_id, 5u);
  EXPECT_EQ(msg.core.shm_name, "/pc_demo");
}

}  // namespace
