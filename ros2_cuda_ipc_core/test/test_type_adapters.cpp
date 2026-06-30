// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <sys/mman.h>

#include "ros2_cuda_ipc_core/type_adapters.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "test_mapper_utils.hpp"

namespace {

class TypeAdapterTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    ros2_cuda_ipc_core::test::RclcppScope::SetUp();
  }
  static void TearDownTestSuite() {
    ros2_cuda_ipc_core::test::RclcppScope::TearDown();
  }
};

TEST_F(TypeAdapterTest, BufferViewAdapterDelegatesReceivePath) {
  auto msg = ros2_cuda_ipc_core::test::make_seeded_buffer_core_message(
      "adapter_buffer", 41);

  ros2_cuda_ipc_core::view::BufferView view;
  rclcpp::TypeAdapter<
      ros2_cuda_ipc_core::view::BufferView,
      ros2_cuda_ipc_msgs::msg::BufferCore>::convert_to_custom(msg, view);

  EXPECT_TRUE(view.valid());
  EXPECT_EQ(view.slot_id, msg.slot_id);
  EXPECT_EQ(view.shm_name, msg.shm_name);

  view.reset();
  ::shm_unlink(msg.shm_name.c_str());
}

TEST_F(TypeAdapterTest, ImageViewAdapterDelegatesSendPath) {
  ros2_cuda_ipc_core::view::ImageView view;
  view.header.frame_id = "frame";
  view.dtype = ros2_cuda_ipc_core::view::DType::U8;
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
  rclcpp::TypeAdapter<
      ros2_cuda_ipc_core::view::ImageView,
      ros2_cuda_ipc_msgs::msg::GpuImage>::convert_to_ros_message(view, msg);

  EXPECT_EQ(msg.header.frame_id, "frame");
  EXPECT_EQ(msg.encoding, "rgb8");
  EXPECT_EQ(msg.shape[1], 5u);
  EXPECT_EQ(msg.core.slot_id, 3u);
}

TEST_F(TypeAdapterTest, PointCloud2AdapterDelegatesReceiveAndSendPath) {
  auto core = ros2_cuda_ipc_core::test::make_seeded_buffer_core_message(
      "adapter_pointcloud", 51);

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
  msg.fields = {field_x};

  ros2_cuda_ipc_core::view::PointCloud2View view;
  rclcpp::TypeAdapter<
      ros2_cuda_ipc_core::view::PointCloud2View,
      ros2_cuda_ipc_msgs::msg::GpuPointCloud2>::convert_to_custom(msg, view);

  ASSERT_TRUE(view.core.valid());
  EXPECT_EQ(view.header.frame_id, "frame_pc");
  EXPECT_EQ(view.fields.size(), 1u);

  ros2_cuda_ipc_msgs::msg::GpuPointCloud2 roundtrip;
  rclcpp::TypeAdapter<ros2_cuda_ipc_core::view::PointCloud2View,
                      ros2_cuda_ipc_msgs::msg::GpuPointCloud2>::
      convert_to_ros_message(view, roundtrip);

  EXPECT_EQ(roundtrip.header.frame_id, "frame_pc");
  EXPECT_EQ(roundtrip.width, 2u);
  EXPECT_EQ(roundtrip.fields.size(), 1u);

  view.core.reset();
  ::shm_unlink(core.shm_name.c_str());
}

}  // namespace
