#include <gtest/gtest.h>

#include <optional>
#include <string>

#include "ros2_cuda_ipc_core/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_core/lease_manager.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_buffer.hpp"
#include "sample_nodes/gpu_buffer_publisher_helper.hpp"

using namespace ros2_cuda_ipc_core;

TEST(GpuBufferPublisherHelperTest, MetadataOnlyWhenNoCudaMem) {
  // Pool with no CUDA allocations (bytes_per_slot=0)
  GpuBufferPool pool(1);
  LeaseManager lm(pool, /*lease_timeout_ms=*/50);

  sample_nodes::GpuBufferPublisherHelper helper(pool, &lm, /*stream=*/nullptr);
  auto f = helper.borrow_frame(/*w=*/16, /*h=*/8, /*c=*/3);
  ASSERT_TRUE(f.has_value());

  ros2_cuda_ipc_msgs::msg::GpuBuffer msg;
  sample_nodes::GpuBufferPublisherHelper::PublishParams params;
  params.seq_id = 1;
  params.expected_consumers = 0;
  params.shm_owner = "test_owner";
  params.width = 16;
  params.height = 8;
  params.channels = 3;
  params.layout = ros2_cuda_ipc_msgs::msg::GpuBuffer::LAYOUT_LINEAR;
  params.format = ros2_cuda_ipc_msgs::msg::GpuBuffer::FORMAT_BGR8;
  bool ok = helper.finalize_and_fill(*f, params, msg);
  EXPECT_FALSE(ok);
  EXPECT_EQ(msg.plane_count, 0u);

  // Slot should be released immediately (expected_consumers=0)
  auto s2 = pool.borrow();
  EXPECT_TRUE(s2.has_value());
}

TEST(GpuBufferPublisherHelperTest, KeepsSlotOnLeaseStart) {
  // No CUDA mem; finalize should release since mem handle export fails.
  GpuBufferPool pool(1);
  LeaseManager lm(pool, /*lease_timeout_ms=*/20);
  sample_nodes::GpuBufferPublisherHelper helper(pool, &lm, /*stream=*/nullptr);

  auto f = helper.borrow_frame(10, 10, 1);
  ASSERT_TRUE(f.has_value());

  ros2_cuda_ipc_msgs::msg::GpuBuffer msg;
  sample_nodes::GpuBufferPublisherHelper::PublishParams params;
  params.seq_id = 2;
  params.expected_consumers = 2;
  params.shm_owner = "owner";
  params.width = 10;
  params.height = 10;
  params.channels = 1;
  params.layout = ros2_cuda_ipc_msgs::msg::GpuBuffer::LAYOUT_LINEAR;
  params.format = ros2_cuda_ipc_msgs::msg::GpuBuffer::FORMAT_BGR8;
  bool ok = helper.finalize_and_fill(*f, params, msg);
  // Without CUDA mem, plane cannot be populated
  EXPECT_FALSE(ok);
  EXPECT_EQ(msg.plane_count, 0u);

  // Slot should be released due to no mem
  auto s2 = pool.borrow();
  EXPECT_TRUE(s2.has_value());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
