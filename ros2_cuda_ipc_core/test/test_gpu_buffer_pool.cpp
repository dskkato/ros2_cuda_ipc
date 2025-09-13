#include <gtest/gtest.h>
#include "ros2_cuda_ipc_core/gpu_buffer_pool.hpp"

using ros2_cuda_ipc_core::GpuBufferPool;

TEST(GpuBufferPool, BorrowRelease) {
  GpuBufferPool pool(2);
  auto s0 = pool.borrow();
  ASSERT_TRUE(s0.has_value());
  auto s1 = pool.borrow();
  ASSERT_TRUE(s1.has_value());
  auto s2 = pool.borrow();
  EXPECT_FALSE(s2.has_value());
  pool.release(*s0);
  auto s3 = pool.borrow();
  EXPECT_TRUE(s3.has_value());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
