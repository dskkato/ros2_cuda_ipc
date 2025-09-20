#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

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
  EXPECT_TRUE(pool.release(*s0));
  EXPECT_FALSE(pool.release(pool.capacity()));
  auto s3 = pool.borrow();
  EXPECT_TRUE(s3.has_value());
}

TEST(GpuBufferPool, BlockingBorrowWaits) {
  GpuBufferPool pool(1);
  auto s0 = pool.borrow(true);
  ASSERT_TRUE(s0.has_value());

  std::atomic<bool> got_slot{false};
  std::thread t([&]() {
    auto s1 = pool.borrow(true);
    if (s1.has_value()) {
      got_slot = true;
      pool.release(*s1);
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_FALSE(got_slot.load());
  EXPECT_TRUE(pool.release(*s0));
  t.join();
  EXPECT_TRUE(got_slot.load());
}

TEST(GpuBufferPool, ExpandPool) {
  GpuBufferPool pool(1);
  EXPECT_EQ(pool.capacity(), 1u);
  EXPECT_TRUE(pool.expand_pool(2));
  EXPECT_EQ(pool.capacity(), 3u);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
