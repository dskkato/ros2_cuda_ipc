#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "ros2_cuda_ipc_core/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_core/lease_manager.hpp"
#include "ros2_cuda_ipc_core/shm_release.hpp"

using namespace ros2_cuda_ipc_core;

TEST(LeaseManagerTest, ShmReleaseFlowReleasesSlot) {
  // Pool without CUDA allocations
  GpuBufferPool pool(1);
  auto s0 = pool.borrow();
  ASSERT_TRUE(s0.has_value());

  LeaseManager lm(pool, /*lease_timeout_ms=*/3000);
  lm.set_owner("test_owner");

  const uint64_t seq = 1;
  auto shm_name =
      lm.start_lease(static_cast<uint32_t>(*s0), seq, /*expected=*/1);
  ASSERT_TRUE(shm_name.has_value());

  // Decrement once -> refcnt should hit 0
  auto newv = shm_decrement(*shm_name, static_cast<uint32_t>(*s0), seq);
  ASSERT_TRUE(newv.has_value());
  EXPECT_EQ(*newv, 0);

  // Tick should detect refcnt==0 and release the slot
  int released = lm.tick();
  EXPECT_EQ(released, 1);

  // Slot can be borrowed again
  auto s1 = pool.borrow();
  EXPECT_TRUE(s1.has_value());
}

TEST(LeaseManagerTest, TimeoutReleasesSlot) {
  GpuBufferPool pool(1);
  auto s0 = pool.borrow();
  ASSERT_TRUE(s0.has_value());

  // Very short timeout to trigger release without SHM decrement
  LeaseManager lm(pool, /*lease_timeout_ms=*/10);
  lm.set_owner("timeout_owner");

  const uint64_t seq = 42;
  auto shm_name =
      lm.start_lease(static_cast<uint32_t>(*s0), seq, /*expected=*/3);
  ASSERT_TRUE(shm_name.has_value());

  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  int released = lm.tick();
  EXPECT_EQ(released, 1);

  auto s1 = pool.borrow();
  EXPECT_TRUE(s1.has_value());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
