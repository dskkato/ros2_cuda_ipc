#include <cuda_runtime_api.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <sstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/cuda/gpu_lease_pool.hpp"
#include "ros2_cuda_ipc_core/lease_handle.hpp"

namespace {

std::string make_unique_shm_name(const std::string &prefix) {
  static std::atomic<int> counter{0};
  std::ostringstream oss;
  oss << "/" << prefix << "_" << ::getpid() << "_" << counter.fetch_add(1);
  return oss.str();
}

}  // namespace

using ros2_cuda_ipc_core::cuda::GpuLeasePool;

class GpuLeasePoolTest : public ::testing::Test {
 protected:
  void SetUp() override {
    int device_count = 0;
    auto err = cudaGetDeviceCount(&device_count);
    if (err != cudaSuccess || device_count == 0) {
      GTEST_SKIP() << "CUDA device not available for tests";
    }

    err = cudaSetDevice(0);
    if (err != cudaSuccess) {
      GTEST_SKIP() << "Failed to select CUDA device 0";
    }
  }
};

TEST_F(GpuLeasePoolTest,
       InitialiseAndAcquireSetsPendingWhenSubscribersPresent) {
  const std::string shm_name = make_unique_shm_name("gpu_pool_basic");
  auto logger = rclcpp::get_logger("GpuLeasePoolTest");
  GpuLeasePool pool({shm_name, 2, std::chrono::milliseconds(100)}, logger);

  ASSERT_TRUE(pool.initialise(1024, 0));
  EXPECT_TRUE(pool.is_initialised());
  EXPECT_EQ(pool.frame_size_bytes(), 1024u);
  EXPECT_EQ(pool.device_index(), 0);

  auto slot_opt = pool.acquire(1);
  ASSERT_TRUE(slot_opt.has_value());
  auto *slot = *slot_opt;
  ASSERT_NE(slot, nullptr);
  EXPECT_NE(slot->device_ptr, nullptr);
  EXPECT_NE(slot->event, nullptr);
  EXPECT_GT(slot->generation, 0u);
  EXPECT_NE(slot->pending_deadline.time_since_epoch().count(), 0);

  auto pending =
      ros2_cuda_ipc_core::LeaseHandle::current_pending(shm_name, slot->index);
  ASSERT_TRUE(pending.has_value());
  EXPECT_EQ(pending.value(), 1u);

  pool.reset();
  ::shm_unlink(shm_name.c_str());
}

TEST_F(GpuLeasePoolTest, AcquireWithoutSubscribersDoesNotSetPendingDeadline) {
  const std::string shm_name = make_unique_shm_name("gpu_pool_nosub");
  auto logger = rclcpp::get_logger("GpuLeasePoolTest");
  GpuLeasePool pool({shm_name, 1, std::chrono::milliseconds(100)}, logger);

  ASSERT_TRUE(pool.initialise(512, 0));

  auto slot_opt = pool.acquire(0);
  ASSERT_TRUE(slot_opt.has_value());
  auto *slot = *slot_opt;
  ASSERT_NE(slot, nullptr);
  EXPECT_EQ(slot->pending_deadline.time_since_epoch().count(), 0);

  auto pending =
      ros2_cuda_ipc_core::LeaseHandle::current_pending(shm_name, slot->index);
  ASSERT_TRUE(pending.has_value());
  EXPECT_EQ(pending.value(), 0u);

  pool.reset();
  ::shm_unlink(shm_name.c_str());
}

TEST_F(GpuLeasePoolTest, ReclaimStalePendingClearsLease) {
  const std::string shm_name = make_unique_shm_name("gpu_pool_reclaim");
  auto logger = rclcpp::get_logger("GpuLeasePoolTest");
  GpuLeasePool pool({shm_name, 1, std::chrono::milliseconds(1)}, logger);

  ASSERT_TRUE(pool.initialise(256, 0));

  auto slot_opt = pool.acquire(1);
  ASSERT_TRUE(slot_opt.has_value());
  auto *slot = *slot_opt;
  ASSERT_NE(slot, nullptr);

  auto pending_before =
      ros2_cuda_ipc_core::LeaseHandle::current_pending(shm_name, slot->index);
  ASSERT_TRUE(pending_before.has_value());
  EXPECT_EQ(pending_before.value(), 1u);

  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  pool.reclaim_stale_pending();

  auto pending_after =
      ros2_cuda_ipc_core::LeaseHandle::current_pending(shm_name, slot->index);
  ASSERT_TRUE(pending_after.has_value());
  EXPECT_EQ(pending_after.value(), 0u);

  pool.reset();
  ::shm_unlink(shm_name.c_str());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
