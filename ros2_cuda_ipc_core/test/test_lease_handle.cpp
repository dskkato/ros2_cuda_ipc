#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <sstream>
#include <string>

#include "ros2_cuda_ipc_core/lease_handle.hpp"

namespace {

std::string make_unique_shm_name(const std::string& prefix) {
  static std::atomic<int> counter{0};
  std::ostringstream oss;
  oss << "/" << prefix << "_" << ::getpid() << "_" << counter.fetch_add(1);
  return oss.str();
}

}  // namespace

TEST(LeaseHandleTest, AcquireReleaseLifecycle) {
  const std::string shm_name = make_unique_shm_name("lease_ut");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 2));

  auto slot = ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
  ASSERT_TRUE(slot.has_value());
  auto gen = ros2_cuda_ipc_core::LeaseHandle::bump_generation(shm_name,
                                                              slot.value(), 0);
  ASSERT_TRUE(gen.has_value());

  {
    auto lease = ros2_cuda_ipc_core::LeaseHandle::acquire(
        shm_name, slot.value(), gen.value());
    ASSERT_TRUE(lease.valid());

    auto other_slot =
        ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
    ASSERT_TRUE(other_slot.has_value());
    EXPECT_NE(other_slot.value(), slot.value());

    auto ref = ros2_cuda_ipc_core::LeaseHandle::current_refcount(shm_name,
                                                                 slot.value());
    ASSERT_TRUE(ref.has_value());
    EXPECT_EQ(ref.value(), 1u);
  }

  auto ref_after =
      ros2_cuda_ipc_core::LeaseHandle::current_refcount(shm_name, slot.value());
  ASSERT_TRUE(ref_after.has_value());
  EXPECT_EQ(ref_after.value(), 0u);

  auto slot_after =
      ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
  ASSERT_TRUE(slot_after.has_value());
  EXPECT_EQ(slot_after.value(), slot.value());

  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, GenerationMismatchReturnsInvalid) {
  const std::string shm_name = make_unique_shm_name("lease_mismatch");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));

  auto slot = ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
  ASSERT_TRUE(slot.has_value());
  auto gen = ros2_cuda_ipc_core::LeaseHandle::bump_generation(shm_name,
                                                              slot.value(), 0);
  ASSERT_TRUE(gen.has_value());

  auto lease = ros2_cuda_ipc_core::LeaseHandle::acquire(shm_name, slot.value(),
                                                        gen.value() + 1);
  EXPECT_FALSE(lease.valid());

  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, PendingPreventsSlotReuse) {
  const std::string shm_name = make_unique_shm_name("lease_pending");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));

  auto slot = ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
  ASSERT_TRUE(slot.has_value());
  auto gen = ros2_cuda_ipc_core::LeaseHandle::bump_generation(shm_name,
                                                              slot.value(), 2);
  ASSERT_TRUE(gen.has_value());

  auto free_slot = ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
  EXPECT_FALSE(free_slot.has_value());

  auto next_gen = ros2_cuda_ipc_core::LeaseHandle::bump_generation(
      shm_name, slot.value(), 0);
  ASSERT_TRUE(next_gen.has_value());

  free_slot = ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
  ASSERT_TRUE(free_slot.has_value());
  EXPECT_EQ(free_slot.value(), slot.value());

  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, PendingDecrementedOnAcquire) {
  const std::string shm_name = make_unique_shm_name("lease_pending_dec");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));

  auto slot = ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
  ASSERT_TRUE(slot.has_value());
  auto gen = ros2_cuda_ipc_core::LeaseHandle::bump_generation(shm_name,
                                                              slot.value(), 2);
  ASSERT_TRUE(gen.has_value());

  {
    auto lease = ros2_cuda_ipc_core::LeaseHandle::acquire(
        shm_name, slot.value(), gen.value());
    ASSERT_TRUE(lease.valid());
    auto pending = ros2_cuda_ipc_core::LeaseHandle::current_pending(
        shm_name, slot.value());
    ASSERT_TRUE(pending.has_value());
    EXPECT_EQ(pending.value(), 1u);
  }

  {
    auto lease = ros2_cuda_ipc_core::LeaseHandle::acquire(
        shm_name, slot.value(), gen.value());
    ASSERT_TRUE(lease.valid());
    auto pending = ros2_cuda_ipc_core::LeaseHandle::current_pending(
        shm_name, slot.value());
    ASSERT_TRUE(pending.has_value());
    EXPECT_EQ(pending.value(), 0u);
  }

  auto free_slot = ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
  ASSERT_TRUE(free_slot.has_value());
  EXPECT_EQ(free_slot.value(), slot.value());

  {
    auto lease = ros2_cuda_ipc_core::LeaseHandle::acquire(
        shm_name, slot.value(), gen.value());
    ASSERT_TRUE(lease.valid());
    auto pending = ros2_cuda_ipc_core::LeaseHandle::current_pending(
        shm_name, slot.value());
    ASSERT_TRUE(pending.has_value());
    EXPECT_EQ(pending.value(), 0u);
  }

  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, ForceClearPendingResetsCounterWhenIdle) {
  const std::string shm_name = make_unique_shm_name("lease_force_clear");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));

  auto gen = ros2_cuda_ipc_core::LeaseHandle::bump_generation(shm_name, 0, 2);
  ASSERT_TRUE(gen.has_value());

  auto pending = ros2_cuda_ipc_core::LeaseHandle::current_pending(shm_name, 0);
  ASSERT_TRUE(pending.has_value());
  EXPECT_EQ(pending.value(), 2u);

  EXPECT_TRUE(
      ros2_cuda_ipc_core::LeaseHandle::force_clear_pending(shm_name, 0));

  pending = ros2_cuda_ipc_core::LeaseHandle::current_pending(shm_name, 0);
  ASSERT_TRUE(pending.has_value());
  EXPECT_EQ(pending.value(), 0u);

  ::shm_unlink(shm_name.c_str());
}
