#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <sstream>
#include <string>

#include "ros2_cuda_ipc_core/lease_handle.hpp"

namespace {

std::string make_unique_shm_name(const std::string &prefix) {
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
  auto gen =
      ros2_cuda_ipc_core::LeaseHandle::bump_generation(shm_name, slot.value());
  ASSERT_TRUE(gen.has_value());

  {
    auto lease = ros2_cuda_ipc_core::LeaseHandle::acquire(
        shm_name, slot.value(), gen.value());
    ASSERT_TRUE(lease.valid());

    auto other_slot =
        ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
    ASSERT_TRUE(other_slot.has_value());
    EXPECT_NE(other_slot.value(), slot.value());
    EXPECT_TRUE(ros2_cuda_ipc_core::LeaseHandle::return_slot(
        shm_name, other_slot.value()));

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
  auto gen =
      ros2_cuda_ipc_core::LeaseHandle::bump_generation(shm_name, slot.value());
  ASSERT_TRUE(gen.has_value());

  auto lease = ros2_cuda_ipc_core::LeaseHandle::acquire(shm_name, slot.value(),
                                                        gen.value() + 1);
  EXPECT_FALSE(lease.valid());

  ::shm_unlink(shm_name.c_str());
}

TEST(LeaseHandleTest, ReturnSlotRespectsRefcount) {
  const std::string shm_name = make_unique_shm_name("lease_return");
  ASSERT_TRUE(ros2_cuda_ipc_core::LeaseHandle::init(shm_name, 1));

  auto slot = ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
  ASSERT_TRUE(slot.has_value());

  auto gen =
      ros2_cuda_ipc_core::LeaseHandle::bump_generation(shm_name, slot.value());
  ASSERT_TRUE(gen.has_value());

  {
    auto lease = ros2_cuda_ipc_core::LeaseHandle::acquire(
        shm_name, slot.value(), gen.value());
    ASSERT_TRUE(lease.valid());

    EXPECT_FALSE(
        ros2_cuda_ipc_core::LeaseHandle::return_slot(shm_name, slot.value()));
  }

  auto recycled = ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(shm_name);
  ASSERT_TRUE(recycled.has_value());
  EXPECT_EQ(recycled.value(), slot.value());

  ::shm_unlink(shm_name.c_str());
}
