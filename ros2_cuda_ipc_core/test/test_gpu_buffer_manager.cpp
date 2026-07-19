// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>

#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/lease/lease_handle.hpp"
#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"

namespace {
std::string unique_name() {
  static std::atomic<int> counter{0};
  std::ostringstream out;
  out << "/gpu_buffer_manager_" << ::getpid() << "_" << counter.fetch_add(1);
  return out.str();
}
}  // namespace

using ros2_cuda_ipc_core::publisher::GpuBufferManager;
using ros2_cuda_ipc_core::publisher::PublishSlot;

static_assert(!std::is_copy_constructible_v<PublishSlot>);
static_assert(!std::is_copy_assignable_v<PublishSlot>);
static_assert(std::is_nothrow_move_constructible_v<PublishSlot>);
static_assert(std::is_nothrow_move_assignable_v<PublishSlot>);

class GpuBufferManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    int count = 0;
    if (cudaGetDeviceCount(&count) != cudaSuccess || count == 0 ||
        cudaSetDevice(0) != cudaSuccess) {
      GTEST_SKIP() << "CUDA device not available";
    }
    shm_name_ = unique_name();
  }
  GpuBufferManager make_manager(
      std::chrono::milliseconds pending_ttl = std::chrono::milliseconds(100)) {
    return GpuBufferManager(
        {shm_name_, 1, 1024, 0, pending_ttl,
         ros2_cuda_ipc_core::transport::MemoryBackendKind::CUDA_IPC},
        rclcpp::get_logger("GpuBufferManagerTest"));
  }
  std::string shm_name_;
};

TEST_F(GpuBufferManagerTest, DescriptorIsGatedByReadyRecording) {
  auto manager = make_manager();
  ASSERT_TRUE(manager.initialise());
  const auto instance_id = manager.publisher_instance_id();
  auto slot = manager.acquire_for_publish(1);
  ASSERT_TRUE(slot.has_value());
  EXPECT_EQ(slot->descriptor(), std::nullopt);
  ASSERT_EQ(slot->record_ready(nullptr), cudaSuccess);
  auto descriptor = slot->descriptor();
  ASSERT_TRUE(descriptor.has_value());
  EXPECT_EQ(descriptor->slot_id, 0u);
  EXPECT_NE(descriptor->lease_shm_name, shm_name_);
  EXPECT_EQ(descriptor->lease_shm_name, manager.shm_name());
  EXPECT_EQ(descriptor->publisher_instance_id, instance_id);
  EXPECT_EQ(descriptor->byte_size, 1024u);
  EXPECT_NE(slot->record_ready(nullptr), cudaSuccess);
  slot->commit_publish();
  slot->commit_publish();
  EXPECT_FALSE(slot->valid());
}

TEST_F(GpuBufferManagerTest, UncommittedDestructionCancelsReservation) {
  auto manager = make_manager();
  ASSERT_TRUE(manager.initialise());
  {
    auto slot = manager.acquire_for_publish(1);
    ASSERT_TRUE(slot.has_value());
  }
  EXPECT_TRUE(manager.acquire_for_publish(1).has_value());
}

TEST_F(GpuBufferManagerTest, CommittedDestructionKeepsPending) {
  auto manager = make_manager();
  ASSERT_TRUE(manager.initialise());
  {
    auto slot = manager.acquire_for_publish(1);
    ASSERT_TRUE(slot.has_value());
    ASSERT_EQ(slot->record_ready(nullptr), cudaSuccess);
    slot->commit_publish();
  }
  EXPECT_FALSE(manager.acquire_for_publish(1).has_value());
}

TEST_F(GpuBufferManagerTest, MovedFromSlotIsInert) {
  auto manager = make_manager();
  ASSERT_TRUE(manager.initialise());
  auto source = manager.acquire_for_publish(1);
  ASSERT_TRUE(source.has_value());
  PublishSlot destination(std::move(*source));
  EXPECT_FALSE(source->valid());
  EXPECT_EQ(source->device_ptr(), nullptr);
  EXPECT_TRUE(destination.valid());
  destination.cancel();
  destination.cancel();
  EXPECT_FALSE(destination.valid());
  EXPECT_TRUE(manager.acquire_for_publish(1).has_value());
}

TEST_F(GpuBufferManagerTest, ResetDoesNotPreventReservationCancellation) {
  auto manager = make_manager();
  ASSERT_TRUE(manager.initialise());
  const std::string actual_name = manager.shm_name();
  const auto instance_id = manager.publisher_instance_id();
  auto slot = manager.acquire_for_publish(1);
  ASSERT_TRUE(slot.has_value());
  const auto pending_before =
      ros2_cuda_ipc_core::lease::LeaseHandle::current_pending(actual_name,
                                                              instance_id, 0);
  ASSERT_TRUE(pending_before.has_value());
  ASSERT_EQ(*pending_before, 1u);

  manager.reset();
  slot.reset();

  const auto pending = ros2_cuda_ipc_core::lease::LeaseHandle::current_pending(
      actual_name, instance_id, 0);
  ASSERT_TRUE(pending.has_value());
  EXPECT_EQ(*pending, 0u);
}

TEST_F(GpuBufferManagerTest, AcquireAutomaticallyReclaimsExpiredPending) {
  auto manager = make_manager(std::chrono::milliseconds(1));
  ASSERT_TRUE(manager.initialise());
  {
    auto slot = manager.acquire_for_publish(1);
    ASSERT_TRUE(slot.has_value());
    ASSERT_EQ(slot->record_ready(nullptr), cudaSuccess);
    slot->commit_publish();
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_TRUE(manager.acquire_for_publish(1).has_value());
}
