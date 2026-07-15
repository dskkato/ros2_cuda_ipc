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
#include "ros2_cuda_ipc_core/cuda/gpu_buffer_controller.hpp"
#include "ros2_cuda_ipc_core/lease_handle.hpp"

namespace {
std::string unique_name() {
  static std::atomic<int> counter{0};
  std::ostringstream out;
  out << "/gpu_controller_" << ::getpid() << "_" << counter.fetch_add(1);
  return out.str();
}
}  // namespace

using ros2_cuda_ipc_core::cuda::GpuBufferController;
using ros2_cuda_ipc_core::cuda::PublishSlot;

static_assert(!std::is_copy_constructible_v<PublishSlot>);
static_assert(!std::is_copy_assignable_v<PublishSlot>);
static_assert(std::is_nothrow_move_constructible_v<PublishSlot>);
static_assert(std::is_nothrow_move_assignable_v<PublishSlot>);

class GpuBufferControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    int count = 0;
    if (cudaGetDeviceCount(&count) != cudaSuccess || count == 0 ||
        cudaSetDevice(0) != cudaSuccess) {
      GTEST_SKIP() << "CUDA device not available";
    }
    shm_name_ = unique_name();
  }
  void TearDown() override {
    if (!shm_name_.empty()) {
      ::shm_unlink(shm_name_.c_str());
    }
  }
  GpuBufferController make_controller(
      std::chrono::milliseconds pending_ttl = std::chrono::milliseconds(100)) {
    return GpuBufferController(
        {shm_name_, 1, 1024, 0, pending_ttl,
         ros2_cuda_ipc_core::MemoryBackendKind::CUDA_IPC},
        rclcpp::get_logger("GpuBufferControllerTest"));
  }
  std::string shm_name_;
};

TEST_F(GpuBufferControllerTest, DescriptorIsGatedByReadyRecording) {
  auto controller = make_controller();
  ASSERT_TRUE(controller.initialise());
  auto slot = controller.acquire_for_publish(1);
  ASSERT_TRUE(slot.has_value());
  EXPECT_EQ(slot->descriptor(), std::nullopt);
  ASSERT_EQ(slot->record_ready(nullptr), cudaSuccess);
  auto descriptor = slot->descriptor();
  ASSERT_TRUE(descriptor.has_value());
  EXPECT_EQ(descriptor->slot_id, 0u);
  EXPECT_EQ(descriptor->lease_shm_name, shm_name_);
  EXPECT_EQ(descriptor->byte_size, 1024u);
  EXPECT_NE(slot->record_ready(nullptr), cudaSuccess);
  slot->commit_publish();
  slot->commit_publish();
  EXPECT_FALSE(slot->valid());
}

TEST_F(GpuBufferControllerTest, UncommittedDestructionCancelsReservation) {
  auto controller = make_controller();
  ASSERT_TRUE(controller.initialise());
  {
    auto slot = controller.acquire_for_publish(1);
    ASSERT_TRUE(slot.has_value());
  }
  EXPECT_TRUE(controller.acquire_for_publish(1).has_value());
}

TEST_F(GpuBufferControllerTest, CommittedDestructionKeepsPending) {
  auto controller = make_controller();
  ASSERT_TRUE(controller.initialise());
  {
    auto slot = controller.acquire_for_publish(1);
    ASSERT_TRUE(slot.has_value());
    ASSERT_EQ(slot->record_ready(nullptr), cudaSuccess);
    slot->commit_publish();
  }
  EXPECT_FALSE(controller.acquire_for_publish(1).has_value());
}

TEST_F(GpuBufferControllerTest, MovedFromSlotIsInert) {
  auto controller = make_controller();
  ASSERT_TRUE(controller.initialise());
  auto source = controller.acquire_for_publish(1);
  ASSERT_TRUE(source.has_value());
  PublishSlot destination(std::move(*source));
  EXPECT_FALSE(source->valid());
  EXPECT_EQ(source->device_ptr(), nullptr);
  EXPECT_TRUE(destination.valid());
  destination.cancel();
  destination.cancel();
  EXPECT_FALSE(destination.valid());
  EXPECT_TRUE(controller.acquire_for_publish(1).has_value());
}

TEST_F(GpuBufferControllerTest, ResetDoesNotPreventReservationCancellation) {
  auto controller = make_controller();
  ASSERT_TRUE(controller.initialise());
  auto slot = controller.acquire_for_publish(1);
  ASSERT_TRUE(slot.has_value());
  const auto pending_before =
      ros2_cuda_ipc_core::LeaseHandle::current_pending(shm_name_, 0);
  ASSERT_TRUE(pending_before.has_value());
  ASSERT_EQ(*pending_before, 1u);

  controller.reset();
  slot.reset();

  const auto pending =
      ros2_cuda_ipc_core::LeaseHandle::current_pending(shm_name_, 0);
  ASSERT_TRUE(pending.has_value());
  EXPECT_EQ(*pending, 0u);
}

TEST_F(GpuBufferControllerTest, AcquireAutomaticallyReclaimsExpiredPending) {
  auto controller = make_controller(std::chrono::milliseconds(1));
  ASSERT_TRUE(controller.initialise());
  {
    auto slot = controller.acquire_for_publish(1);
    ASSERT_TRUE(slot.has_value());
    ASSERT_EQ(slot->record_ready(nullptr), cudaSuccess);
    slot->commit_publish();
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_TRUE(controller.acquire_for_publish(1).has_value());
}
