// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <fcntl.h>
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
  GpuBufferManager make_manager() {
    return GpuBufferManager(
        {shm_name_, 1, 1024, 0,
         ros2_cuda_ipc_core::transport::MemoryBackendKind::CUDA_IPC});
  }
  std::string shm_name_;
};

TEST_F(GpuBufferManagerTest, DescriptorIsGatedByReadyRecording) {
  auto manager = make_manager();
  ASSERT_TRUE(manager.initialise());
  const auto instance_id = manager.publisher_instance_id();
  auto slot = manager.acquire_for_publish();
  ASSERT_TRUE(slot.has_value());
  EXPECT_EQ(slot->descriptor(), std::nullopt);
  ASSERT_TRUE(slot->record_ready(nullptr));
  auto descriptor = slot->descriptor();
  ASSERT_TRUE(descriptor.has_value());
  EXPECT_EQ(descriptor->slot_id, 0u);
  EXPECT_NE(descriptor->lease_shm_name, shm_name_);
  EXPECT_EQ(descriptor->lease_shm_name, manager.shm_name());
  EXPECT_EQ(descriptor->publisher_instance_id, instance_id);
  EXPECT_EQ(descriptor->byte_size, 1024u);
  EXPECT_FALSE(slot->record_ready(nullptr));
  slot->commit_publish();
  slot->commit_publish();
  EXPECT_FALSE(slot->valid());
}

TEST_F(GpuBufferManagerTest, RuntimeCreatedStreamCanRecordReady) {
  auto manager = make_manager();
  ASSERT_TRUE(manager.initialise());
  auto slot = manager.acquire_for_publish();
  ASSERT_TRUE(slot.has_value());

  cudaStream_t stream = nullptr;
  ASSERT_EQ(cudaStreamCreate(&stream), cudaSuccess);
  const auto result = slot->record_ready(stream);
  ASSERT_TRUE(result) << result.error().to_string();
  ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
  ASSERT_EQ(cudaStreamDestroy(stream), cudaSuccess);
}

TEST_F(GpuBufferManagerTest, DriverCreatedStreamCanRecordReady) {
  auto manager = make_manager();
  ASSERT_TRUE(manager.initialise());
  auto slot = manager.acquire_for_publish();
  ASSERT_TRUE(slot.has_value());

  CUstream stream = nullptr;
  ASSERT_EQ(cuStreamCreate(&stream, CU_STREAM_DEFAULT), CUDA_SUCCESS);
  const auto result = slot->record_ready(stream);
  ASSERT_TRUE(result) << result.error().to_string();
  ASSERT_EQ(cuStreamSynchronize(stream), CUDA_SUCCESS);
  ASSERT_EQ(cuStreamDestroy(stream), CUDA_SUCCESS);
}

TEST_F(GpuBufferManagerTest, UncommittedDestructionCancelsReservation) {
  auto manager = make_manager();
  ASSERT_TRUE(manager.initialise());
  {
    auto slot = manager.acquire_for_publish();
    ASSERT_TRUE(slot.has_value());
  }
  EXPECT_TRUE(manager.acquire_for_publish().has_value());
}

TEST_F(GpuBufferManagerTest, CommittedDestructionKeepsGracePeriod) {
  auto manager = make_manager();
  ASSERT_TRUE(manager.initialise());
  {
    auto slot = manager.acquire_for_publish();
    ASSERT_TRUE(slot.has_value());
    ASSERT_TRUE(slot->record_ready(nullptr));
    slot->commit_publish();
  }
  EXPECT_FALSE(manager.acquire_for_publish().has_value());
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  EXPECT_TRUE(manager.acquire_for_publish().has_value());
}

TEST_F(GpuBufferManagerTest, FailedCommitDoesNotMarkSlotCommitted) {
  auto manager = make_manager();
  ASSERT_TRUE(manager.initialise());
  auto slot = manager.acquire_for_publish();
  ASSERT_TRUE(slot.has_value());
  ASSERT_TRUE(slot->record_ready(nullptr));

  auto mapping = ros2_cuda_ipc_core::lease::LeaseMapping::attach(
      manager.shm_name(), manager.publisher_instance_id());
  ASSERT_TRUE(mapping);
  const auto generation =
      ros2_cuda_ipc_core::lease::LeaseHandle::current_generation(mapping, 0);
  ASSERT_TRUE(generation.has_value());

  mapping->slot(0)->generation.store(*generation + 1,
                                     std::memory_order_release);
  EXPECT_FALSE(slot->commit_publish());
  EXPECT_TRUE(slot->valid());

  // Restore the reservation generation so its cancellation can release the
  // temporary Publisher reference after the failed-commit check.
  mapping->slot(0)->generation.store(*generation, std::memory_order_release);
  slot->cancel();
  EXPECT_FALSE(slot->valid());
  EXPECT_TRUE(manager.acquire_for_publish().has_value());
}

TEST_F(GpuBufferManagerTest, MovedFromSlotIsInert) {
  auto manager = make_manager();
  ASSERT_TRUE(manager.initialise());
  auto source = manager.acquire_for_publish();
  ASSERT_TRUE(source.has_value());
  PublishSlot destination(std::move(*source));
  EXPECT_FALSE(source->valid());
  EXPECT_EQ(source->device_ptr(), nullptr);
  EXPECT_TRUE(destination.valid());
  destination.cancel();
  destination.cancel();
  EXPECT_FALSE(destination.valid());
  EXPECT_TRUE(manager.acquire_for_publish().has_value());
}

TEST_F(GpuBufferManagerTest, ResetDoesNotPreventReservationCancellation) {
  auto manager = make_manager();
  ASSERT_TRUE(manager.initialise());
  const std::string actual_name = manager.shm_name();
  const auto instance_id = manager.publisher_instance_id();
  auto slot = manager.acquire_for_publish();
  ASSERT_TRUE(slot.has_value());
  auto mapping =
      ros2_cuda_ipc_core::lease::LeaseMapping::attach(actual_name, instance_id);
  ASSERT_TRUE(mapping);
  const auto ref_before =
      ros2_cuda_ipc_core::lease::LeaseHandle::current_refcount(mapping, 0);
  ASSERT_TRUE(ref_before.has_value());
  ASSERT_EQ(*ref_before, 1u);

  manager.reset();
  slot.reset();

  const auto ref_after =
      ros2_cuda_ipc_core::lease::LeaseHandle::current_refcount(mapping, 0);
  ASSERT_TRUE(ref_after.has_value());
  EXPECT_EQ(*ref_after, 0u);

  // The name itself is nevertheless gone immediately after manager reset.
  const int fd = ::shm_open(actual_name.c_str(), O_RDWR, 0660);
  EXPECT_EQ(fd, -1);
  if (fd != -1) {
    ::close(fd);
  }
}

TEST_F(GpuBufferManagerTest, AcquireAfterGracePeriod) {
  auto manager = make_manager();
  ASSERT_TRUE(manager.initialise());
  {
    auto slot = manager.acquire_for_publish();
    ASSERT_TRUE(slot.has_value());
    ASSERT_TRUE(slot->record_ready(nullptr));
    slot->commit_publish();
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(105));
  EXPECT_TRUE(manager.acquire_for_publish().has_value());
}
