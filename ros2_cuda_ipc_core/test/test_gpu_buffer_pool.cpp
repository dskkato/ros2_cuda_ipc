// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/publisher/gpu_buffer_pool.hpp"

namespace {

struct CleanupObservation {
  int destroy_calls = 0;
  int populated_slots_at_destroy = 0;
};

class PartiallyFailingBackend
    : public ros2_cuda_ipc_core::backend::MemoryBackend {
 public:
  explicit PartiallyFailingBackend(
      std::shared_ptr<CleanupObservation> observation)
      : observation_(std::move(observation)) {}

  bool allocate(uint64_t, int,
                std::vector<ros2_cuda_ipc_core::backend::SlotResources>& slots,
                rclcpp::Logger) override {
    slots[0].device_ptr = reinterpret_cast<void*>(0x1);
    return false;
  }

  void destroy(std::vector<ros2_cuda_ipc_core::backend::SlotResources>& slots,
               rclcpp::Logger) noexcept override {
    ++observation_->destroy_calls;
    for (auto& slot : slots) {
      if (slot.device_ptr != nullptr) {
        ++observation_->populated_slots_at_destroy;
        slot.device_ptr = nullptr;
      }
    }
  }

 private:
  std::shared_ptr<CleanupObservation> observation_;
};

}  // namespace

TEST(GpuBufferPoolTest, PartialBackendFailureIsRolledBack) {
  int device_count = 0;
  if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count == 0) {
    GTEST_SKIP() << "CUDA device not available";
  }
  auto observation = std::make_shared<CleanupObservation>();
  ros2_cuda_ipc_core::publisher::GpuBufferPool pool(
      2, ros2_cuda_ipc_core::transport::MemoryBackendKind::CUDA_IPC,
      rclcpp::get_logger("GpuBufferPoolTest"),
      std::make_unique<PartiallyFailingBackend>(observation));
  EXPECT_FALSE(pool.initialise(1024, 0));
  EXPECT_FALSE(pool.is_initialised());
  EXPECT_EQ(observation->destroy_calls, 1);
  EXPECT_EQ(observation->populated_slots_at_destroy, 1);
  EXPECT_EQ(pool.size(), 0u);
}
