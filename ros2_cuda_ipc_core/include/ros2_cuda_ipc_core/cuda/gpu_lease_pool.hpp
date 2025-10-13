#pragma once

#include <cuda_runtime_api.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/logger.hpp"

namespace ros2_cuda_ipc_core::cuda {

class GpuLeasePool {
 public:
  struct Config {
    std::string shm_name;
    std::size_t slot_count = 0;
    std::chrono::milliseconds pending_ttl{0};
  };

  struct Slot {
    uint32_t index = 0;
    void *device_ptr = nullptr;
    cudaEvent_t event = nullptr;
    cudaIpcMemHandle_t mem_handle{};
    cudaIpcEventHandle_t event_handle{};
    uint32_t generation = 0;
    std::chrono::steady_clock::time_point pending_deadline{};
  };

  explicit GpuLeasePool(Config config, rclcpp::Logger logger);
  ~GpuLeasePool();

  GpuLeasePool(const GpuLeasePool &) = delete;
  GpuLeasePool &operator=(const GpuLeasePool &) = delete;
  GpuLeasePool(GpuLeasePool &&) = default;
  GpuLeasePool &operator=(GpuLeasePool &&) = default;

  bool initialise(uint64_t frame_size_bytes, int device_index);
  void reset() noexcept;

  bool is_initialised() const noexcept { return initialised_; }
  bool matches(uint64_t frame_size_bytes, int device_index) const noexcept;

  std::optional<Slot *> acquire(std::size_t subscriber_count);
  void reclaim_stale_pending();
  bool cancel_pending(Slot &slot);

  uint64_t frame_size_bytes() const noexcept { return frame_size_bytes_; }
  int device_index() const noexcept { return device_index_; }

 private:
  bool allocate_slots();
  void destroy_slots() noexcept;

  Config config_;
  std::vector<Slot> slots_;
  uint64_t frame_size_bytes_ = 0;
  int device_index_ = -1;
  bool initialised_ = false;
  rclcpp::Logger logger_;
};

}  // namespace ros2_cuda_ipc_core::cuda
