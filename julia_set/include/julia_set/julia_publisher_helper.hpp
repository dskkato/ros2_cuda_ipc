#pragma once

#include <cuda_runtime_api.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "ros2_cuda_ipc_core/image_view.hpp"

namespace julia_set {

class JuliaPublisherHelper {
 public:
  struct Config {
    std::string shm_name = "/ros2_cuda_ipc_julia";
    uint32_t width = 1280;
    uint32_t height = 720;
    uint32_t channels = 3;
    ros2_cuda_ipc_core::DType dtype = ros2_cuda_ipc_core::DType::U8;
    std::string encoding = "rgb8";
    std::size_t slot_count = 4;
    int device_index = 0;
    std::chrono::milliseconds pending_ttl{300};
    float zoom = 1.0f;
    float offset_x = 0.0f;
    float offset_y = 0.0f;
    float constant_real = -0.8f;
    float constant_imag = 0.156f;
    uint32_t max_iterations = 300;
  };

  explicit JuliaPublisherHelper(const Config &config);
  ~JuliaPublisherHelper();

  JuliaPublisherHelper(const JuliaPublisherHelper &) = delete;
  JuliaPublisherHelper &operator=(const JuliaPublisherHelper &) = delete;
  JuliaPublisherHelper(JuliaPublisherHelper &&) = delete;
  JuliaPublisherHelper &operator=(JuliaPublisherHelper &&) = delete;

  std::optional<ros2_cuda_ipc_core::ImageView> produce(
      std::size_t subscriber_count, float time_phase = 0.0f);

  uint64_t frame_size_bytes() const noexcept { return frame_size_bytes_; }

 private:
  struct Slot {
    uint32_t index = 0;
    void *device_ptr = nullptr;
    cudaEvent_t event = nullptr;
    cudaIpcMemHandle_t mem_handle{};
    cudaIpcEventHandle_t event_handle{};
    uint32_t generation = 0;
    std::chrono::steady_clock::time_point pending_deadline{};
  };

  Config config_;
  std::vector<Slot> slots_;
  cudaStream_t stream_ = nullptr;
  uint64_t frame_size_bytes_ = 0;

  void initialise_shm();
  void allocate_slots();
  void destroy_slots() noexcept;
  void reclaim_stale_pending();
  static bool deadline_reached(
      const std::chrono::steady_clock::time_point &deadline,
      const std::chrono::steady_clock::time_point &now) {
    return deadline.time_since_epoch().count() != 0 && now >= deadline;
  }
};

}  // namespace julia_set
