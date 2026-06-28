#pragma once

#include <cuda_runtime_api.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "rclcpp/logger.hpp"
#include "ros2_cuda_ipc_core/cuda/gpu_lease_pool.hpp"
#include "ros2_cuda_ipc_core/image_view.hpp"
#include "ros2_cuda_ipc_core/memory_types.hpp"

namespace julia_set {

class JuliaPublisherHelper {
 public:
  struct Config {
    std::string shm_name = "/ros2_cuda_ipc_julia";
    uint32_t width = 1280;
    uint32_t height = 720;
    uint32_t channels = 1;
    ros2_cuda_ipc_core::DType dtype = ros2_cuda_ipc_core::DType::U8;
    std::string encoding = "mono8";
    std::size_t slot_count = 4;
    int device_index = 0;
    std::chrono::milliseconds pending_ttl{300};
    float zoom = 1.0f;
    float offset_x = 0.0f;
    float offset_y = 0.0f;
    float constant_real = -0.8f;
    float constant_imag = 0.156f;
    uint32_t max_iterations = 300;
    ros2_cuda_ipc_core::MemoryBackendKind backend =
        ros2_cuda_ipc_core::MemoryBackendKind::CUDA_IPC;
  };

  JuliaPublisherHelper(const Config& config, const rclcpp::Logger& logger);
  ~JuliaPublisherHelper();

  JuliaPublisherHelper(const JuliaPublisherHelper&) = delete;
  JuliaPublisherHelper& operator=(const JuliaPublisherHelper&) = delete;
  JuliaPublisherHelper(JuliaPublisherHelper&&) = delete;
  JuliaPublisherHelper& operator=(JuliaPublisherHelper&&) = delete;

  std::optional<ros2_cuda_ipc_core::ImageView> produce(
      std::size_t subscriber_count, float time_phase = 0.0f);

  uint64_t frame_size_bytes() const noexcept { return frame_size_bytes_; }

 private:
  Config config_;
  rclcpp::Logger logger_;
  ros2_cuda_ipc_core::cuda::GpuLeasePool pool_;
  cudaStream_t stream_ = nullptr;
  uint64_t frame_size_bytes_ = 0;
};

}  // namespace julia_set
