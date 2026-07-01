// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda_runtime_api.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "rclcpp/logger.hpp"
#include "ros2_cuda_ipc_core/cuda/gpu_lease_pool.hpp"
#include "ros2_cuda_ipc_core/memory_types.hpp"
#include "ros2_cuda_ipc_core/view/image_view.hpp"

namespace multi_process_image_fanout {

class ImagePublisherHelper {
 public:
  struct Config {
    std::string shm_name = "/ros2_cuda_ipc_fanout";
    uint32_t width = 1920;
    uint32_t height = 1080;
    std::size_t slot_count = 4;
    int device_index = 0;
    std::chrono::milliseconds pending_ttl{300};
    ros2_cuda_ipc_core::MemoryBackendKind backend =
        ros2_cuda_ipc_core::MemoryBackendKind::CUDA_IPC;
    std::string encoding = "rgba8";
  };

  ImagePublisherHelper(const Config& config, const rclcpp::Logger& logger);
  ~ImagePublisherHelper();

  ImagePublisherHelper(const ImagePublisherHelper&) = delete;
  ImagePublisherHelper& operator=(const ImagePublisherHelper&) = delete;
  ImagePublisherHelper(ImagePublisherHelper&&) = delete;
  ImagePublisherHelper& operator=(ImagePublisherHelper&&) = delete;

  std::optional<ros2_cuda_ipc_core::view::ImageView> produce(
      std::size_t subscriber_count, uint64_t frame_index);

  uint64_t frame_size_bytes() const noexcept { return frame_size_bytes_; }

 private:
  Config config_;
  rclcpp::Logger logger_;
  ros2_cuda_ipc_core::cuda::GpuLeasePool pool_;
  cudaStream_t stream_ = nullptr;
  uint64_t frame_size_bytes_ = 0;
};

}  // namespace multi_process_image_fanout
