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
#include "ros2_cuda_ipc_core/cuda/gpu_buffer_controller.hpp"
#include "ros2_cuda_ipc_core/memory_types.hpp"
#include "ros2_cuda_ipc_core/view/image_view.hpp"

namespace multi_process_image_fanout {

// Owns the CUDA and lease-pool state required to publish GPU-only ImageView
// frames. Keeping this logic outside the ROS node makes the example easier to
// read: the node handles parameters, timers, and publishing, while this helper
// shows the publisher-side ros2_cuda_ipc workflow in one place.
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

  // The returned PublishSlot must remain alive until the caller either
  // publishes output and commits it, or abandons the attempt and lets RAII
  // cancellation run.
  std::optional<ros2_cuda_ipc_core::cuda::PublishSlot> produce(
      std::size_t subscriber_count, uint64_t frame_index,
      ros2_cuda_ipc_core::view::ImageView& output);

 private:
  Config config_;
  rclcpp::Logger logger_;
  ros2_cuda_ipc_core::cuda::GpuBufferController controller_;
  cudaStream_t stream_ = nullptr;
  uint64_t frame_size_bytes_ = 0;
};

}  // namespace multi_process_image_fanout
