// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "multi_process_image_fanout/image_publisher_helper.hpp"

#include <cuda_runtime_api.h>

#include <stdexcept>

#include "multi_process_image_fanout/cuda_checks.hpp"
#include "multi_process_image_fanout/kernels.hpp"
#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/cuda/nvtx_scoped_range.hpp"

namespace multi_process_image_fanout {

using ros2_cuda_ipc_core::cuda::NvtxScopedRange;

namespace {

constexpr uint64_t kBytesPerPixel = 4;

}  // namespace

ImagePublisherHelper::ImagePublisherHelper(const Config& config,
                                           const rclcpp::Logger& logger)
    : config_(config),
      logger_(logger),
      controller_({config_.shm_name, config_.slot_count,
                   static_cast<uint64_t>(config_.width) * config_.height *
                       kBytesPerPixel,
                   config_.device_index, config_.pending_ttl, config_.backend},
                  logger_.get_child("GpuBufferController")) {
  if (config_.width == 0 || config_.height == 0) {
    throw std::runtime_error("width and height must be greater than zero");
  }
  if (config_.slot_count == 0) {
    throw std::runtime_error("slot_count must be greater than zero");
  }

  throw_on_cuda_error(cudaSetDevice(config_.device_index), "cudaSetDevice");

  // Size the lease pool buffers and create the single non-blocking CUDA stream
  // used by the publisher-side kernel.
  frame_size_bytes_ =
      static_cast<uint64_t>(config_.width) * config_.height * kBytesPerPixel;

  throw_on_cuda_error(
      cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking),
      "cudaStreamCreateWithFlags");

  if (!controller_.initialise()) {
    throw std::runtime_error("Failed to initialise GPU buffer controller");
  }
}

ImagePublisherHelper::~ImagePublisherHelper() {
  controller_.reset();
  if (stream_ != nullptr) {
    const cudaError_t err = cudaStreamDestroy(stream_);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(logger_, "cudaStreamDestroy failed: %s",
                   ros2_cuda_ipc_core::cuda::cuda_error_to_string(err).c_str());
    }
    stream_ = nullptr;
  }
}

std::optional<ros2_cuda_ipc_core::cuda::PublishSlot>
ImagePublisherHelper::produce(std::size_t subscriber_count,
                              uint64_t frame_index,
                              ros2_cuda_ipc_core::view::ImageView& output) {
  NvtxScopedRange produce_range("ImagePublisherHelper::produce");
  if (!controller_.is_initialised()) {
    return std::nullopt;
  }

  // Old pending leases are reclaimed before trying to reuse slots.
  controller_.reclaim_stale_pending();

  std::optional<ros2_cuda_ipc_core::cuda::PublishSlot> slot;
  {
    // Expected consumers come from the ROS subscription count.
    NvtxScopedRange acquire_range("ImagePublisherHelper::acquire_slot");
    slot = controller_.acquire_for_publish(
        static_cast<uint32_t>(subscriber_count));
    if (!slot) {
      RCLCPP_WARN(logger_, "No available GPU slots (all leases in use)");
      return std::nullopt;
    }
  }

  cudaError_t err = cudaSuccess;
  {
    // Generate the frame directly in the leased GPU buffer.
    NvtxScopedRange kernel_range(
        "ImagePublisherHelper::generate_rgba_pattern_kernel");
    err = launch_generate_rgba_pattern_kernel(
        static_cast<uint8_t*>(slot->device_ptr()),
        static_cast<int>(config_.width), static_cast<int>(config_.height),
        config_.width * kBytesPerPixel, frame_index, stream_);
  }
  if (err != cudaSuccess) {
    RCLCPP_ERROR(logger_, "launch_generate_rgba_pattern_kernel failed: %s",
                 ros2_cuda_ipc_core::cuda::cuda_error_to_string(err).c_str());
    return std::nullopt;
  }

  {
    // Consumers wait on this event before reading the slot.
    NvtxScopedRange event_record_range("ImagePublisherHelper::cudaEventRecord");
    err = slot->record_ready(stream_);
  }
  if (err != cudaSuccess) {
    RCLCPP_ERROR(logger_, "cudaEventRecord failed: %s",
                 ros2_cuda_ipc_core::cuda::cuda_error_to_string(err).c_str());
    return std::nullopt;
  }

  const auto descriptor = slot->descriptor();
  if (!descriptor) {
    RCLCPP_ERROR(logger_, "Failed to create GPU buffer descriptor");
    return std::nullopt;
  }

  // Publishable metadata for the GPU buffer and IPC handles. The pixel payload
  // stays on the device.
  output = {};
  output.core = descriptor->to_publisher_view(slot->device_ptr());
  output.dtype = ros2_cuda_ipc_core::view::DType::U8;
  output.shape = {config_.height, config_.width, kDefaultChannels};
  output.strides = {config_.width * kBytesPerPixel, kDefaultChannels, 1};
  output.encoding = config_.encoding;

  return slot;
}

}  // namespace multi_process_image_fanout
