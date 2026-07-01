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
      pool_({config_.shm_name, config_.slot_count, config_.pending_ttl,
             config_.backend},
            logger_.get_child("GpuLeasePool")) {
  if (config_.width == 0 || config_.height == 0) {
    throw std::runtime_error("width and height must be greater than zero");
  }
  if (config_.slot_count == 0) {
    throw std::runtime_error("slot_count must be greater than zero");
  }

  throw_on_cuda_error(cudaSetDevice(config_.device_index), "cudaSetDevice");

  frame_size_bytes_ =
      static_cast<uint64_t>(config_.width) * config_.height * kBytesPerPixel;

  throw_on_cuda_error(
      cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking),
      "cudaStreamCreateWithFlags");

  if (!pool_.initialise(frame_size_bytes_, config_.device_index)) {
    throw std::runtime_error("Failed to initialise GPU lease pool");
  }
}

ImagePublisherHelper::~ImagePublisherHelper() {
  pool_.reset();
  if (stream_ != nullptr) {
    const cudaError_t err = cudaStreamDestroy(stream_);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(logger_, "cudaStreamDestroy failed: %s",
                   ros2_cuda_ipc_core::cuda::cuda_error_to_string(err).c_str());
    }
    stream_ = nullptr;
  }
}

std::optional<ros2_cuda_ipc_core::view::ImageView>
ImagePublisherHelper::produce(std::size_t subscriber_count,
                              uint64_t frame_index) {
  NvtxScopedRange produce_range("ImagePublisherHelper::produce");
  if (!pool_.is_initialised()) {
    return std::nullopt;
  }

  pool_.reclaim_stale_pending();

  ros2_cuda_ipc_core::cuda::GpuLeasePool::Slot* slot = nullptr;
  {
    NvtxScopedRange acquire_range("ImagePublisherHelper::acquire_slot");
    auto slot_ptr = pool_.acquire(subscriber_count);
    if (!slot_ptr.has_value() || *slot_ptr == nullptr) {
      RCLCPP_WARN(logger_, "No available GPU slots (all leases in use)");
      return std::nullopt;
    }
    slot = *slot_ptr;
  }

  cudaError_t err = cudaSuccess;
  {
    NvtxScopedRange kernel_range(
        "ImagePublisherHelper::generate_rgba_pattern_kernel");
    err = launch_generate_rgba_pattern_kernel(
        static_cast<uint8_t*>(slot->device_ptr),
        static_cast<int>(config_.width), static_cast<int>(config_.height),
        config_.width * kBytesPerPixel, frame_index, stream_);
  }
  if (err != cudaSuccess) {
    RCLCPP_ERROR(logger_, "launch_generate_rgba_pattern_kernel failed: %s",
                 ros2_cuda_ipc_core::cuda::cuda_error_to_string(err).c_str());
    return std::nullopt;
  }

  {
    NvtxScopedRange event_record_range("ImagePublisherHelper::cudaEventRecord");
    err = cudaEventRecord(slot->event, stream_);
  }
  if (err != cudaSuccess) {
    RCLCPP_ERROR(logger_, "cudaEventRecord failed for slot %u: %s", slot->index,
                 ros2_cuda_ipc_core::cuda::cuda_error_to_string(err).c_str());
    return std::nullopt;
  }

  ros2_cuda_ipc_core::view::ImageView view;
  view.core.dev_ptr = slot->device_ptr;
  view.core.ready_evt = slot->event;
  view.core.device_id = config_.device_index;
  view.core.byte_size = frame_size_bytes_;
  view.core.slot_id = slot->index;
  view.core.generation = slot->generation;
  view.core.shm_name = config_.shm_name;
  view.core.set_ipc_handles(slot->backend, slot->mem_handle.data(),
                            slot->mem_handle.size(), slot->event_handle);
  view.dtype = ros2_cuda_ipc_core::view::DType::U8;
  view.shape = {config_.height, config_.width, kDefaultChannels};
  view.strides = {config_.width * kBytesPerPixel, kDefaultChannels, 1};
  view.encoding = config_.encoding;

  return view;
}

}  // namespace multi_process_image_fanout
