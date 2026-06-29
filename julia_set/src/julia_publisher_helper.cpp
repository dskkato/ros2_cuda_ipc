// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: Apache-2.0

// Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "julia_set/julia_publisher_helper.hpp"

#include <cuda_runtime_api.h>

#include <cmath>
#include <stdexcept>

#include "julia_set/cuda/julia_kernel.hpp"
#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/cuda/cuda_util.hpp"
#include "ros2_cuda_ipc_core/nvtx_scoped_range.hpp"

namespace julia_set {

using ros2_cuda_ipc_core::NvtxScopedRange;
using ros2_cuda_ipc_core::cuda::cuda_error_to_string;
namespace {

uint32_t dtype_bytes(ros2_cuda_ipc_core::DType dtype) {
  switch (dtype) {
    case ros2_cuda_ipc_core::DType::U8:
      return 1;
    case ros2_cuda_ipc_core::DType::U16:
    case ros2_cuda_ipc_core::DType::F16:
    case ros2_cuda_ipc_core::DType::S16:
      return 2;
    case ros2_cuda_ipc_core::DType::F32:
    case ros2_cuda_ipc_core::DType::S32:
    case ros2_cuda_ipc_core::DType::U32:
      return 4;
    case ros2_cuda_ipc_core::DType::F64:
      return 8;
  }
  return 1;
}

}  // namespace

JuliaPublisherHelper::JuliaPublisherHelper(const Config& config,
                                           const rclcpp::Logger& logger)
    : config_(config),
      logger_(logger),
      pool_({config_.shm_name, config_.slot_count, config_.pending_ttl,
             config_.backend},
            logger_.get_child("GpuLeasePool")) {
  if (config_.slot_count == 0) {
    throw std::runtime_error("slot_count must be greater than zero");
  }

  cudaError_t err = cudaSetDevice(config_.device_index);
  if (err != cudaSuccess) {
    throw std::runtime_error("cudaSetDevice failed: " +
                             cuda_error_to_string(err));
  }

  frame_size_bytes_ = static_cast<uint64_t>(config_.width) * config_.height *
                      config_.channels * dtype_bytes(config_.dtype);
  err = cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking);
  if (err != cudaSuccess) {
    throw std::runtime_error("cudaStreamCreateWithFlags failed: " +
                             cuda_error_to_string(err));
  }

  if (!pool_.initialise(frame_size_bytes_, config_.device_index)) {
    throw std::runtime_error("Failed to initialise GPU lease pool");
  }
}

JuliaPublisherHelper::~JuliaPublisherHelper() {
  pool_.reset();
  if (stream_) {
    const cudaError_t stream_err = cudaStreamDestroy(stream_);
    if (stream_err != cudaSuccess) {
      RCLCPP_ERROR(logger_, "cudaStreamDestroy failed: %s",
                   cuda_error_to_string(stream_err).c_str());
    }
    stream_ = nullptr;
  }
}

std::optional<ros2_cuda_ipc_core::ImageView> JuliaPublisherHelper::produce(
    std::size_t subscriber_count, float time_phase) {
  NvtxScopedRange produce_range("JuliaPublisherHelper::produce");
  if (!pool_.is_initialised()) {
    return std::nullopt;
  }

  if (config_.dtype != ros2_cuda_ipc_core::DType::U8) {
    RCLCPP_ERROR(logger_, "Unsupported dtype for Julia set rendering");
    return std::nullopt;
  }

  if (config_.channels != 1) {
    RCLCPP_ERROR(logger_,
                 "JuliaPublisherHelper expects single-channel output (got %u)",
                 config_.channels);
    return std::nullopt;
  }

  pool_.reclaim_stale_pending();

  auto slot_ptr = pool_.acquire(subscriber_count);
  if (!slot_ptr.has_value() || *slot_ptr == nullptr) {
    RCLCPP_WARN(logger_, "No available GPU slots (all leases in use)");
    return std::nullopt;
  }

  auto* slot = *slot_ptr;

  const float zoom = config_.zoom + 0.5f * std::sin(time_phase);
  const float offset_x = config_.offset_x + 0.2f * std::cos(time_phase * 0.5f);
  const float offset_y = config_.offset_y + 0.2f * std::sin(time_phase * 0.5f);
  const float c_real =
      config_.constant_real + 0.1f * std::cos(time_phase * 0.3f);
  const float c_imag =
      config_.constant_imag + 0.1f * std::sin(time_phase * 0.7f);

  cudaError_t err = cudaSuccess;
  {
    NvtxScopedRange launch_range("JuliaPublisherHelper::launch_julia_kernel");
    err = launch_julia_kernel(static_cast<uint8_t*>(slot->device_ptr),
                              config_.width, config_.height, zoom, offset_x,
                              offset_y, c_real, c_imag, config_.max_iterations,
                              stream_);
  }
  if (err != cudaSuccess) {
    RCLCPP_ERROR(logger_, "launch_julia_kernel failed for slot %u: %s",
                 slot->index, cuda_error_to_string(err).c_str());
    return std::nullopt;
  }

  {
    NvtxScopedRange event_record_range("JuliaPublisherHelper::cudaEventRecord");
    err = cudaEventRecord(slot->event, stream_);
  }
  if (err != cudaSuccess) {
    RCLCPP_ERROR(logger_, "cudaEventRecord failed for slot %u: %s", slot->index,
                 cuda_error_to_string(err).c_str());
    return std::nullopt;
  }

  ros2_cuda_ipc_core::ImageView view;
  view.core.dev_ptr = slot->device_ptr;
  view.core.ready_evt = slot->event;
  view.core.device_id = config_.device_index;
  view.core.byte_size = frame_size_bytes_;
  view.core.slot_id = slot->index;
  view.core.generation = slot->generation;
  view.core.shm_name = config_.shm_name;
  view.core.set_ipc_handles(slot->backend, slot->mem_handle.data(),
                            slot->mem_handle.size(), slot->event_handle);
  view.dtype = config_.dtype;
  view.shape = {config_.height, config_.width, config_.channels};
  const uint64_t elem_size = dtype_bytes(config_.dtype);
  view.strides = {
      static_cast<uint64_t>(config_.width) * config_.channels * elem_size,
      static_cast<uint64_t>(config_.channels) * elem_size, elem_size};
  if (!config_.encoding.empty()) {
    view.encoding = config_.encoding;
  } else {
    view.encoding.clear();
  }

  return view;
}

}  // namespace julia_set
