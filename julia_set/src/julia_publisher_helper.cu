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

#include <cuda_runtime_api.h>

#include <cmath>
#include <stdexcept>

#include "julia_set/julia_publisher_helper.hpp"
#include "julia_set/nvtx_scoped_range.hpp"
#include "rclcpp/logging.hpp"

namespace julia_set {
namespace {

__global__ void julia_kernel(uint8_t *data, uint32_t width, uint32_t height,
                             uint32_t channels, float zoom, float offset_x,
                             float offset_y, float c_real, float c_imag,
                             uint32_t max_iterations) {
  const uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
  const uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= width || y >= height) {
    return;
  }

  const float jx =
      zoom * (static_cast<float>(x) / static_cast<float>(width) - 0.5f) +
      offset_x;
  const float jy =
      zoom * (static_cast<float>(y) / static_cast<float>(height) - 0.5f) +
      offset_y;

  float zx = jx;
  float zy = jy;
  uint32_t iteration = 0;
  while (zx * zx + zy * zy < 4.0f && iteration < max_iterations) {
    const float temp = zx * zx - zy * zy + c_real;
    zy = 2.0f * zx * zy + c_imag;
    zx = temp;
    ++iteration;
  }

  float t = static_cast<float>(iteration) / static_cast<float>(max_iterations);
  t = fminf(1.0f, fmaxf(0.0f, t));
  const uint8_t r =
      static_cast<uint8_t>(9.0f * (1.0f - t) * t * t * t * 255.0f);
  const uint8_t g =
      static_cast<uint8_t>(15.0f * (1.0f - t) * (1.0f - t) * t * t * 255.0f);
  const uint8_t b = static_cast<uint8_t>(8.5f * (1.0f - t) * (1.0f - t) *
                                         (1.0f - t) * t * 255.0f);

  const uint32_t idx = (y * width + x) * channels;
  if (channels >= 3) {
    data[idx + 0] = r;
    data[idx + 1] = g;
    data[idx + 2] = b;
    if (channels > 3) {
      data[idx + 3] = 255;
    }
  } else if (channels == 2) {
    data[idx + 0] = r;
    data[idx + 1] = g;
  } else if (channels == 1) {
    data[idx] = r;
  }
}

cudaError_t launch_julia_kernel(uint8_t *data, uint32_t width, uint32_t height,
                                uint32_t channels, float zoom, float offset_x,
                                float offset_y, float c_real, float c_imag,
                                uint32_t max_iterations, cudaStream_t stream) {
  const dim3 block_dim(16, 16);
  const dim3 grid_dim((width + block_dim.x - 1) / block_dim.x,
                      (height + block_dim.y - 1) / block_dim.y);
  julia_kernel<<<grid_dim, block_dim, 0, stream>>>(
      data, width, height, channels, zoom, offset_x, offset_y, c_real, c_imag,
      max_iterations);
  return cudaGetLastError();
}

std::string cuda_error_to_string(cudaError_t err) {
  return std::string(cudaGetErrorName(err)) + ": " + cudaGetErrorString(err);
}

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

JuliaPublisherHelper::JuliaPublisherHelper(const Config &config)
    : config_(config) {
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

  initialise_shm();
  allocate_slots();
}

JuliaPublisherHelper::~JuliaPublisherHelper() { destroy_slots(); }

void JuliaPublisherHelper::initialise_shm() {
  if (!ros2_cuda_ipc_core::LeaseHandle::init(
          config_.shm_name, static_cast<uint32_t>(config_.slot_count))) {
    throw std::runtime_error("Failed to initialise lease shared memory: " +
                             config_.shm_name);
  }
}

void JuliaPublisherHelper::allocate_slots() {
  slots_.resize(config_.slot_count);
  for (std::size_t i = 0; i < slots_.size(); ++i) {
    auto &slot = slots_[i];
    slot.index = static_cast<uint32_t>(i);

    cudaError_t err = cudaMalloc(&slot.device_ptr, frame_size_bytes_);
    if (err != cudaSuccess) {
      throw std::runtime_error("cudaMalloc failed: " +
                               cuda_error_to_string(err));
    }

    err = cudaEventCreateWithFlags(
        &slot.event, cudaEventDisableTiming | cudaEventInterprocess);
    if (err != cudaSuccess) {
      throw std::runtime_error("cudaEventCreateWithFlags failed: " +
                               cuda_error_to_string(err));
    }

    err = cudaIpcGetMemHandle(&slot.mem_handle, slot.device_ptr);
    if (err != cudaSuccess) {
      throw std::runtime_error("cudaIpcGetMemHandle failed: " +
                               cuda_error_to_string(err));
    }

    err = cudaIpcGetEventHandle(&slot.event_handle, slot.event);
    if (err != cudaSuccess) {
      throw std::runtime_error("cudaIpcGetEventHandle failed: " +
                               cuda_error_to_string(err));
    }
  }
}

void JuliaPublisherHelper::destroy_slots() noexcept {
  for (auto &slot : slots_) {
    if (slot.event) {
      const cudaError_t event_err = cudaEventDestroy(slot.event);
      if (event_err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("JuliaPublisherHelper"),
                     "cudaEventDestroy failed for slot %u: %s", slot.index,
                     cuda_error_to_string(event_err).c_str());
      }
      slot.event = nullptr;
    }
    if (slot.device_ptr) {
      const cudaError_t free_err = cudaFree(slot.device_ptr);
      if (free_err != cudaSuccess) {
        RCLCPP_ERROR(rclcpp::get_logger("JuliaPublisherHelper"),
                     "cudaFree failed for slot %u: %s", slot.index,
                     cuda_error_to_string(free_err).c_str());
      }
      slot.device_ptr = nullptr;
    }
    slot.pending_deadline = {};
  }
  if (stream_) {
    const cudaError_t stream_err = cudaStreamDestroy(stream_);
    if (stream_err != cudaSuccess) {
      RCLCPP_ERROR(rclcpp::get_logger("JuliaPublisherHelper"),
                   "cudaStreamDestroy failed: %s",
                   cuda_error_to_string(stream_err).c_str());
    }
    stream_ = nullptr;
  }
}

void JuliaPublisherHelper::reclaim_stale_pending() {
  if (config_.pending_ttl.count() <= 0) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  auto logger = rclcpp::get_logger("JuliaPublisherHelper");

  for (auto &slot : slots_) {
    if (!deadline_reached(slot.pending_deadline, now)) {
      continue;
    }

    auto pending = ros2_cuda_ipc_core::LeaseHandle::current_pending(
        config_.shm_name, slot.index);
    if (!pending.has_value()) {
      continue;
    }
    if (pending.value() == 0) {
      slot.pending_deadline = {};
      continue;
    }

    auto refcnt = ros2_cuda_ipc_core::LeaseHandle::current_refcount(
        config_.shm_name, slot.index);
    if (!refcnt.has_value() || refcnt.value() != 0) {
      continue;
    }

    if (ros2_cuda_ipc_core::LeaseHandle::force_clear_pending(config_.shm_name,
                                                             slot.index)) {
      RCLCPP_WARN(
          logger, "Force-cleared pending lease slot=%u after %lld ms timeout",
          slot.index, static_cast<long long>(config_.pending_ttl.count()));
      slot.pending_deadline = {};
    }
  }
}

std::optional<ros2_cuda_ipc_core::ImageView> JuliaPublisherHelper::produce(
    std::size_t subscriber_count, float time_phase) {
  NvtxScopedRange produce_range("JuliaPublisherHelper::produce");
  if (slots_.empty()) {
    return std::nullopt;
  }

  if (config_.dtype != ros2_cuda_ipc_core::DType::U8) {
    RCLCPP_ERROR(rclcpp::get_logger("JuliaPublisherHelper"),
                 "Unsupported dtype for Julia set rendering");
    return std::nullopt;
  }

  reclaim_stale_pending();

  auto free_slot =
      ros2_cuda_ipc_core::LeaseHandle::choose_empty_slot(config_.shm_name);
  if (!free_slot.has_value()) {
    RCLCPP_WARN(rclcpp::get_logger("JuliaPublisherHelper"),
                "No available GPU slots (all leases in use)");
    return std::nullopt;
  }
  if (free_slot.value() >= slots_.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("JuliaPublisherHelper"),
                 "LeaseHandle returned invalid slot index %u",
                 free_slot.value());
    return std::nullopt;
  }

  auto &slot = slots_[free_slot.value()];

  auto generation = ros2_cuda_ipc_core::LeaseHandle::bump_generation(
      config_.shm_name, slot.index, static_cast<uint32_t>(subscriber_count));
  if (!generation.has_value()) {
    RCLCPP_WARN(rclcpp::get_logger("JuliaPublisherHelper"),
                "Failed to bump generation for slot %u", slot.index);
    return std::nullopt;
  }
  slot.generation = generation.value();

  const auto now = std::chrono::steady_clock::now();
  if (subscriber_count > 0 && config_.pending_ttl.count() > 0) {
    slot.pending_deadline = now + config_.pending_ttl;
  } else {
    slot.pending_deadline = {};
  }

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
    err = launch_julia_kernel(static_cast<uint8_t *>(slot.device_ptr),
                              config_.width, config_.height, config_.channels,
                              zoom, offset_x, offset_y, c_real, c_imag,
                              config_.max_iterations, stream_);
  }
  if (err != cudaSuccess) {
    RCLCPP_ERROR(rclcpp::get_logger("JuliaPublisherHelper"),
                 "launch_julia_kernel failed for slot %u: %s", slot.index,
                 cuda_error_to_string(err).c_str());
    return std::nullopt;
  }

  {
    NvtxScopedRange event_record_range("JuliaPublisherHelper::cudaEventRecord");
    err = cudaEventRecord(slot.event, stream_);
  }
  if (err != cudaSuccess) {
    RCLCPP_ERROR(rclcpp::get_logger("JuliaPublisherHelper"),
                 "cudaEventRecord failed for slot %u: %s", slot.index,
                 cuda_error_to_string(err).c_str());
    return std::nullopt;
  }

  ros2_cuda_ipc_core::ImageView view;
  view.core.dev_ptr = slot.device_ptr;
  view.core.ready_evt = slot.event;
  view.core.device_id = config_.device_index;
  view.core.byte_size = frame_size_bytes_;
  view.core.slot_id = slot.index;
  view.core.generation = slot.generation;
  view.core.shm_name = config_.shm_name;
  view.core.set_ipc_handles(slot.mem_handle, slot.event_handle);
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
