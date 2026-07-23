// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_pool.hpp"

#include <cstring>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/backend/cuda_ipc/memory_backend.hpp"
#include "ros2_cuda_ipc_core/backend/vmm_fd/memory_backend.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_util.hpp"

namespace ros2_cuda_ipc_core::publisher {
namespace {

std::unique_ptr<GpuBufferPool::MemoryBackend> make_backend(
    transport::MemoryBackendKind backend) {
  if (backend == transport::MemoryBackendKind::VMM_FD) {
    return backend::vmm_fd::make_vmm_fd_memory_backend();
  }
  return backend::cuda_ipc::make_cuda_ipc_memory_backend();
}

}  // namespace

GpuBufferPool::GpuBufferPool(std::size_t slot_count,
                             transport::MemoryBackendKind backend,
                             rclcpp::Logger logger)
    : GpuBufferPool(slot_count, backend, std::move(logger), nullptr) {}

GpuBufferPool::GpuBufferPool(std::size_t slot_count,
                             transport::MemoryBackendKind backend,
                             rclcpp::Logger logger,
                             std::unique_ptr<MemoryBackend> memory_backend)
    : slot_count_(slot_count),
      backend_kind_(backend),
      logger_(std::move(logger)),
      memory_backend_(std::move(memory_backend)) {}

GpuBufferPool::~GpuBufferPool() { destroy_slots(); }

bool GpuBufferPool::initialise(uint64_t byte_size, int device_index) {
  if (slot_count_ == 0 || byte_size == 0) {
    RCLCPP_ERROR(logger_,
                 "GpuBufferPool requires slot_count and byte_size > 0");
    return false;
  }
  if (initialised_ || !slots_.empty()) {
    destroy_slots();
  }
  const cudaError_t set_device_error = cudaSetDevice(device_index);
  if (set_device_error != cudaSuccess) {
    RCLCPP_ERROR(logger_, "cudaSetDevice failed: %s",
                 detail::cuda_error_to_string(set_device_error).c_str());
    return false;
  }
  auto context_result = detail::CudaDeviceContext::retain_primary(device_index);
  if (!context_result) {
    RCLCPP_ERROR(logger_, "Failed to retain CUDA primary context: %s",
                 context_result.error().to_string().c_str());
    return false;
  }
  context_ = std::move(context_result).value();
  byte_size_ = byte_size;
  device_index_ = device_index;
  slots_.assign(slot_count_, {});
  for (std::size_t i = 0; i < slots_.size(); ++i) {
    slots_[i].index = static_cast<uint32_t>(i);
  }
  if (!allocate_slots()) {
    destroy_slots();
    return false;
  }
  initialised_ = true;
  return true;
}

void GpuBufferPool::reset() noexcept { destroy_slots(); }

bool GpuBufferPool::matches(uint64_t byte_size,
                            int device_index) const noexcept {
  return initialised_ && byte_size_ == byte_size &&
         device_index_ == device_index;
}

void* GpuBufferPool::device_ptr(uint32_t slot_id) const noexcept {
  const auto* slot = resources(slot_id);
  return slot ? slot->device_ptr : nullptr;
}

detail::CudaResult<void> GpuBufferPool::record_ready(
    uint32_t slot_id, cudaStream_t stream) noexcept {
  const auto* slot = resources(slot_id);
  if (!initialised_ || slot == nullptr || slot->event == nullptr) {
    return detail::CudaResult<void>::failure(
        detail::CudaDriverError(CUDA_ERROR_INVALID_HANDLE));
  }
  if (!context_) {
    return detail::CudaResult<void>::failure(
        detail::CudaDriverError(CUDA_ERROR_INVALID_CONTEXT));
  }
  auto guard_result = context_->push_current();
  if (!guard_result) {
    return detail::CudaResult<void>::failure(guard_result.error());
  }
  auto guard = std::move(guard_result).value();
  const CUresult result = cuEventRecord(slot->event, stream);
  if (result != CUDA_SUCCESS) {
    return detail::CudaResult<void>::failure(detail::CudaDriverError(result));
  }
  return detail::CudaResult<void>::success();
}

const GpuBufferPool::SlotResources* GpuBufferPool::resources(
    uint32_t slot_id) const noexcept {
  if (slot_id >= slots_.size()) {
    return nullptr;
  }
  return &slots_[slot_id];
}

bool GpuBufferPool::allocate_slots() {
  if (!memory_backend_) {
    memory_backend_ = make_backend(backend_kind_);
  }
  if (!memory_backend_) {
    return false;
  }
  if (!memory_backend_->allocate(byte_size_, device_index_, slots_, logger_)) {
    memory_backend_->destroy(slots_, logger_);
    memory_backend_.reset();
    return false;
  }
  if (!context_) {
    RCLCPP_ERROR(logger_, "CUDA primary context is unavailable");
    return false;
  }
  auto guard_result = context_->push_current();
  if (!guard_result) {
    RCLCPP_ERROR(logger_, "Failed to activate CUDA context: %s",
                 guard_result.error().to_string().c_str());
    return false;
  }
  auto guard = std::move(guard_result).value();
  for (auto& slot : slots_) {
    CUresult result = cuEventCreate(
        &slot.event, CU_EVENT_DISABLE_TIMING | CU_EVENT_INTERPROCESS);
    if (result != CUDA_SUCCESS) {
      RCLCPP_ERROR(logger_, "cuEventCreate failed: %s",
                   detail::CudaDriverError(result).to_string().c_str());
      return false;
    }
    CUipcEventHandle event_handle{};
    result = cuIpcGetEventHandle(&event_handle, slot.event);
    if (result != CUDA_SUCCESS) {
      RCLCPP_ERROR(logger_, "cuIpcGetEventHandle failed: %s",
                   detail::CudaDriverError(result).to_string().c_str());
      return false;
    }
    static_assert(
        sizeof(event_handle) == transport::EventHandlePayload{}.size(),
        "CUDA IPC event handle payload size changed");
    std::memcpy(slot.event_handle.data(), &event_handle, sizeof(event_handle));
  }
  return true;
}

void GpuBufferPool::destroy_slots() noexcept {
  if (device_index_ >= 0) {
    cudaSetDevice(device_index_);
  }
  if (context_) {
    auto guard_result = context_->push_current();
    if (!guard_result) {
      RCLCPP_ERROR(logger_,
                   "Failed to activate CUDA context for event cleanup: %s",
                   guard_result.error().to_string().c_str());
    } else {
      auto guard = std::move(guard_result).value();
      for (auto& slot : slots_) {
        if (slot.event != nullptr) {
          const CUresult result = cuEventDestroy(slot.event);
          if (result != CUDA_SUCCESS) {
            RCLCPP_ERROR(logger_, "cuEventDestroy failed for slot %u: %s",
                         slot.index,
                         detail::CudaDriverError(result).to_string().c_str());
          }
          slot.event = nullptr;
        }
      }
    }
  }
  if (memory_backend_) {
    memory_backend_->destroy(slots_, logger_);
    memory_backend_.reset();
  }
  slots_.clear();
  byte_size_ = 0;
  device_index_ = -1;
  initialised_ = false;
  context_.reset();
}

}  // namespace ros2_cuda_ipc_core::publisher
