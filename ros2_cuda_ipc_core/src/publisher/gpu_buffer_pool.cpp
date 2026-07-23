// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_pool.hpp"

#include <cstring>
#include <optional>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/backend/cuda_ipc/memory_backend.hpp"
#include "ros2_cuda_ipc_core/backend/vmm_fd/memory_backend.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
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
  detail::ScopedPrimaryContext context(device_index);
  if (!context.ok()) {
    RCLCPP_ERROR(logger_, "CUDA Driver context setup failed: %s",
                 detail::cu_result_to_string(context.status()).c_str());
    return false;
  }
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

cudaError_t GpuBufferPool::record_ready(uint32_t slot_id,
                                        cudaStream_t stream) noexcept {
  const auto* slot = resources(slot_id);
  if (!initialised_ || slot == nullptr || slot->event == nullptr) {
    return cudaErrorInvalidResourceHandle;
  }
  detail::ScopedPrimaryContext context(device_index_);
  if (!context.ok()) {
    return cudaErrorUnknown;
  }
  return detail::cuda_error_from_driver(
      cuEventRecord(reinterpret_cast<CUevent>(slot->event),
                    reinterpret_cast<CUstream>(stream)));
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
  for (auto& slot : slots_) {
    CUevent event = nullptr;
    CUresult result =
        cuEventCreate(&event, CU_EVENT_DISABLE_TIMING | CU_EVENT_INTERPROCESS);
    if (result != CUDA_SUCCESS) {
      RCLCPP_ERROR(logger_, "cuEventCreate failed: %s",
                   detail::cu_result_to_string(result).c_str());
      return false;
    }
    slot.event = reinterpret_cast<cudaEvent_t>(event);
    CUipcEventHandle handle{};
    result = cuIpcGetEventHandle(&handle, event);
    if (result != CUDA_SUCCESS) {
      RCLCPP_ERROR(logger_, "cuIpcGetEventHandle failed: %s",
                   detail::cu_result_to_string(result).c_str());
      (void)cuEventDestroy(event);
      slot.event = nullptr;
      return false;
    }
    std::memcpy(&slot.event_handle, &handle, sizeof(handle));
  }
  return true;
}

void GpuBufferPool::destroy_slots() noexcept {
  std::optional<detail::ScopedPrimaryContext> context;
  if (device_index_ >= 0) {
    context.emplace(device_index_);
    if (!context->ok()) {
      RCLCPP_ERROR(logger_,
                   "CUDA Driver context setup failed during cleanup: %s",
                   detail::cu_result_to_string(context->status()).c_str());
    }
  }
  for (auto& slot : slots_) {
    if (slot.event != nullptr) {
      const CUresult result =
          cuEventDestroy(reinterpret_cast<CUevent>(slot.event));
      if (result != CUDA_SUCCESS) {
        RCLCPP_ERROR(logger_, "cuEventDestroy failed for slot %u: %s",
                     slot.index, detail::cu_result_to_string(result).c_str());
      }
      slot.event = nullptr;
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
}

}  // namespace ros2_cuda_ipc_core::publisher
