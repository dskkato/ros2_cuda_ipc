// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_pool.hpp"

#include <rcutils/logging_macros.h>

#include <optional>

#include "ros2_cuda_ipc_core/backend/cuda_ipc/memory_backend.hpp"
#include "ros2_cuda_ipc_core/backend/vmm_fd/memory_backend.hpp"

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
                             transport::MemoryBackendKind backend)
    : GpuBufferPool(slot_count, backend, nullptr) {}

GpuBufferPool::GpuBufferPool(std::size_t slot_count,
                             transport::MemoryBackendKind backend,
                             std::unique_ptr<MemoryBackend> memory_backend)
    : slot_count_(slot_count),
      backend_kind_(backend),
      memory_backend_(std::move(memory_backend)) {}

GpuBufferPool::~GpuBufferPool() { destroy_slots(); }

bool GpuBufferPool::initialise(uint64_t byte_size, int device_index) {
  if (slot_count_ == 0 || byte_size == 0) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.publisher.gpu_buffer_pool",
        "GpuBufferPool requires slot_count and byte_size > 0");
    return false;
  }
  if (initialised_ || !slots_.empty()) {
    destroy_slots();
  }
  auto context_result = detail::CudaDeviceContext::retain_primary(device_index);
  if (!context_result) {
    RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.publisher.gpu_buffer_pool",
                            "Failed to retain CUDA primary context: %s",
                            context_result.error().to_string().c_str());
    return false;
  }
  context_ = std::move(context_result).value();
  byte_size_ = byte_size;
  device_index_ = device_index;
  slots_.clear();
  slots_.resize(slot_count_);
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

detail::CudaResult<void> GpuBufferPool::record_ready(uint32_t slot_id,
                                                     CUstream stream) noexcept {
  const auto* slot = resources(slot_id);
  if (!initialised_ || slot == nullptr || !slot->ready_event) {
    return detail::CudaResult<void>::failure(
        detail::CudaDriverError(CUDA_ERROR_INVALID_HANDLE));
  }
  return slot->ready_event->record(stream);
}

const GpuBufferPool::SlotResources* GpuBufferPool::resources(
    uint32_t slot_id) const noexcept {
  if (slot_id >= slots_.size()) {
    return nullptr;
  }
  return &slots_[slot_id];
}

bool GpuBufferPool::allocate_slots() {
  if (!context_) {
    RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.publisher.gpu_buffer_pool",
                            "CUDA primary context is unavailable");
    return false;
  }
  if (!memory_backend_) {
    memory_backend_ = make_backend(backend_kind_);
  }
  if (!memory_backend_) {
    return false;
  }
  {
    auto memory_guard_result = context_->push_current();
    if (!memory_guard_result) {
      RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.publisher.gpu_buffer_pool",
                              "Failed to activate CUDA context: %s",
                              memory_guard_result.error().to_string().c_str());
      return false;
    }
    auto memory_guard = std::move(memory_guard_result).value();
    if (!memory_backend_->allocate(byte_size_, device_index_, slots_)) {
      memory_backend_->destroy(slots_);
      memory_backend_.reset();
      return false;
    }
  }
  for (auto& slot : slots_) {
    auto event_result = detail::InterprocessEvent::create(context_);
    if (!event_result) {
      RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.publisher.gpu_buffer_pool",
                              "Failed to create interprocess event: %s",
                              event_result.error().to_string().c_str());
      return false;
    }
    slot.ready_event = std::move(event_result).value();
  }
  return true;
}

void GpuBufferPool::destroy_slots() noexcept {
  for (auto& slot : slots_) {
    slot.ready_event.reset();
  }
  std::optional<detail::CudaContextGuard> guard;
  if (context_) {
    auto guard_result = context_->push_current();
    if (!guard_result) {
      RCUTILS_LOG_ERROR_NAMED(
          "ros2_cuda_ipc_core.publisher.gpu_buffer_pool",
          "Failed to activate CUDA context for event cleanup: %s",
          guard_result.error().to_string().c_str());
    } else {
      guard.emplace(std::move(guard_result).value());
    }
  }
  if (memory_backend_) {
    memory_backend_->destroy(slots_);
    memory_backend_.reset();
  }
  slots_.clear();
  byte_size_ = 0;
  device_index_ = -1;
  initialised_ = false;
  context_.reset();
}

}  // namespace ros2_cuda_ipc_core::publisher
