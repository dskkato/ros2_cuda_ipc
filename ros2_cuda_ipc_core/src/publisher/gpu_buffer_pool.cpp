// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_pool.hpp"

#include <rcutils/logging_macros.h>

#include <optional>

namespace ros2_cuda_ipc_core::publisher {

GpuBufferPool::GpuBufferPool(std::size_t block_count)
    : block_count_(block_count) {}

GpuBufferPool::~GpuBufferPool() { destroy_blocks(); }

bool GpuBufferPool::initialise(uint64_t byte_size, int device_index) {
  if (block_count_ == 0 || byte_size == 0) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.publisher.gpu_buffer_pool",
        "GpuBufferPool requires block_count and byte_size > 0");
    return false;
  }
  if (initialised_ || !blocks_.empty()) {
    destroy_blocks();
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
  blocks_.clear();
  blocks_.resize(block_count_);
  if (!allocate_blocks()) {
    destroy_blocks();
    return false;
  }
  initialised_ = true;
  return true;
}

void GpuBufferPool::reset() noexcept { destroy_blocks(); }

bool GpuBufferPool::matches(uint64_t byte_size,
                            int device_index) const noexcept {
  return initialised_ && byte_size_ == byte_size &&
         device_index_ == device_index;
}

void* GpuBufferPool::device_ptr(uint32_t pool_index) const noexcept {
  const auto* block = resources(pool_index);
  return block ? block->device_ptr : nullptr;
}

detail::CudaResult<void> GpuBufferPool::record_ready(uint32_t pool_index,
                                                     CUstream stream) noexcept {
  const auto* block = resources(pool_index);
  if (!initialised_ || block == nullptr || !block->ready_event) {
    return detail::CudaResult<void>::failure(
        detail::CudaDriverError(CUDA_ERROR_INVALID_HANDLE));
  }
  return block->ready_event->record(stream);
}

const GpuBufferPool::GpuBufferBlock* GpuBufferPool::resources(
    uint32_t pool_index) const noexcept {
  if (pool_index >= blocks_.size()) {
    return nullptr;
  }
  return &blocks_[pool_index];
}

bool GpuBufferPool::set_shared_metadata(
    uint32_t pool_index, uint32_t block_id,
    std::shared_ptr<buffer_metadata::BufferMetadata> metadata) noexcept {
  if (pool_index >= blocks_.size() || !metadata) return false;
  blocks_[pool_index].block_id = block_id;
  blocks_[pool_index].shared_metadata = std::move(metadata);
  return true;
}

bool GpuBufferPool::allocate_blocks() {
  if (!context_) {
    RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.publisher.gpu_buffer_pool",
                            "CUDA primary context is unavailable");
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
    if (!backend::allocate_vmm_fd_memory(byte_size_, device_index_, blocks_)) {
      backend::destroy_vmm_fd_memory(blocks_);
      return false;
    }
  }
  for (auto& block : blocks_) {
    auto event_result = detail::InterprocessEvent::create(context_);
    if (!event_result) {
      RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.publisher.gpu_buffer_pool",
                              "Failed to create interprocess event: %s",
                              event_result.error().to_string().c_str());
      return false;
    }
    block.ready_event = std::move(event_result).value();
  }
  return true;
}

void GpuBufferPool::destroy_blocks() noexcept {
  for (auto& block : blocks_) {
    block.ready_event.reset();
    block.shared_metadata.reset();
    block.block_id = 0;
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
  backend::destroy_vmm_fd_memory(blocks_);
  blocks_.clear();
  byte_size_ = 0;
  device_index_ = -1;
  initialised_ = false;
  context_.reset();
}

}  // namespace ros2_cuda_ipc_core::publisher
