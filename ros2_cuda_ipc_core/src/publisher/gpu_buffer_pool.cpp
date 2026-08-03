// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_pool.hpp"

#include <limits.h>
#include <rcutils/logging_macros.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <limits>
#include <optional>
#include <random>

namespace ros2_cuda_ipc_core::publisher {
namespace {

std::atomic<uint32_t> next_block_id{1};
std::atomic<uint64_t> next_uid{1};

uint32_t allocate_block_id() {
  uint32_t id = next_block_id.fetch_add(1, std::memory_order_relaxed);
  if (id == 0) id = next_block_id.fetch_add(1, std::memory_order_relaxed);
  return id;
}

uint64_t allocate_uid() {
  uint64_t value = next_uid.fetch_add(1, std::memory_order_relaxed);
  if (value == 0) value = next_uid.fetch_add(1, std::memory_order_relaxed);
  std::random_device device;
  value ^= static_cast<uint64_t>(device()) << 32;
  value ^= static_cast<uint64_t>(device());
  return value == 0 ? 1 : value;
}

}  // namespace

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
  destroy_blocks();
  auto context_result = detail::CudaDeviceContext::retain_primary(device_index);
  if (!context_result) return false;
  context_ = std::move(context_result).value();
  byte_size_ = byte_size;
  device_index_ = device_index;
  blocks_.resize(block_count_);
  const uint32_t pid = static_cast<uint32_t>(::getpid());
  for (auto& block : blocks_) {
    block.publisher_pid = pid;
    block.block_id = allocate_block_id();
    block.metadata_shm_name =
        buffer_metadata::block_metadata_shm_name(pid, block.block_id);
    if (block.metadata_shm_name.size() > NAME_MAX ||
        !(block.metadata = buffer_metadata::BufferMetadata::create(
              block.metadata_shm_name, allocate_uid()))) {
      destroy_blocks();
      return false;
    }
  }
  for (std::size_t i = 0; i < blocks_.size(); ++i) {
    blocks_[i].resources.index = static_cast<uint32_t>(i);
  }
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

GpuBufferBlock* GpuBufferPool::block(uint32_t index) noexcept {
  return index < blocks_.size() ? &blocks_[index] : nullptr;
}

const GpuBufferBlock* GpuBufferPool::block(uint32_t index) const noexcept {
  return index < blocks_.size() ? &blocks_[index] : nullptr;
}

void* GpuBufferPool::device_ptr(uint32_t block_index) const noexcept {
  const auto* value = block(block_index);
  return value ? value->resources.device_ptr : nullptr;
}

detail::CudaResult<void> GpuBufferPool::record_ready(uint32_t block_index,
                                                     CUstream stream) noexcept {
  const auto* value = block(block_index);
  if (!initialised_ || value == nullptr || !value->resources.ready_event) {
    return detail::CudaResult<void>::failure(
        detail::CudaDriverError(CUDA_ERROR_INVALID_HANDLE));
  }
  return value->resources.ready_event->record(stream);
}

const backend::BlockResources* GpuBufferPool::resources(
    uint32_t block_index) const noexcept {
  const auto* value = block(block_index);
  return value ? &value->resources : nullptr;
}

bool GpuBufferPool::allocate_blocks() {
  if (!context_) return false;
  auto memory_guard_result = context_->push_current();
  if (!memory_guard_result) return false;
  auto memory_guard = std::move(memory_guard_result).value();
  std::vector<backend::BlockResources> resources;
  resources.resize(blocks_.size());
  for (std::size_t i = 0; i < resources.size(); ++i) {
    resources[i].index = static_cast<uint32_t>(i);
  }
  if (!backend::allocate_vmm_fd_memory(byte_size_, device_index_, resources)) {
    backend::destroy_vmm_fd_memory(resources);
    return false;
  }
  for (auto& resource : resources) {
    auto event_result = detail::InterprocessEvent::create(context_);
    if (!event_result) {
      backend::destroy_vmm_fd_memory(resources);
      return false;
    }
    resource.ready_event = std::move(event_result).value();
  }
  for (std::size_t i = 0; i < blocks_.size(); ++i) {
    blocks_[i].resources = std::move(resources[i]);
  }
  return true;
}

void GpuBufferPool::destroy_blocks() noexcept {
  for (auto& block : blocks_) block.resources.ready_event.reset();
  std::optional<detail::CudaContextGuard> guard;
  if (context_) {
    auto result = context_->push_current();
    if (result) guard.emplace(std::move(result).value());
  }
  std::vector<backend::BlockResources> resources;
  resources.reserve(blocks_.size());
  for (auto& block : blocks_) resources.push_back(std::move(block.resources));
  backend::destroy_vmm_fd_memory(resources);
  for (const auto& block : blocks_) {
    if (!block.metadata_shm_name.empty()) {
      (void)::shm_unlink(block.metadata_shm_name.c_str());
    }
  }
  blocks_.clear();
  byte_size_ = 0;
  device_index_ = -1;
  initialised_ = false;
  context_.reset();
}

}  // namespace ros2_cuda_ipc_core::publisher
