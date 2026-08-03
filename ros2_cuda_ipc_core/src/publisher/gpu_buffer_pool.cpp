// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_pool.hpp"

#include <dirent.h>
#include <limits.h>
#include <rcutils/logging_macros.h>
#include <signal.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <limits>
#include <utility>

namespace ros2_cuda_ipc_core::publisher {
namespace {

std::atomic<uint32_t> next_process_block_id{0};

std::optional<uint32_t> allocate_process_block_id() {
  uint32_t current = next_process_block_id.load(std::memory_order_relaxed);
  while (current != std::numeric_limits<uint32_t>::max()) {
    if (next_process_block_id.compare_exchange_weak(
            current, current + 1, std::memory_order_relaxed,
            std::memory_order_relaxed)) {
      return current;
    }
  }
  return std::nullopt;
}

void cleanup_orphaned_metadata_objects() noexcept {
  DIR* directory = ::opendir("/dev/shm");
  if (directory == nullptr) return;
  while (const dirent* entry = ::readdir(directory)) {
    uint32_t pid = 0;
    uint32_t block_id = 0;
    char suffix = '\0';
    if (std::sscanf(entry->d_name, "ros2_cuda_ipc_%u_%u%c", &pid, &block_id,
                    &suffix) != 2 ||
        pid == 0) {
      continue;
    }
    if (::kill(static_cast<pid_t>(pid), 0) == 0 || errno != ESRCH) {
      continue;
    }
    const std::string shm_name = "/" + std::string(entry->d_name);
    if (::shm_unlink(shm_name.c_str()) != 0 && errno != ENOENT) {
      RCUTILS_LOG_WARN_NAMED(
          "ros2_cuda_ipc_core.publisher.gpu_buffer_pool",
          "Failed to clean orphaned block metadata name=%s errno=%d",
          shm_name.c_str(), errno);
    }
  }
  ::closedir(directory);
}

}  // namespace

GpuBufferPool::GpuBufferPool(std::size_t block_count)
    : block_count_(block_count),
      publisher_pid_(static_cast<uint32_t>(::getpid())) {}

GpuBufferPool::~GpuBufferPool() { destroy_blocks(); }

bool GpuBufferPool::initialise(uint64_t byte_size, int device_index) {
  if (block_count_ == 0 ||
      block_count_ > std::numeric_limits<uint32_t>::max() || byte_size == 0) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.publisher.gpu_buffer_pool",
        "GpuBufferPool requires a valid block_count and byte_size");
    return false;
  }
  if (initialised_ || !blocks_.empty()) {
    destroy_blocks();
  }
  cleanup_orphaned_metadata_objects();

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

  // Metadata and GPU resources are built into the same block objects. Any
  // later failure is handled by destroy_blocks(), which unlinks every name
  // already created and releases every resource already allocated.
  for (auto& block : blocks_) {
    const auto block_id = allocate_process_block_id();
    if (!block_id) {
      RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.publisher.gpu_buffer_pool",
                              "Process-local block_id space exhausted");
      destroy_blocks();
      return false;
    }
    const std::string shm_name =
        buffer_metadata::shm_name_for_block(publisher_pid_, *block_id);
    if (shm_name.size() > NAME_MAX) {
      RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.publisher.gpu_buffer_pool",
                              "Generated shared-memory name is too long");
      destroy_blocks();
      return false;
    }
    auto mapping = buffer_metadata::BufferMetadata::create(shm_name);
    if (!mapping && errno == EEXIST) {
      RCUTILS_LOG_WARN_NAMED(
          "ros2_cuda_ipc_core.publisher.gpu_buffer_pool",
          "Replacing stale block metadata after PID reuse name=%s",
          shm_name.c_str());
      const int unlink_result = ::shm_unlink(shm_name.c_str());
      if (unlink_result == 0 || errno == ENOENT) {
        mapping = buffer_metadata::BufferMetadata::create(shm_name);
      }
    }
    if (!mapping) {
      destroy_blocks();
      return false;
    }
    block.block_id = *block_id;
    block.shared_metadata = std::move(mapping);
  }

  if (!allocate_blocks()) {
    destroy_blocks();
    return false;
  }
  initialised_ = true;
  next_block_ = 0;
  return true;
}

void GpuBufferPool::reset() noexcept { destroy_blocks(); }

bool GpuBufferPool::matches(uint64_t byte_size,
                            int device_index) const noexcept {
  return initialised_ && byte_size_ == byte_size &&
         device_index_ == device_index;
}

std::optional<GpuBufferPool::BlockReservation>
GpuBufferPool::reserve_for_publish() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialised_ || blocks_.empty()) return std::nullopt;
  for (std::size_t offset = 0; offset < blocks_.size(); ++offset) {
    const std::size_t index = (next_block_ + offset) % blocks_.size();
    auto& block = blocks_[index];
    if (!block.shared_metadata) continue;
    auto reservation =
        buffer_metadata::BufferRef::reserve_for_publish(block.shared_metadata);
    if (!reservation) continue;
    next_block_ = (index + 1) % blocks_.size();
    return BlockReservation{reservation->mapping, block.block_id,
                            reservation->uid, publisher_pid_,
                            static_cast<uint32_t>(index)};
  }
  return std::nullopt;
}

void* GpuBufferPool::device_ptr(uint32_t pool_index) const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto* block = resources(pool_index);
  return block ? block->device_ptr : nullptr;
}

detail::CudaResult<void> GpuBufferPool::record_ready(uint32_t pool_index,
                                                     CUstream stream) noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto* block = resources(pool_index);
  if (!initialised_ || block == nullptr || !block->ready_event) {
    return detail::CudaResult<void>::failure(
        detail::CudaDriverError(CUDA_ERROR_INVALID_HANDLE));
  }
  return block->ready_event->record(stream);
}

void* GpuBufferPool::device_ptr(
    const BlockReservation& reservation) const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto* block =
      owns(reservation) ? resources(reservation.pool_index) : nullptr;
  return block ? block->device_ptr : nullptr;
}

detail::CudaResult<void> GpuBufferPool::record_ready(
    const BlockReservation& reservation, CUstream stream) noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto* block =
      owns(reservation) ? resources(reservation.pool_index) : nullptr;
  if (block == nullptr || !block->ready_event) {
    return detail::CudaResult<void>::failure(
        detail::CudaDriverError(CUDA_ERROR_INVALID_HANDLE));
  }
  return block->ready_event->record(stream);
}

std::optional<transport::BufferDescriptor> GpuBufferPool::try_build_descriptor(
    const BlockReservation& reservation) const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto* block =
      owns(reservation) ? resources(reservation.pool_index) : nullptr;
  if (block == nullptr || !block->ready_event) return std::nullopt;
  transport::BufferDescriptor result;
  result.publisher_pid = reservation.publisher_pid;
  result.block_id = block->block_id;
  result.uid = reservation.uid;
  result.device_id = device_index_;
  result.byte_size = byte_size_;
  result.vmm_socket_path = block->vmm_socket_path;
  result.ready_event_handle = block->ready_event->ipc_handle();
  return result;
}

bool GpuBufferPool::commit(const BlockReservation& reservation) noexcept {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!owns(reservation)) return false;
  }
  return buffer_metadata::BufferRef::commit_publish(reservation.mapping,
                                                    reservation.uid);
}

bool GpuBufferPool::cancel(const BlockReservation& reservation) noexcept {
  // Keep cancellation valid after reset. The reservation owns the mapping
  // until its PublishBlock is destroyed, matching the previous lifecycle.
  return reservation.publisher_pid == publisher_pid_ && reservation.mapping &&
         buffer_metadata::BufferRef::cancel_publish(reservation.mapping,
                                                    reservation.uid);
}

const GpuBufferPool::GpuBufferBlock* GpuBufferPool::resources(
    uint32_t pool_index) const noexcept {
  if (pool_index >= blocks_.size()) return nullptr;
  return &blocks_[pool_index];
}

bool GpuBufferPool::owns(const BlockReservation& reservation) const noexcept {
  if (!initialised_ || reservation.publisher_pid != publisher_pid_ ||
      reservation.mapping == nullptr) {
    return false;
  }
  const auto* block = resources(reservation.pool_index);
  return block != nullptr && block->block_id == reservation.block_id &&
         block->shared_metadata.get() == reservation.mapping.get();
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
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& block : blocks_) block.ready_event.reset();

  std::optional<detail::CudaContextGuard> guard;
  if (context_) {
    auto guard_result = context_->push_current();
    if (!guard_result) {
      RCUTILS_LOG_ERROR_NAMED(
          "ros2_cuda_ipc_core.publisher.gpu_buffer_pool",
          "Failed to activate CUDA context for resource cleanup: %s",
          guard_result.error().to_string().c_str());
    } else {
      guard.emplace(std::move(guard_result).value());
    }
  }
  backend::destroy_vmm_fd_memory(blocks_);

  for (auto& block : blocks_) {
    if (block.shared_metadata) {
      const auto& shm_name = block.shared_metadata->shm_name();
      if (::shm_unlink(shm_name.c_str()) != 0 && errno != ENOENT) {
        RCUTILS_LOG_WARN_NAMED(
            "ros2_cuda_ipc_core.publisher.gpu_buffer_pool",
            "Failed to unlink block metadata shared memory name=%s errno=%d",
            shm_name.c_str(), errno);
      }
    }
    block.shared_metadata.reset();
    block.block_id = 0;
  }
  blocks_.clear();
  next_block_ = 0;
  byte_size_ = 0;
  device_index_ = -1;
  initialised_ = false;
  context_.reset();
}

}  // namespace ros2_cuda_ipc_core::publisher
