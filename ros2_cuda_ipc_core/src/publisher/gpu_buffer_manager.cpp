// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"

#include <rcutils/logging_macros.h>

#include <utility>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"

namespace ros2_cuda_ipc_core::publisher {

PublishSlot::PublishSlot(GpuBufferManager* owner,
                         Reservation reservation) noexcept
    : owner_(owner), reservation_(std::move(reservation)) {}

PublishSlot::PublishSlot(PublishSlot&& other) noexcept {
  move_from(std::move(other));
}

PublishSlot& PublishSlot::operator=(PublishSlot&& other) noexcept {
  if (this != &other) {
    cancel();
    move_from(std::move(other));
  }
  return *this;
}

PublishSlot::~PublishSlot() noexcept { cancel(); }

void PublishSlot::move_from(PublishSlot&& other) noexcept {
  owner_ = other.owner_;
  reservation_ = std::move(other.reservation_);
  other.owner_ = nullptr;
}

void* PublishSlot::device_ptr() const noexcept {
  return owner_ ? owner_->device_ptr(reservation_) : nullptr;
}

detail::CudaResult<transport::BlockDescriptor> PublishSlot::prepare_publish(
    CUstream stream) noexcept {
  if (!owner_) {
    return detail::CudaResult<transport::BlockDescriptor>::failure(
        detail::CudaDriverError(CUDA_ERROR_INVALID_HANDLE));
  }
  auto descriptor = owner_->try_build_descriptor(reservation_);
  if (!descriptor) {
    quarantine("descriptor creation");
    return detail::CudaResult<transport::BlockDescriptor>::failure(
        detail::CudaDriverError(CUDA_ERROR_INVALID_HANDLE));
  }
  auto ready_result = owner_->record_ready(reservation_, stream);
  if (!ready_result) {
    quarantine("ready event recording", &ready_result.error());
    return detail::CudaResult<transport::BlockDescriptor>::failure(
        ready_result.error());
  }
  if (!owner_->commit(reservation_)) {
    quarantine("reservation commit");
    return detail::CudaResult<transport::BlockDescriptor>::failure(
        detail::CudaDriverError(CUDA_ERROR_INVALID_HANDLE));
  }
  owner_ = nullptr;
  return detail::CudaResult<transport::BlockDescriptor>::success(
      std::move(*descriptor));
}

void PublishSlot::quarantine(const char* step,
                             const detail::CudaDriverError* error) noexcept {
  owner_ = nullptr;
  RCUTILS_LOG_ERROR_NAMED(
      "ros2_cuda_ipc_core.publisher.gpu_buffer_manager",
      "Block %u uid %llu cannot be safely released during %s%s",
      reservation_.block_id, static_cast<unsigned long long>(reservation_.uid),
      step, error ? error->to_string().c_str() : "");
}

void PublishSlot::cancel() noexcept {
  if (owner_ && owner_->cancel(reservation_))
    owner_ = nullptr;
  else if (owner_)
    quarantine("reservation cancellation");
}

GpuBufferManager::GpuBufferManager(Config config)
    : config_(std::move(config)), buffer_pool_(config_.block_count) {}

GpuBufferManager::~GpuBufferManager() { reset(); }

bool GpuBufferManager::initialise() {
  reset();
  return buffer_pool_.initialise(config_.byte_size, config_.device_index);
}

void GpuBufferManager::reset() noexcept { buffer_pool_.reset(); }

bool GpuBufferManager::is_initialised() const noexcept {
  return buffer_pool_.is_initialised();
}

std::optional<PublishSlot> GpuBufferManager::acquire_for_publish() {
  if (!is_initialised() || buffer_pool_.size() == 0) return std::nullopt;
  const std::size_t count = buffer_pool_.size();
  const uint32_t start =
      next_block_index_.fetch_add(1, std::memory_order_relaxed);
  for (std::size_t offset = 0; offset < count; ++offset) {
    const uint32_t index = static_cast<uint32_t>((start + offset) % count);
    auto* block = buffer_pool_.block(index);
    if (!block) continue;
    auto reservation =
        buffer_metadata::BufferRef::reserve_for_publish(block->metadata);
    if (!reservation) continue;
    return PublishSlot(
        this, PublishSlot::Reservation{reservation->mapping, index,
                                       block->publisher_pid, block->block_id,
                                       reservation->uid});
  }
  return std::nullopt;
}

void* GpuBufferManager::device_ptr(
    const PublishSlot::Reservation& reservation) const noexcept {
  const auto* block = buffer_pool_.block(reservation.block_index);
  if (!block || block->publisher_pid != reservation.publisher_pid ||
      block->block_id != reservation.block_id ||
      block->metadata.get() != reservation.mapping.get())
    return nullptr;
  return buffer_pool_.device_ptr(reservation.block_index);
}

detail::CudaResult<void> GpuBufferManager::record_ready(
    const PublishSlot::Reservation& reservation, CUstream stream) noexcept {
  if (device_ptr(reservation) == nullptr) {
    return detail::CudaResult<void>::failure(
        detail::CudaDriverError(CUDA_ERROR_INVALID_HANDLE));
  }
  return buffer_pool_.record_ready(reservation.block_index, stream);
}

std::optional<transport::BlockDescriptor>
GpuBufferManager::try_build_descriptor(
    const PublishSlot::Reservation& reservation) const noexcept {
  const auto* block = buffer_pool_.block(reservation.block_index);
  const auto* resources = buffer_pool_.resources(reservation.block_index);
  if (!block || !resources ||
      block->metadata.get() != reservation.mapping.get() ||
      !resources->ready_event)
    return std::nullopt;
  transport::BlockDescriptor result;
  result.publisher_pid = reservation.publisher_pid;
  result.block_id = reservation.block_id;
  result.uid = reservation.uid;
  result.device_id = config_.device_index;
  result.byte_size = config_.byte_size;
  result.vmm_socket_path = resources->vmm_socket_path;
  result.ready_event_handle = resources->ready_event->ipc_handle();
  return result;
}

bool GpuBufferManager::commit(
    const PublishSlot::Reservation& reservation) noexcept {
  return buffer_metadata::BufferRef::commit_publish(reservation.mapping,
                                                    reservation.uid);
}

bool GpuBufferManager::cancel(
    const PublishSlot::Reservation& reservation) noexcept {
  return buffer_metadata::BufferRef::cancel_publish(reservation.mapping,
                                                    reservation.uid);
}

}  // namespace ros2_cuda_ipc_core::publisher
