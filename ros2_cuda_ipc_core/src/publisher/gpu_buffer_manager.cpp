// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"

#include <rcutils/logging_macros.h>

#include <utility>

namespace ros2_cuda_ipc_core::publisher {

PublishBlock::PublishBlock(
    GpuBufferManager* owner,
    BufferMetadataManager::Reservation reservation) noexcept
    : owner_(owner), reservation_(std::move(reservation)) {}

PublishBlock::PublishBlock(PublishBlock&& other) noexcept {
  move_from(std::move(other));
}

PublishBlock& PublishBlock::operator=(PublishBlock&& other) noexcept {
  if (this != &other) {
    cancel();
    move_from(std::move(other));
  }
  return *this;
}

PublishBlock::~PublishBlock() noexcept { cancel(); }

void PublishBlock::move_from(PublishBlock&& other) noexcept {
  owner_ = other.owner_;
  reservation_ = std::move(other.reservation_);
  other.owner_ = nullptr;
}

bool PublishBlock::valid() const noexcept { return owner_ != nullptr; }

void* PublishBlock::device_ptr() const noexcept {
  return valid() ? owner_->device_ptr(reservation_) : nullptr;
}

detail::CudaResult<transport::BufferDescriptor> PublishBlock::prepare_publish(
    CUstream stream) noexcept {
  if (owner_ == nullptr) {
    return detail::CudaResult<transport::BufferDescriptor>::failure(
        detail::CudaDriverError(CUDA_ERROR_INVALID_HANDLE));
  }

  auto descriptor = owner_->try_build_descriptor(reservation_);
  if (!descriptor) {
    quarantine("descriptor creation");
    return detail::CudaResult<transport::BufferDescriptor>::failure(
        detail::CudaDriverError(CUDA_ERROR_INVALID_HANDLE));
  }

  auto ready_result = owner_->record_ready(reservation_, stream);
  if (!ready_result) {
    quarantine("ready event recording", &ready_result.error());
    return detail::CudaResult<transport::BufferDescriptor>::failure(
        ready_result.error());
  }

  if (!owner_->commit(reservation_)) {
    quarantine("reservation commit");
    return detail::CudaResult<transport::BufferDescriptor>::failure(
        detail::CudaDriverError(CUDA_ERROR_INVALID_HANDLE));
  }
  owner_ = nullptr;
  return detail::CudaResult<transport::BufferDescriptor>::success(
      std::move(*descriptor));
}

void PublishBlock::quarantine(const char* step,
                              const detail::CudaDriverError* error) noexcept {
  owner_ = nullptr;
  if (error != nullptr) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.publisher.gpu_buffer_manager",
        "Failed to safely release or publish block %u uid %llu "
        "publisher=%s during %s. The block will not be reused until "
        "GpuBufferManager is reset. CUDA error: %s",
        reservation_.block_id,
        static_cast<unsigned long long>(reservation_.uid),
        reservation_.shm_name.c_str(), step, error->to_string().c_str());
    return;
  }
  RCUTILS_LOG_ERROR_NAMED(
      "ros2_cuda_ipc_core.publisher.gpu_buffer_manager",
      "Failed to safely release or publish block %u uid %llu "
      "publisher=%s during %s. The block will not be reused until "
      "GpuBufferManager is reset.",
      reservation_.block_id, static_cast<unsigned long long>(reservation_.uid),
      reservation_.shm_name.c_str(), step);
}

void PublishBlock::cancel() noexcept {
  if (owner_ != nullptr) {
    if (owner_->cancel(reservation_)) {
      owner_ = nullptr;
    } else {
      quarantine("reservation cancellation");
    }
  }
}

GpuBufferManager::GpuBufferManager(Config config)
    : config_(std::move(config)),
      buffer_pool_(config_.block_count),
      buffer_metadata_manager_(config_.block_count) {}

GpuBufferManager::~GpuBufferManager() { reset(); }

bool GpuBufferManager::initialise() {
  reset();
  if (!buffer_metadata_manager_.initialise()) {
    return false;
  }
  if (!buffer_pool_.initialise(config_.byte_size, config_.device_index)) {
    buffer_metadata_manager_.reset();
    return false;
  }
  for (std::size_t index = 0; index < buffer_pool_.size(); ++index) {
    const auto block_id =
        buffer_metadata_manager_.block_id_for_pool_index(index);
    const auto metadata =
        buffer_metadata_manager_.metadata_for_pool_index(index);
    if (!block_id || !buffer_pool_.set_shared_metadata(
                         static_cast<uint32_t>(index), *block_id, metadata)) {
      reset();
      return false;
    }
  }
  return true;
}

void GpuBufferManager::reset() noexcept {
  buffer_pool_.reset();
  buffer_metadata_manager_.reset();
}

bool GpuBufferManager::is_initialised() const noexcept {
  return buffer_pool_.is_initialised() &&
         buffer_metadata_manager_.is_initialised();
}

uint32_t GpuBufferManager::publisher_pid() const noexcept {
  return buffer_metadata_manager_.publisher_pid();
}

std::optional<PublishBlock> GpuBufferManager::acquire_for_publish() {
  if (!is_initialised()) {
    return std::nullopt;
  }
  auto reservation = buffer_metadata_manager_.reserve_for_publish();
  if (!reservation) {
    return std::nullopt;
  }
  return PublishBlock(this, *reservation);
}

void* GpuBufferManager::device_ptr(
    const BufferMetadataManager::Reservation& reservation) const noexcept {
  if (reservation.publisher_pid != buffer_metadata_manager_.publisher_pid()) {
    return nullptr;
  }
  return buffer_pool_.device_ptr(static_cast<uint32_t>(reservation.pool_index));
}

detail::CudaResult<void> GpuBufferManager::record_ready(
    const BufferMetadataManager::Reservation& reservation,
    CUstream stream) noexcept {
  if (reservation.publisher_pid != buffer_metadata_manager_.publisher_pid()) {
    return detail::CudaResult<void>::failure(
        detail::CudaDriverError(CUDA_ERROR_INVALID_HANDLE));
  }
  return buffer_pool_.record_ready(
      static_cast<uint32_t>(reservation.pool_index), stream);
}

std::optional<transport::BufferDescriptor>
GpuBufferManager::try_build_descriptor(
    const BufferMetadataManager::Reservation& reservation) const noexcept {
  const auto* resources =
      buffer_pool_.resources(static_cast<uint32_t>(reservation.pool_index));
  if (resources == nullptr || resources->block_id != reservation.block_id ||
      !resources->shared_metadata || !resources->ready_event) {
    return std::nullopt;
  }
  if (reservation.publisher_pid != buffer_metadata_manager_.publisher_pid()) {
    return std::nullopt;
  }
  transport::BufferDescriptor result;
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
    const BufferMetadataManager::Reservation& reservation) noexcept {
  return buffer_metadata_manager_.commit(reservation);
}

bool GpuBufferManager::cancel(
    const BufferMetadataManager::Reservation& reservation) noexcept {
  return buffer_metadata_manager_.cancel(reservation);
}

}  // namespace ros2_cuda_ipc_core::publisher
