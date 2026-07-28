// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"

#include <rcutils/logging_macros.h>

#include <utility>

namespace ros2_cuda_ipc_core::publisher {

PublishSlot::PublishSlot(GpuBufferManager* owner,
                         LeaseManager::Reservation reservation) noexcept
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

bool PublishSlot::valid() const noexcept { return owner_ != nullptr; }

void* PublishSlot::device_ptr() const noexcept {
  return valid() ? owner_->device_ptr(reservation_) : nullptr;
}

detail::CudaResult<transport::BufferDescriptor> PublishSlot::prepare_publish(
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

void PublishSlot::quarantine(const char* step,
                             const detail::CudaDriverError* error) noexcept {
  owner_ = nullptr;
  if (error != nullptr) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.publisher.gpu_buffer_manager",
        "Failed to safely release or publish slot %u generation %u "
        "publisher=%s during %s. The slot will not be reused until "
        "GpuBufferManager is reset. CUDA error: %s",
        reservation_.slot_id, reservation_.generation,
        reservation_.shm_name.c_str(), step, error->to_string().c_str());
    return;
  }
  RCUTILS_LOG_ERROR_NAMED(
      "ros2_cuda_ipc_core.publisher.gpu_buffer_manager",
      "Failed to safely release or publish slot %u generation %u "
      "publisher=%s during %s. The slot will not be reused until "
      "GpuBufferManager is reset.",
      reservation_.slot_id, reservation_.generation,
      reservation_.shm_name.c_str(), step);
}

void PublishSlot::cancel() noexcept {
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
      buffer_pool_(config_.slot_count, config_.backend),
      lease_manager_(config_.shm_name_prefix, config_.slot_count) {}

GpuBufferManager::~GpuBufferManager() { reset(); }

bool GpuBufferManager::initialise() {
  reset();
  if (!lease_manager_.initialise()) {
    return false;
  }
  if (!buffer_pool_.initialise(config_.byte_size, config_.device_index)) {
    lease_manager_.reset();
    return false;
  }
  return true;
}

void GpuBufferManager::reset() noexcept {
  buffer_pool_.reset();
  lease_manager_.reset();
}

bool GpuBufferManager::is_initialised() const noexcept {
  return buffer_pool_.is_initialised() && lease_manager_.is_initialised();
}

std::string GpuBufferManager::shm_name() const {
  return lease_manager_.shm_name();
}

PublisherInstanceId GpuBufferManager::publisher_instance_id() const {
  return lease_manager_.publisher_instance_id();
}

std::optional<PublishSlot> GpuBufferManager::acquire_for_publish() {
  if (!is_initialised()) {
    return std::nullopt;
  }
  auto reservation = lease_manager_.reserve_for_publish();
  if (!reservation) {
    return std::nullopt;
  }
  return PublishSlot(this, *reservation);
}

void* GpuBufferManager::device_ptr(
    const LeaseManager::Reservation& reservation) const noexcept {
  if (reservation.shm_name != lease_manager_.shm_name() ||
      reservation.publisher_instance_id !=
          lease_manager_.publisher_instance_id()) {
    return nullptr;
  }
  return buffer_pool_.device_ptr(reservation.slot_id);
}

detail::CudaResult<void> GpuBufferManager::record_ready(
    const LeaseManager::Reservation& reservation, CUstream stream) noexcept {
  if (reservation.shm_name != lease_manager_.shm_name() ||
      reservation.publisher_instance_id !=
          lease_manager_.publisher_instance_id()) {
    return detail::CudaResult<void>::failure(
        detail::CudaDriverError(CUDA_ERROR_INVALID_HANDLE));
  }
  return buffer_pool_.record_ready(reservation.slot_id, stream);
}

std::optional<transport::BufferDescriptor>
GpuBufferManager::try_build_descriptor(
    const LeaseManager::Reservation& reservation) const noexcept {
  const auto* resources = buffer_pool_.resources(reservation.slot_id);
  if (resources == nullptr || !resources->ready_event) {
    return std::nullopt;
  }
  if (reservation.shm_name != lease_manager_.shm_name() ||
      reservation.publisher_instance_id !=
          lease_manager_.publisher_instance_id()) {
    return std::nullopt;
  }
  transport::BufferDescriptor result;
  result.lease_shm_name = reservation.shm_name;
  result.publisher_instance_id = reservation.publisher_instance_id;
  result.slot_id = reservation.slot_id;
  result.generation = reservation.generation;
  result.device_id = config_.device_index;
  result.byte_size = config_.byte_size;
  result.backend = resources->backend;
  result.memory_handle = resources->mem_handle;
  result.ready_event_handle = resources->ready_event->ipc_handle();
  return result;
}

bool GpuBufferManager::commit(
    const LeaseManager::Reservation& reservation) noexcept {
  return lease_manager_.commit(reservation);
}

bool GpuBufferManager::cancel(
    const LeaseManager::Reservation& reservation) noexcept {
  return lease_manager_.cancel(reservation);
}

}  // namespace ros2_cuda_ipc_core::publisher
