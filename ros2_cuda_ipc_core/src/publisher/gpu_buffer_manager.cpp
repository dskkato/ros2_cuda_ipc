// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"

#include <rcutils/logging_macros.h>

#include <utility>

namespace ros2_cuda_ipc_core::publisher {

std::string PreparePublishError::to_string() const {
  std::string result;
  switch (code_) {
    case PreparePublishErrorCode::kInvalidState:
      result = "invalid_state";
      break;
    case PreparePublishErrorCode::kReadyEventRecordFailed:
      result = "ready_event_record_failed";
      break;
    case PreparePublishErrorCode::kDescriptorCreationFailed:
      result = "descriptor_creation_failed";
      break;
    case PreparePublishErrorCode::kReservationCommitFailed:
      result = "reservation_commit_failed";
      break;
  }
  if (cuda_error_) {
    result += ": ";
    result += cuda_error_->to_string();
  }
  return result;
}

PublishSlot::PublishSlot(GpuBufferManager* owner,
                         LeaseManager::Reservation reservation) noexcept
    : owner_(owner), reservation_(reservation), state_(State::reserved) {}

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
  reservation_ = other.reservation_;
  state_ = other.state_;
  other.owner_ = nullptr;
  other.state_ = State::moved_from;
}

bool PublishSlot::valid() const noexcept {
  return owner_ != nullptr &&
         (state_ == State::reserved || state_ == State::ready_recorded);
}

void* PublishSlot::device_ptr() const noexcept {
  return valid() ? owner_->device_ptr(reservation_) : nullptr;
}

PreparePublishResult<transport::BufferDescriptor> PublishSlot::prepare_publish(
    CUstream stream) noexcept {
  if (owner_ == nullptr || state_ != State::reserved) {
    return PreparePublishResult<transport::BufferDescriptor>::failure(
        PreparePublishError(PreparePublishErrorCode::kInvalidState));
  }

  auto ready_result = record_ready(stream);
  if (!ready_result) {
    return PreparePublishResult<transport::BufferDescriptor>::failure(
        PreparePublishError(
            PreparePublishErrorCode::kReadyEventRecordFailed,
            std::optional<detail::CudaDriverError>(ready_result.error())));
  }

  auto result = descriptor();
  if (!result) {
    return PreparePublishResult<transport::BufferDescriptor>::failure(
        PreparePublishError(
            PreparePublishErrorCode::kDescriptorCreationFailed));
  }
  if (!commit_publish()) {
    return PreparePublishResult<transport::BufferDescriptor>::failure(
        PreparePublishError(PreparePublishErrorCode::kReservationCommitFailed));
  }
  return PreparePublishResult<transport::BufferDescriptor>::success(
      std::move(*result));
}

detail::CudaResult<void> PublishSlot::record_ready(CUstream stream) noexcept {
  if (owner_ == nullptr || state_ != State::reserved) {
    return detail::CudaResult<void>::failure(
        detail::CudaDriverError(CUDA_ERROR_INVALID_HANDLE));
  }
  auto result = owner_->record_ready(reservation_, stream);
  if (result) {
    state_ = State::ready_recorded;
  } else {
    quarantine(result.error());
  }
  return result;
}

std::optional<transport::BufferDescriptor> PublishSlot::descriptor() const {
  if (owner_ == nullptr || state_ != State::ready_recorded) {
    return std::nullopt;
  }
  return owner_->descriptor(reservation_);
}

bool PublishSlot::commit_publish() noexcept {
  if (owner_ == nullptr) {
    return false;
  }
  if (state_ == State::committed) {
    return true;
  }
  if (state_ != State::ready_recorded) {
    return false;
  }
  if (owner_->commit(reservation_)) {
    state_ = State::committed;
    return true;
  }
  // A failed commit must not make the slot inert while its reservation is
  // still active. The cancel path is generation-checked as well, so it is
  // safe to attempt even when the commit failed due to a stale reservation.
  if (owner_->cancel(reservation_)) {
    state_ = State::cancelled;
  }
  return false;
}

void PublishSlot::quarantine(const detail::CudaDriverError& error) noexcept {
  state_ = State::quarantined;
  RCUTILS_LOG_ERROR_NAMED(
      "ros2_cuda_ipc_core.publisher.gpu_buffer_manager",
      "Ready event recording failed for slot %u generation %u publisher=%s. "
      "The slot has been quarantined and will not be reused until "
      "GpuBufferManager is reset. CUDA error: %s",
      reservation_.slot_id, reservation_.generation,
      reservation_.shm_name.c_str(), error.to_string().c_str());
}

void PublishSlot::cancel() noexcept {
  if (owner_ != nullptr &&
      (state_ == State::reserved || state_ == State::ready_recorded)) {
    if (owner_->cancel(reservation_)) {
      state_ = State::cancelled;
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

std::optional<transport::BufferDescriptor> GpuBufferManager::descriptor(
    const LeaseManager::Reservation& reservation) const {
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
