// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"

#include <utility>

namespace ros2_cuda_ipc_core::publisher {

WriteHandle::WriteHandle(PublishSlot* slot, CUstream stream) noexcept
    : slot_(slot), stream_(stream) {}

WriteHandle::WriteHandle(WriteHandle&& other) noexcept
    : slot_(other.slot_), stream_(other.stream_) {
  other.slot_ = nullptr;
  other.stream_ = nullptr;
}

WriteHandle& WriteHandle::operator=(WriteHandle&& other) noexcept {
  if (this != &other) {
    finish();
    slot_ = other.slot_;
    stream_ = other.stream_;
    other.slot_ = nullptr;
    other.stream_ = nullptr;
  }
  return *this;
}

WriteHandle::~WriteHandle() noexcept { finish(); }

void* WriteHandle::data() const noexcept {
  return slot_ != nullptr ? slot_->write_data() : nullptr;
}

void WriteHandle::finish() noexcept {
  if (slot_ != nullptr) {
    slot_->finish_write(stream_);
    slot_ = nullptr;
    stream_ = nullptr;
  }
}

PublishSlot::PublishSlot(GpuBufferManager* owner,
                         LeaseManager::Reservation reservation) noexcept
    : owner_(owner),
      reservation_(std::move(reservation)),
      state_(State::reserved) {}

PublishSlot::PublishSlot(PublishSlot&& other) noexcept {
  move_from(std::move(other));
}

PublishSlot& PublishSlot::operator=(PublishSlot&& other) noexcept {
  if (this != &other) {
    release();
    move_from(std::move(other));
  }
  return *this;
}

PublishSlot::~PublishSlot() { release(); }

void PublishSlot::move_from(PublishSlot&& other) noexcept {
  owner_ = other.owner_;
  reservation_ = std::move(other.reservation_);
  state_ = other.state_;
  ready_error_ = std::move(other.ready_error_);
  other.owner_ = nullptr;
  other.state_ = State::moved_from;
  other.ready_error_.reset();
}

bool PublishSlot::valid() const noexcept {
  return owner_ != nullptr &&
         (state_ == State::reserved || state_ == State::writing ||
          state_ == State::ready_recorded);
}

std::optional<WriteHandle> PublishSlot::write(CUstream stream) noexcept {
  if (owner_ == nullptr || state_ != State::reserved) {
    return std::nullopt;
  }
  state_ = State::writing;
  return WriteHandle(this, stream);
}

void* PublishSlot::write_data() const noexcept {
  if (owner_ == nullptr || state_ != State::writing) {
    return nullptr;
  }
  return owner_->device_ptr(reservation_);
}

void PublishSlot::finish_write(CUstream stream) noexcept {
  if (owner_ == nullptr || state_ != State::writing) {
    return;
  }

  auto result = owner_->record_ready(reservation_, stream);
  if (result) {
    state_ = State::ready_recorded;
    ready_error_.reset();
  } else {
    ready_error_ = result.error();
    state_ = State::ready_failed;
  }
}

std::optional<transport::BufferDescriptor> PublishSlot::descriptor() const {
  if (owner_ == nullptr || state_ != State::ready_recorded) {
    return std::nullopt;
  }
  return owner_->descriptor(reservation_);
}

std::optional<detail::CudaDriverError> PublishSlot::ready_error()
    const noexcept {
  return ready_error_;
}

void PublishSlot::release() noexcept {
  if (owner_ != nullptr && state_ != State::released &&
      state_ != State::moved_from) {
    owner_->commit(reservation_);
    state_ = State::released;
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
  return PublishSlot(this, std::move(*reservation));
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

}  // namespace ros2_cuda_ipc_core::publisher
