// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"

#include <cassert>
#include <utility>

namespace ros2_cuda_ipc_core::publisher {

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

PublishSlot::~PublishSlot() { cancel(); }

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

cudaError_t PublishSlot::record_ready(cudaStream_t stream) noexcept {
  if (owner_ == nullptr || state_ != State::reserved) {
    return cudaErrorInvalidResourceHandle;
  }
  const cudaError_t error = owner_->record_ready(reservation_, stream);
  if (error == cudaSuccess) {
    state_ = State::ready_recorded;
  }
  return error;
}

std::optional<transport::BufferDescriptor> PublishSlot::descriptor() const {
  if (owner_ == nullptr || state_ != State::ready_recorded) {
    return std::nullopt;
  }
  return owner_->descriptor(reservation_);
}

void PublishSlot::commit_publish() noexcept {
  assert(owner_ != nullptr);
  assert(state_ == State::ready_recorded || state_ == State::committed);
  if (owner_ != nullptr && state_ == State::ready_recorded) {
    state_ = State::committed;
  }
}

void PublishSlot::cancel() noexcept {
  if (owner_ != nullptr &&
      (state_ == State::reserved || state_ == State::ready_recorded)) {
    owner_->cancel(reservation_);
    state_ = State::cancelled;
  }
}

GpuBufferManager::GpuBufferManager(Config config, rclcpp::Logger logger)
    : config_(std::move(config)),
      logger_(std::move(logger)),
      buffer_pool_(config_.slot_count, config_.backend,
                   logger_.get_child("GpuBufferPool")),
      lease_manager_(config_.shm_name, config_.slot_count, config_.pending_ttl,
                     logger_.get_child("LeaseManager")) {}

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

std::optional<PublishSlot> GpuBufferManager::acquire_for_publish(
    uint32_t pending_count) {
  if (!is_initialised()) {
    return std::nullopt;
  }
  lease_manager_.reclaim_stale_pending();
  auto reservation = lease_manager_.reserve_for_publish(pending_count);
  if (!reservation) {
    return std::nullopt;
  }
  return PublishSlot(this, *reservation);
}

void GpuBufferManager::reclaim_stale_pending() {
  lease_manager_.reclaim_stale_pending();
}

void* GpuBufferManager::device_ptr(
    const LeaseManager::Reservation& reservation) const noexcept {
  return buffer_pool_.device_ptr(reservation.slot_id);
}

cudaError_t GpuBufferManager::record_ready(
    const LeaseManager::Reservation& reservation,
    cudaStream_t stream) noexcept {
  return buffer_pool_.record_ready(reservation.slot_id, stream);
}

std::optional<transport::BufferDescriptor> GpuBufferManager::descriptor(
    const LeaseManager::Reservation& reservation) const {
  const auto* resources = buffer_pool_.resources(reservation.slot_id);
  if (resources == nullptr) {
    return std::nullopt;
  }
  transport::BufferDescriptor result;
  result.lease_shm_name = config_.shm_name;
  result.slot_id = reservation.slot_id;
  result.generation = reservation.generation;
  result.device_id = config_.device_index;
  result.byte_size = config_.byte_size;
  result.backend = resources->backend;
  result.memory_handle = resources->mem_handle;
  result.ready_event_handle = resources->event_handle;
  return result;
}

void GpuBufferManager::cancel(
    const LeaseManager::Reservation& reservation) noexcept {
  lease_manager_.cancel(reservation);
}

}  // namespace ros2_cuda_ipc_core::publisher
