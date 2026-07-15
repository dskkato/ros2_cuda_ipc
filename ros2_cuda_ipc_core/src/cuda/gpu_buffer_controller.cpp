// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/cuda/gpu_buffer_controller.hpp"

#include <cassert>
#include <utility>

namespace ros2_cuda_ipc_core::cuda {

PublishSlot::PublishSlot(GpuBufferController* owner,
                         SlotController::Reservation reservation) noexcept
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

std::optional<BufferDescriptor> PublishSlot::descriptor() const {
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

GpuBufferController::GpuBufferController(Config config, rclcpp::Logger logger)
    : config_(std::move(config)),
      logger_(std::move(logger)),
      buffer_pool_(config_.slot_count, config_.backend,
                   logger_.get_child("GpuBufferPool")),
      slot_controller_(config_.shm_name, config_.slot_count,
                       config_.pending_ttl,
                       logger_.get_child("SlotController")) {}

bool GpuBufferController::initialise() {
  reset();
  if (!slot_controller_.initialise()) {
    return false;
  }
  if (!buffer_pool_.initialise(config_.byte_size, config_.device_index)) {
    slot_controller_.reset();
    return false;
  }
  return true;
}

void GpuBufferController::reset() noexcept {
  buffer_pool_.reset();
  slot_controller_.reset();
}

bool GpuBufferController::is_initialised() const noexcept {
  return buffer_pool_.is_initialised() && slot_controller_.is_initialised();
}

std::optional<PublishSlot> GpuBufferController::acquire_for_publish(
    uint32_t pending_count) {
  if (!is_initialised()) {
    return std::nullopt;
  }
  slot_controller_.reclaim_stale_pending();
  auto reservation = slot_controller_.reserve_for_publish(pending_count);
  if (!reservation) {
    return std::nullopt;
  }
  return PublishSlot(this, *reservation);
}

void GpuBufferController::reclaim_stale_pending() {
  slot_controller_.reclaim_stale_pending();
}

void* GpuBufferController::device_ptr(
    const SlotController::Reservation& reservation) const noexcept {
  return buffer_pool_.device_ptr(reservation.slot_id);
}

cudaError_t GpuBufferController::record_ready(
    const SlotController::Reservation& reservation,
    cudaStream_t stream) noexcept {
  return buffer_pool_.record_ready(reservation.slot_id, stream);
}

std::optional<BufferDescriptor> GpuBufferController::descriptor(
    const SlotController::Reservation& reservation) const {
  const auto* resources = buffer_pool_.resources(reservation.slot_id);
  if (resources == nullptr) {
    return std::nullopt;
  }
  BufferDescriptor result;
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

void GpuBufferController::cancel(
    const SlotController::Reservation& reservation) noexcept {
  slot_controller_.cancel(reservation);
}

}  // namespace ros2_cuda_ipc_core::cuda
