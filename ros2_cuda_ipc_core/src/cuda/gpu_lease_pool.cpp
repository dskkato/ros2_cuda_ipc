// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/cuda/gpu_lease_pool.hpp"

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/lease_handle.hpp"
#include "ros2_cuda_ipc_core/memory_types.hpp"

namespace ros2_cuda_ipc_core::cuda {

using ros2_cuda_ipc_core::LeaseHandle;

GpuLeasePool::GpuLeasePool(Config config, rclcpp::Logger logger)
    : config_(std::move(config)),
      logger_(std::move(logger)),
      buffer_pool_(config_.slot_count, config_.backend,
                   logger_.get_child("GpuBufferPool")),
      slot_controller_(config_.shm_name, config_.slot_count,
                       config_.pending_ttl,
                       logger_.get_child("SlotController")) {}

GpuLeasePool::~GpuLeasePool() { destroy_slots(); }

bool GpuLeasePool::initialise(uint64_t frame_size_bytes, int device_index) {
  if (config_.slot_count == 0) {
    RCLCPP_ERROR(logger_, "GpuLeasePool requires slot_count > 0");
    return false;
  }

  destroy_slots();

  if (!slot_controller_.initialise()) {
    RCLCPP_ERROR(logger_, "Failed to initialise lease shared memory %s",
                 config_.shm_name.c_str());
    return false;
  }

  frame_size_bytes_ = frame_size_bytes;
  device_index_ = device_index;
  slots_.assign(config_.slot_count, {});
  for (std::size_t i = 0; i < slots_.size(); ++i) {
    slots_[i].index = static_cast<uint32_t>(i);
  }

  if (!buffer_pool_.initialise(frame_size_bytes, device_index)) {
    destroy_slots();
    return false;
  }
  for (uint32_t i = 0; i < slots_.size(); ++i) {
    sync_slot(i);
  }

  initialised_ = true;
  return true;
}

void GpuLeasePool::reset() noexcept { destroy_slots(); }

bool GpuLeasePool::matches(uint64_t frame_size_bytes,
                           int device_index) const noexcept {
  return initialised_ && frame_size_bytes_ == frame_size_bytes &&
         device_index_ == device_index;
}

GpuLeasePool::Slot* GpuLeasePool::acquire(std::size_t subscriber_count) {
  if (!initialised_) {
    return nullptr;
  }

  auto reservation = slot_controller_.reserve_for_publish(
      static_cast<uint32_t>(subscriber_count));
  if (!reservation) {
    return nullptr;
  }
  Slot& slot = slots_[reservation->slot_id];
  slot.generation = reservation->generation;
  slot.pending_deadline = slot_controller_.pending_deadline(slot.index);
  return &slot;
}

void GpuLeasePool::reclaim_stale_pending() {
  slot_controller_.reclaim_stale_pending();
  for (auto& slot : slots_) {
    slot.pending_deadline = slot_controller_.pending_deadline(slot.index);
  }
}

bool GpuLeasePool::cancel_pending(Slot& slot) {
  if (slot_controller_.cancel({slot.index, slot.generation})) {
    slot.pending_deadline = {};
    RCLCPP_DEBUG(logger_, "Cleared pending lease for slot %u", slot.index);
    return true;
  }
  return false;
}

view::BufferView GpuLeasePool::buffer_view_from(const Slot& slot) const {
  view::BufferView view;
  view.dev_ptr = slot.device_ptr;
  view.ready_evt = slot.event;
  view.device_id = device_index_;
  view.byte_size = frame_size_bytes_;
  view.slot_id = slot.index;
  view.generation = slot.generation;
  view.shm_name = config_.shm_name;
  view.set_ipc_handles(slot.backend, slot.mem_handle.data(),
                       slot.mem_handle.size(), slot.event_handle);
  return view;
}

void GpuLeasePool::sync_slot(uint32_t slot_id) {
  const auto* resources = buffer_pool_.resources(slot_id);
  if (resources == nullptr || slot_id >= slots_.size()) {
    return;
  }
  auto& slot = slots_[slot_id];
  slot.device_ptr = resources->device_ptr;
  slot.event = resources->event;
  slot.event_handle = resources->event_handle;
  slot.backend = resources->backend;
  slot.mem_handle = resources->mem_handle;
}

void GpuLeasePool::destroy_slots() noexcept {
  buffer_pool_.reset();
  slot_controller_.reset();
  slots_.clear();
  frame_size_bytes_ = 0;
  device_index_ = -1;
  initialised_ = false;
}

}  // namespace ros2_cuda_ipc_core::cuda
