// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/publisher/slot_controller.hpp"

#include <stdexcept>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/lease/lease_handle.hpp"

namespace ros2_cuda_ipc_core::publisher {
namespace {
rclcpp::Clock::SharedPtr require_clock(rclcpp::Clock::SharedPtr clock) {
  if (!clock) {
    throw std::invalid_argument("SlotController requires a clock");
  }
  return clock;
}

rclcpp::Time zero_time(rcl_clock_type_t clock_type) {
  return rclcpp::Time(int64_t{0}, clock_type);
}

bool deadline_reached(const rclcpp::Time& deadline, const rclcpp::Time& now) {
  return deadline.nanoseconds() != 0 && now >= deadline;
}
}  // namespace

SlotController::SlotController(std::string shm_name, std::size_t slot_count,
                               std::chrono::milliseconds pending_ttl,
                               rclcpp::Logger logger,
                               rclcpp::Clock::SharedPtr clock)
    : shm_name_(std::move(shm_name)),
      slot_count_(slot_count),
      pending_ttl_(pending_ttl),
      clock_(require_clock(std::move(clock))),
      logger_(std::move(logger)) {}

bool SlotController::initialise() {
  reset();
  if (slot_count_ == 0 || !lease::LeaseHandle::init(
                              shm_name_, static_cast<uint32_t>(slot_count_))) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(deadlines_mutex_);
    pending_deadlines_.assign(slot_count_, zero_time(clock_->get_clock_type()));
    initialised_ = true;
  }
  return true;
}

void SlotController::reset() noexcept {
  std::lock_guard<std::mutex> lock(deadlines_mutex_);
  pending_deadlines_.clear();
  initialised_ = false;
}

bool SlotController::is_initialised() const noexcept {
  std::lock_guard<std::mutex> lock(deadlines_mutex_);
  return initialised_;
}

std::optional<SlotController::Reservation> SlotController::reserve_for_publish(
    uint32_t pending_count) {
  {
    std::lock_guard<std::mutex> lock(deadlines_mutex_);
    if (!initialised_) {
      return std::nullopt;
    }
  }
  const auto reservation =
      lease::LeaseHandle::reserve_for_publish(shm_name_, pending_count);
  if (!reservation) {
    return std::nullopt;
  }
  if (reservation->slot_id >= slot_count_) {
    RCLCPP_ERROR(logger_,
                 "Lease shared-memory capacity changed unexpectedly: "
                 "slot=%u configured_count=%zu",
                 reservation->slot_id, slot_count_);
    const bool rolled_back = lease::LeaseHandle::cancel_pending(
        shm_name_, reservation->slot_id, reservation->generation);
    if (!rolled_back) {
      RCLCPP_ERROR(logger_,
                   "Failed to roll back out-of-range reservation slot=%u "
                   "generation=%u",
                   reservation->slot_id, reservation->generation);
    }
    return std::nullopt;
  }
  {
    std::lock_guard<std::mutex> lock(deadlines_mutex_);
    if (initialised_ && reservation->slot_id < pending_deadlines_.size()) {
      if (pending_count > 0 && pending_ttl_.count() > 0) {
        pending_deadlines_[reservation->slot_id] =
            clock_->now() + rclcpp::Duration(pending_ttl_);
      } else {
        pending_deadlines_[reservation->slot_id] =
            zero_time(clock_->get_clock_type());
      }
      return Reservation{reservation->slot_id, reservation->generation};
    }
  }

  const bool rolled_back = lease::LeaseHandle::cancel_pending(
      shm_name_, reservation->slot_id, reservation->generation);
  if (!rolled_back) {
    RCLCPP_ERROR(logger_,
                 "Failed to roll back reservation slot=%u generation=%u",
                 reservation->slot_id, reservation->generation);
  }
  return std::nullopt;
}

bool SlotController::cancel(const Reservation& reservation) noexcept {
  if (reservation.slot_id >= slot_count_) {
    return false;
  }
  const bool cancelled = lease::LeaseHandle::cancel_pending(
      shm_name_, reservation.slot_id, reservation.generation);
  if (!cancelled) {
    RCLCPP_ERROR(logger_, "Failed to cancel reservation slot=%u generation=%u",
                 reservation.slot_id, reservation.generation);
  }
  if (cancelled) {
    std::lock_guard<std::mutex> lock(deadlines_mutex_);
    if (reservation.slot_id < pending_deadlines_.size()) {
      pending_deadlines_[reservation.slot_id] =
          zero_time(clock_->get_clock_type());
    }
  }
  return cancelled;
}

void SlotController::reclaim_stale_pending() {
  std::lock_guard<std::mutex> lock(deadlines_mutex_);
  if (!initialised_ || pending_ttl_.count() <= 0) {
    return;
  }
  const auto now = clock_->now();
  for (uint32_t slot_id = 0; slot_id < pending_deadlines_.size(); ++slot_id) {
    auto& deadline = pending_deadlines_[slot_id];
    if (!deadline_reached(deadline, now)) {
      continue;
    }
    const auto pending =
        lease::LeaseHandle::current_pending(shm_name_, slot_id);
    if (!pending) {
      continue;
    }
    if (*pending == 0) {
      deadline = zero_time(clock_->get_clock_type());
      continue;
    }
    if (lease::LeaseHandle::force_clear_pending(shm_name_, slot_id)) {
      RCLCPP_WARN(logger_,
                  "Force-cleared pending lease slot=%u after %lld ms timeout",
                  slot_id, static_cast<long long>(pending_ttl_.count()));
      deadline = zero_time(clock_->get_clock_type());
    }
  }
}

rclcpp::Time SlotController::pending_deadline(uint32_t slot_id) const noexcept {
  std::lock_guard<std::mutex> lock(deadlines_mutex_);
  if (slot_id >= pending_deadlines_.size()) {
    return zero_time(clock_->get_clock_type());
  }
  return pending_deadlines_[slot_id];
}

}  // namespace ros2_cuda_ipc_core::publisher
