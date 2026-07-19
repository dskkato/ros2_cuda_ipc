// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/publisher/lease_manager.hpp"

#include <limits.h>
#include <sys/mman.h>
#include <uuid/uuid.h>

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/lease/lease_handle.hpp"

namespace ros2_cuda_ipc_core::publisher {
namespace {
using Clock = std::chrono::steady_clock;

bool deadline_reached(const Clock::time_point& deadline,
                      const Clock::time_point& now) {
  return deadline.time_since_epoch().count() != 0 && now >= deadline;
}

bool valid_prefix(const std::string& prefix) {
  return prefix.size() > 1 && prefix.front() == '/' &&
         prefix.find('/', 1) == std::string::npos;
}

std::pair<PublisherInstanceId, std::string> make_instance_identity(
    const std::string& prefix) {
  uuid_t uuid;
  uuid_generate(uuid);
  PublisherInstanceId id{};
  std::copy(std::begin(uuid), std::end(uuid), id.begin());
  char text[37]{};
  uuid_unparse_lower(uuid, text);
  return {id, prefix + "_" + text};
}
}  // namespace

LeaseManager::LeaseManager(std::string shm_name_prefix, std::size_t slot_count,
                           std::chrono::milliseconds pending_ttl,
                           rclcpp::Logger logger)
    : shm_name_prefix_(std::move(shm_name_prefix)),
      slot_count_(slot_count),
      pending_ttl_(pending_ttl),
      logger_(std::move(logger)) {}

LeaseManager::~LeaseManager() { reset(); }

bool LeaseManager::initialise() {
  reset();
  if (slot_count_ == 0 || slot_count_ > std::numeric_limits<uint32_t>::max() ||
      !valid_prefix(shm_name_prefix_)) {
    RCLCPP_ERROR(logger_, "Invalid shared-memory name prefix: %s",
                 shm_name_prefix_.c_str());
    return false;
  }
  auto [instance_id, instance_name] = make_instance_identity(shm_name_prefix_);
  if (instance_name.size() > NAME_MAX) {
    RCLCPP_ERROR(logger_, "Generated shared-memory name is too long");
    return false;
  }
  std::vector<Clock::time_point> pending_deadlines(slot_count_);
  if (!lease::LeaseHandle::init(instance_name, instance_id,
                                static_cast<uint32_t>(slot_count_))) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(deadlines_mutex_);
    shm_name_ = std::move(instance_name);
    publisher_instance_id_ = instance_id;
    pending_deadlines_ = std::move(pending_deadlines);
    initialised_ = true;
  }
  return true;
}

void LeaseManager::reset() noexcept {
  std::string owned_name;
  {
    std::lock_guard<std::mutex> lock(deadlines_mutex_);
    if (initialised_) {
      owned_name = std::move(shm_name_);
    }
    shm_name_.clear();
    publisher_instance_id_ = {};
    pending_deadlines_.clear();
    initialised_ = false;
  }
  if (!owned_name.empty() && ::shm_unlink(owned_name.c_str()) != 0) {
    RCLCPP_WARN(logger_, "Failed to unlink lease shared memory name=%s",
                owned_name.c_str());
  }
}

bool LeaseManager::is_initialised() const noexcept {
  std::lock_guard<std::mutex> lock(deadlines_mutex_);
  return initialised_;
}

std::optional<LeaseManager::Reservation> LeaseManager::reserve_for_publish(
    uint32_t pending_count) {
  std::string shm_name;
  PublisherInstanceId instance_id{};
  {
    std::lock_guard<std::mutex> lock(deadlines_mutex_);
    if (!initialised_) {
      return std::nullopt;
    }
    shm_name = shm_name_;
    instance_id = publisher_instance_id_;
  }
  const auto reservation = lease::LeaseHandle::reserve_for_publish(
      shm_name, instance_id, pending_count);
  if (!reservation) {
    return std::nullopt;
  }
  if (reservation->slot_id >= slot_count_) {
    RCLCPP_ERROR(logger_,
                 "Lease shared-memory capacity changed unexpectedly: "
                 "slot=%u configured_count=%zu",
                 reservation->slot_id, slot_count_);
    const bool rolled_back = lease::LeaseHandle::cancel_pending(
        shm_name, reservation->slot_id, reservation->generation, instance_id);
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
    if (initialised_ && shm_name_ == shm_name &&
        publisher_instance_id_ == instance_id &&
        reservation->slot_id < pending_deadlines_.size()) {
      if (pending_count > 0 && pending_ttl_.count() > 0) {
        pending_deadlines_[reservation->slot_id] = Clock::now() + pending_ttl_;
      } else {
        pending_deadlines_[reservation->slot_id] = {};
      }
      return Reservation{reservation->slot_id, reservation->generation,
                         shm_name, instance_id};
    }
  }

  const bool rolled_back = lease::LeaseHandle::cancel_pending(
      shm_name, reservation->slot_id, reservation->generation, instance_id);
  if (!rolled_back) {
    RCLCPP_ERROR(logger_,
                 "Failed to roll back reservation slot=%u generation=%u",
                 reservation->slot_id, reservation->generation);
  }
  return std::nullopt;
}

bool LeaseManager::cancel(const Reservation& reservation) noexcept {
  if (reservation.slot_id >= slot_count_) {
    return false;
  }
  const bool cancelled = lease::LeaseHandle::cancel_pending(
      reservation.shm_name, reservation.slot_id, reservation.generation,
      reservation.publisher_instance_id);
  if (!cancelled) {
    RCLCPP_ERROR(logger_, "Failed to cancel reservation slot=%u generation=%u",
                 reservation.slot_id, reservation.generation);
  }
  if (cancelled) {
    std::lock_guard<std::mutex> lock(deadlines_mutex_);
    if (reservation.shm_name == shm_name_ &&
        reservation.publisher_instance_id == publisher_instance_id_ &&
        reservation.slot_id < pending_deadlines_.size()) {
      pending_deadlines_[reservation.slot_id] = {};
    }
  }
  return cancelled;
}

void LeaseManager::reclaim_stale_pending() {
  std::lock_guard<std::mutex> lock(deadlines_mutex_);
  if (!initialised_ || pending_ttl_.count() <= 0) {
    return;
  }
  const auto now = Clock::now();
  for (uint32_t slot_id = 0; slot_id < pending_deadlines_.size(); ++slot_id) {
    auto& deadline = pending_deadlines_[slot_id];
    if (!deadline_reached(deadline, now)) {
      continue;
    }
    const auto pending = lease::LeaseHandle::current_pending(
        shm_name_, publisher_instance_id_, slot_id);
    if (!pending) {
      continue;
    }
    if (*pending == 0) {
      deadline = {};
      continue;
    }
    if (lease::LeaseHandle::force_clear_pending(
            shm_name_, publisher_instance_id_, slot_id)) {
      RCLCPP_WARN(logger_,
                  "Force-cleared pending lease slot=%u after %lld ms timeout",
                  slot_id, static_cast<long long>(pending_ttl_.count()));
      deadline = {};
    }
  }
}

Clock::time_point LeaseManager::pending_deadline(
    uint32_t slot_id) const noexcept {
  std::lock_guard<std::mutex> lock(deadlines_mutex_);
  if (slot_id >= pending_deadlines_.size()) {
    return {};
  }
  return pending_deadlines_[slot_id];
}

std::string LeaseManager::shm_name() const {
  std::lock_guard<std::mutex> lock(deadlines_mutex_);
  return shm_name_;
}

PublisherInstanceId LeaseManager::publisher_instance_id() const {
  std::lock_guard<std::mutex> lock(deadlines_mutex_);
  return publisher_instance_id_;
}

}  // namespace ros2_cuda_ipc_core::publisher
