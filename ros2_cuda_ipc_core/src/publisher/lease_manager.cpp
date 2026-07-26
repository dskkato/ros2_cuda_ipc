// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/publisher/lease_manager.hpp"

#include <limits.h>
#include <rcutils/logging_macros.h>
#include <sys/mman.h>
#include <uuid/uuid.h>

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "ros2_cuda_ipc_core/lease/lease_handle.hpp"

namespace ros2_cuda_ipc_core::publisher {
namespace {
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

LeaseManager::LeaseManager(std::string shm_name_prefix, std::size_t slot_count)
    : shm_name_prefix_(std::move(shm_name_prefix)), slot_count_(slot_count) {}

LeaseManager::~LeaseManager() { reset(); }

bool LeaseManager::initialise() {
  reset();
  if (slot_count_ == 0 || slot_count_ > std::numeric_limits<uint32_t>::max()) {
    RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.publisher.lease_manager",
                            "Invalid slot_count: %zu", slot_count_);
    return false;
  }
  if (!valid_prefix(shm_name_prefix_)) {
    RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.publisher.lease_manager",
                            "Invalid shared-memory name prefix: %s",
                            shm_name_prefix_.c_str());
    return false;
  }
  auto [instance_id, instance_name] = make_instance_identity(shm_name_prefix_);
  if (instance_name.size() > NAME_MAX) {
    RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.publisher.lease_manager",
                            "Generated shared-memory name is too long");
    return false;
  }
  auto mapping = lease::LeaseMapping::create(
      instance_name, instance_id, static_cast<uint32_t>(slot_count_));
  if (!mapping) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    shm_name_ = std::move(instance_name);
    publisher_instance_id_ = instance_id;
    mapping_ = std::move(mapping);
    initialised_ = true;
  }
  return true;
}

void LeaseManager::reset() noexcept {
  std::string owned_name;
  std::shared_ptr<lease::LeaseMapping> owned_mapping;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialised_) {
      owned_name = std::move(shm_name_);
      owned_mapping = std::move(mapping_);
    }
    shm_name_.clear();
    publisher_instance_id_ = {};
    mapping_.reset();
    initialised_ = false;
  }
  if (!owned_name.empty() && ::shm_unlink(owned_name.c_str()) != 0) {
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.publisher.lease_manager",
                           "Failed to unlink lease shared memory name=%s",
                           owned_name.c_str());
  }
  // Keep the mapping alive until after unlink. Reservations may still own it.
  owned_mapping.reset();
}

bool LeaseManager::is_initialised() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return initialised_;
}

std::optional<LeaseManager::Reservation> LeaseManager::reserve_for_publish() {
  std::string shm_name;
  PublisherInstanceId instance_id{};
  std::shared_ptr<lease::LeaseMapping> mapping;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialised_) {
      return std::nullopt;
    }
    shm_name = shm_name_;
    instance_id = publisher_instance_id_;
    mapping = mapping_;
  }
  const auto reservation = lease::LeaseHandle::reserve_for_publish(mapping);
  if (!reservation) {
    return std::nullopt;
  }
  if (reservation->slot_id >= slot_count_) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.publisher.lease_manager",
        "Lease shared-memory capacity changed unexpectedly: "
        "slot=%u configured_count=%zu",
        reservation->slot_id, slot_count_);
    const bool rolled_back = lease::LeaseHandle::cancel_publish(
        reservation->mapping, reservation->slot_id, reservation->generation);
    if (!rolled_back) {
      RCUTILS_LOG_ERROR_NAMED(
          "ros2_cuda_ipc_core.publisher.lease_manager",
          "Failed to roll back out-of-range reservation slot=%u generation=%u",
          reservation->slot_id, reservation->generation);
    }
    return std::nullopt;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialised_ && shm_name_ == shm_name &&
        publisher_instance_id_ == instance_id) {
      return Reservation{reservation->mapping, reservation->slot_id,
                         reservation->generation, shm_name, instance_id};
    }
  }

  const bool rolled_back = lease::LeaseHandle::cancel_publish(
      reservation->mapping, reservation->slot_id, reservation->generation);
  if (!rolled_back) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.publisher.lease_manager",
        "Failed to roll back reservation slot=%u generation=%u",
        reservation->slot_id, reservation->generation);
  }
  return std::nullopt;
}

bool LeaseManager::commit(const Reservation& reservation) noexcept {
  const bool committed = lease::LeaseHandle::commit_publish(
      reservation.mapping, reservation.slot_id, reservation.generation);
  if (!committed) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.publisher.lease_manager",
        "Failed to commit reservation slot=%u generation=%u",
        reservation.slot_id, reservation.generation);
  }
  return committed;
}

bool LeaseManager::cancel(const Reservation& reservation) noexcept {
  if (reservation.slot_id >= slot_count_) {
    return false;
  }
  const bool cancelled = lease::LeaseHandle::cancel_publish(
      reservation.mapping, reservation.slot_id, reservation.generation);
  if (!cancelled) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.publisher.lease_manager",
        "Failed to cancel reservation slot=%u generation=%u",
        reservation.slot_id, reservation.generation);
  }
  return cancelled;
}

std::string LeaseManager::shm_name() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return shm_name_;
}

PublisherInstanceId LeaseManager::publisher_instance_id() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return publisher_instance_id_;
}

}  // namespace ros2_cuda_ipc_core::publisher
