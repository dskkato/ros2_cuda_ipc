// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/publisher/buffer_metadata_manager.hpp"

#include <limits.h>
#include <rcutils/logging_macros.h>
#include <sys/mman.h>
#include <uuid/uuid.h>

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"

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

BufferMetadataManager::BufferMetadataManager(std::string shm_name_prefix,
                                             std::size_t block_count)
    : shm_name_prefix_(std::move(shm_name_prefix)), block_count_(block_count) {}

BufferMetadataManager::~BufferMetadataManager() { reset(); }

bool BufferMetadataManager::initialise() {
  reset();
  if (block_count_ == 0 ||
      block_count_ > std::numeric_limits<uint32_t>::max()) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.publisher.buffer_metadata_manager",
        "Invalid block_count: %zu", block_count_);
    return false;
  }
  if (!valid_prefix(shm_name_prefix_)) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.publisher.buffer_metadata_manager",
        "Invalid shared-memory name prefix: %s", shm_name_prefix_.c_str());
    return false;
  }
  auto [instance_id, instance_name] = make_instance_identity(shm_name_prefix_);
  if (instance_name.size() > NAME_MAX) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.publisher.buffer_metadata_manager",
        "Generated shared-memory name is too long");
    return false;
  }
  auto mapping = buffer_metadata::BufferMetadata::create(
      instance_name, instance_id, static_cast<uint32_t>(block_count_));
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

void BufferMetadataManager::reset() noexcept {
  std::string owned_name;
  std::shared_ptr<buffer_metadata::BufferMetadata> owned_mapping;
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
    RCUTILS_LOG_WARN_NAMED(
        "ros2_cuda_ipc_core.publisher.buffer_metadata_manager",
        "Failed to unlink buffer metadata shared memory name=%s",
        owned_name.c_str());
  }
  // Keep the mapping alive until after unlink. Reservations may still own it.
  owned_mapping.reset();
}

bool BufferMetadataManager::is_initialised() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return initialised_;
}

std::optional<BufferMetadataManager::Reservation>
BufferMetadataManager::reserve_for_publish() {
  std::string shm_name;
  PublisherInstanceId instance_id{};
  std::shared_ptr<buffer_metadata::BufferMetadata> mapping;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialised_) {
      return std::nullopt;
    }
    shm_name = shm_name_;
    instance_id = publisher_instance_id_;
    mapping = mapping_;
  }
  const auto reservation =
      buffer_metadata::BufferRef::reserve_for_publish(mapping);
  if (!reservation) {
    return std::nullopt;
  }
  if (reservation->block_id >= block_count_) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.publisher.buffer_metadata_manager",
        "Buffer metadata shared-memory capacity changed unexpectedly: "
        "block=%u configured_count=%zu",
        reservation->block_id, block_count_);
    const bool rolled_back = buffer_metadata::BufferRef::cancel_publish(
        reservation->mapping, reservation->block_id, reservation->uid);
    if (!rolled_back) {
      RCUTILS_LOG_ERROR_NAMED(
          "ros2_cuda_ipc_core.publisher.buffer_metadata_manager",
          "Failed to roll back out-of-range reservation block=%u uid=%u",
          reservation->block_id, reservation->uid);
    }
    return std::nullopt;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialised_ && shm_name_ == shm_name &&
        publisher_instance_id_ == instance_id) {
      return Reservation{reservation->mapping, reservation->block_id,
                         reservation->uid, shm_name, instance_id};
    }
  }

  const bool rolled_back = buffer_metadata::BufferRef::cancel_publish(
      reservation->mapping, reservation->block_id, reservation->uid);
  if (!rolled_back) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.publisher.buffer_metadata_manager",
        "Failed to roll back reservation block=%u uid=%u",
        reservation->block_id, reservation->uid);
  }
  return std::nullopt;
}

bool BufferMetadataManager::commit(const Reservation& reservation) noexcept {
  const bool committed = buffer_metadata::BufferRef::commit_publish(
      reservation.mapping, reservation.block_id, reservation.uid);
  if (!committed) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.publisher.buffer_metadata_manager",
        "Failed to commit reservation block=%u uid=%u", reservation.block_id,
        reservation.uid);
  }
  return committed;
}

bool BufferMetadataManager::cancel(const Reservation& reservation) noexcept {
  if (reservation.block_id >= block_count_) {
    return false;
  }
  const bool cancelled = buffer_metadata::BufferRef::cancel_publish(
      reservation.mapping, reservation.block_id, reservation.uid);
  if (!cancelled) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.publisher.buffer_metadata_manager",
        "Failed to cancel reservation block=%u uid=%u", reservation.block_id,
        reservation.uid);
  }
  return cancelled;
}

std::string BufferMetadataManager::shm_name() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return shm_name_;
}

PublisherInstanceId BufferMetadataManager::publisher_instance_id() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return publisher_instance_id_;
}

}  // namespace ros2_cuda_ipc_core::publisher
