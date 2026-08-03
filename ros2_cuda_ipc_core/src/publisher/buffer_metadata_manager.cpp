// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/publisher/buffer_metadata_manager.hpp"

#include <dirent.h>
#include <limits.h>
#include <rcutils/logging_macros.h>
#include <signal.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <limits>
#include <sstream>
#include <utility>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"

namespace ros2_cuda_ipc_core::publisher {
namespace {

std::atomic<uint32_t> next_process_block_id{0};

std::optional<uint32_t> allocate_process_block_id() {
  uint32_t current = next_process_block_id.load(std::memory_order_relaxed);
  while (current != std::numeric_limits<uint32_t>::max()) {
    if (next_process_block_id.compare_exchange_weak(
            current, current + 1, std::memory_order_relaxed,
            std::memory_order_relaxed)) {
      return current;
    }
  }
  return std::nullopt;
}

void cleanup_orphaned_metadata_objects() noexcept {
  DIR* directory = ::opendir("/dev/shm");
  if (directory == nullptr) return;
  while (const dirent* entry = ::readdir(directory)) {
    uint32_t pid = 0;
    uint32_t block_id = 0;
    char suffix = '\0';
    if (std::sscanf(entry->d_name, "ros2_cuda_ipc_%u_%u%c", &pid, &block_id,
                    &suffix) != 2 ||
        pid == 0) {
      continue;
    }
    if (::kill(static_cast<pid_t>(pid), 0) == 0 || errno != ESRCH) {
      continue;
    }
    const std::string shm_name = "/" + std::string(entry->d_name);
    if (::shm_unlink(shm_name.c_str()) != 0 && errno != ENOENT) {
      RCUTILS_LOG_WARN_NAMED(
          "ros2_cuda_ipc_core.publisher.buffer_metadata_manager",
          "Failed to clean orphaned block metadata name=%s errno=%d",
          shm_name.c_str(), errno);
    }
  }
  ::closedir(directory);
}

}  // namespace

BufferMetadataManager::BufferMetadataManager(std::size_t block_count)
    : block_count_(block_count),
      publisher_pid_(static_cast<uint32_t>(::getpid())) {}

BufferMetadataManager::~BufferMetadataManager() { reset(); }

std::string BufferMetadataManager::shm_name_for_block(uint32_t publisher_pid,
                                                      uint32_t block_id) {
  std::ostringstream name;
  name << "/ros2_cuda_ipc_" << publisher_pid << "_" << block_id;
  return name.str();
}

bool BufferMetadataManager::initialise() {
  reset();
  cleanup_orphaned_metadata_objects();
  if (block_count_ == 0 ||
      block_count_ > std::numeric_limits<uint32_t>::max()) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.publisher.buffer_metadata_manager",
        "Invalid block_count: %zu", block_count_);
    return false;
  }

  std::vector<Entry> entries;
  entries.reserve(block_count_);
  for (std::size_t index = 0; index < block_count_; ++index) {
    const auto block_id = allocate_process_block_id();
    if (!block_id) {
      RCUTILS_LOG_ERROR_NAMED(
          "ros2_cuda_ipc_core.publisher.buffer_metadata_manager",
          "Process-local block_id space exhausted");
      for (const auto& entry : entries) {
        (void)::shm_unlink(entry.shm_name.c_str());
      }
      return false;
    }
    std::string shm_name = shm_name_for_block(publisher_pid_, *block_id);
    if (shm_name.size() > NAME_MAX) {
      RCUTILS_LOG_ERROR_NAMED(
          "ros2_cuda_ipc_core.publisher.buffer_metadata_manager",
          "Generated shared-memory name is too long");
      for (const auto& entry : entries) {
        (void)::shm_unlink(entry.shm_name.c_str());
      }
      return false;
    }
    auto mapping = buffer_metadata::BufferMetadata::create(shm_name);
    if (!mapping && errno == EEXIST) {
      // Within one process block_id is never reused. Therefore an existing
      // name with our pid and newly allocated block_id can only be an object
      // left by a previous process that received the same PID.
      RCUTILS_LOG_WARN_NAMED(
          "ros2_cuda_ipc_core.publisher.buffer_metadata_manager",
          "Replacing stale block metadata after PID reuse name=%s",
          shm_name.c_str());
      const int unlink_result = ::shm_unlink(shm_name.c_str());
      if (unlink_result == 0 || errno == ENOENT) {
        mapping = buffer_metadata::BufferMetadata::create(shm_name);
      }
    }
    if (!mapping) {
      for (const auto& entry : entries) {
        (void)::shm_unlink(entry.shm_name.c_str());
      }
      return false;
    }
    entries.push_back(
        Entry{*block_id, std::move(shm_name), std::move(mapping)});
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    entries_ = std::move(entries);
    next_entry_ = 0;
    initialised_ = true;
  }
  return true;
}

void BufferMetadataManager::reset() noexcept {
  std::vector<Entry> entries;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    entries.swap(entries_);
    next_entry_ = 0;
    initialised_ = false;
  }
  for (const auto& entry : entries) {
    if (::shm_unlink(entry.shm_name.c_str()) != 0) {
      RCUTILS_LOG_WARN_NAMED(
          "ros2_cuda_ipc_core.publisher.buffer_metadata_manager",
          "Failed to unlink block metadata shared memory name=%s",
          entry.shm_name.c_str());
    }
  }
  entries.clear();
}

bool BufferMetadataManager::is_initialised() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return initialised_;
}

std::shared_ptr<buffer_metadata::BufferMetadata>
BufferMetadataManager::metadata_for_pool_index(std::size_t pool_index) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialised_ || pool_index >= entries_.size()) return nullptr;
  return entries_[pool_index].mapping;
}

std::optional<uint32_t> BufferMetadataManager::block_id_for_pool_index(
    std::size_t pool_index) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialised_ || pool_index >= entries_.size()) return std::nullopt;
  return entries_[pool_index].block_id;
}

std::optional<BufferMetadataManager::Reservation>
BufferMetadataManager::reserve_for_publish() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialised_ || entries_.empty()) return std::nullopt;
    for (std::size_t offset = 0; offset < entries_.size(); ++offset) {
      const std::size_t index = (next_entry_ + offset) % entries_.size();
      auto& candidate = entries_[index];
      auto reservation =
          buffer_metadata::BufferRef::reserve_for_publish(candidate.mapping);
      if (!reservation) continue;
      next_entry_ = (index + 1) % entries_.size();
      return Reservation{reservation->mapping, candidate.block_id,
                         reservation->uid,     publisher_pid_,
                         candidate.shm_name,   index};
    }
  }
  return std::nullopt;
}

bool BufferMetadataManager::commit(const Reservation& reservation) noexcept {
  const bool committed = buffer_metadata::BufferRef::commit_publish(
      reservation.mapping, reservation.uid);
  if (!committed) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.publisher.buffer_metadata_manager",
        "Failed to commit block reservation block=%u uid=%llu",
        reservation.block_id, static_cast<unsigned long long>(reservation.uid));
  }
  return committed;
}

bool BufferMetadataManager::cancel(const Reservation& reservation) noexcept {
  const bool cancelled = buffer_metadata::BufferRef::cancel_publish(
      reservation.mapping, reservation.uid);
  if (!cancelled) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.publisher.buffer_metadata_manager",
        "Failed to cancel block reservation block=%u uid=%llu",
        reservation.block_id, static_cast<unsigned long long>(reservation.uid));
  }
  return cancelled;
}

}  // namespace ros2_cuda_ipc_core::publisher
