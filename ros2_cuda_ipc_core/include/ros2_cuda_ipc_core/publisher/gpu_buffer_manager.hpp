// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
#include "ros2_cuda_ipc_core/publisher/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_core/publisher/lease_manager.hpp"
#include "ros2_cuda_ipc_core/transport/buffer_descriptor.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::publisher {

class GpuBufferManager;

enum class PreparePublishErrorCode {
  kInvalidState,
  kReadyEventRecordFailed,
  kDescriptorCreationFailed,
  kReservationCommitFailed,
};

class PreparePublishError {
 public:
  PreparePublishError(
      PreparePublishErrorCode code,
      std::optional<detail::CudaDriverError> cuda_error = std::nullopt)
      : code_(code), cuda_error_(std::move(cuda_error)) {}

  PreparePublishErrorCode code() const noexcept { return code_; }

  const std::optional<detail::CudaDriverError>& cuda_error() const noexcept {
    return cuda_error_;
  }

  std::string to_string() const;

 private:
  PreparePublishErrorCode code_;
  std::optional<detail::CudaDriverError> cuda_error_;
};

template <typename T>
class [[nodiscard]] PreparePublishResult {
 public:
  static PreparePublishResult success(T value) {
    PreparePublishResult result;
    result.value_.emplace(std::move(value));
    return result;
  }

  static PreparePublishResult failure(PreparePublishError error) {
    PreparePublishResult result;
    result.error_.emplace(std::move(error));
    return result;
  }

  explicit operator bool() const noexcept { return value_.has_value(); }

  T& value() & { return value_.value(); }
  const T& value() const& { return value_.value(); }
  T&& value() && { return std::move(value_.value()); }

  PreparePublishError& error() & { return error_.value(); }
  const PreparePublishError& error() const& { return error_.value(); }

 private:
  PreparePublishResult() = default;

  std::optional<T> value_;
  std::optional<PreparePublishError> error_;
};

/// Represents one active publish attempt.
///
/// The owning GpuBufferManager must outlive every PublishSlot created from
/// it. Calling GpuBufferManager::reset() invalidates slot resource
/// operations. Destruction cancels ordinary uncommitted reservations, while a
/// quarantined slot intentionally retains its reservation until reset.
class PublishSlot {
 public:
  /// Move a publish slot while transferring ownership of its reservation.
  PublishSlot(PublishSlot&& other) noexcept;

  /// Cancel the current reservation, if any, before taking ownership of other.
  PublishSlot& operator=(PublishSlot&& other) noexcept;

  PublishSlot(const PublishSlot&) = delete;
  PublishSlot& operator=(const PublishSlot&) = delete;

  /// Cancel an ordinary uncommitted reservation on destruction.
  ~PublishSlot() noexcept;

  /// Return the device pointer associated with the reserved slot.
  ///
  /// @return Device pointer when the slot is usable; nullptr otherwise.
  void* device_ptr() const noexcept;

  /// Record the ready event, build the descriptor, and commit the reservation.
  ///
  /// This is the normal publishing path. A successful return marks the slot
  /// as published before the caller hands the descriptor to middleware.
  /// Callers should publish the returned descriptor immediately.
  ///
  /// @return The descriptor when preparation and commit succeed; a structured
  /// publisher preparation error otherwise.
  [[nodiscard]] PreparePublishResult<transport::BufferDescriptor>
  prepare_publish(CUstream stream) noexcept;

  /// Cancel the reservation when it has not been committed or quarantined.
  void cancel() noexcept;

  /// Check whether the slot can still be used for publishing.
  bool valid() const noexcept;

 private:
  /// Allow the manager to construct slots only from valid reservations.
  friend class GpuBufferManager;

  enum class State {
    reserved,
    ready_recorded,
    committed,
    quarantined,
    cancelled,
    moved_from
  };

  PublishSlot(GpuBufferManager* owner,
              LeaseManager::Reservation reservation) noexcept;
  void move_from(PublishSlot&& other) noexcept;
  detail::CudaResult<void> record_ready(CUstream stream) noexcept;
  std::optional<transport::BufferDescriptor> descriptor() const;
  bool commit_publish() noexcept;
  void quarantine(const detail::CudaDriverError& error) noexcept;

  GpuBufferManager* owner_ = nullptr;
  LeaseManager::Reservation reservation_{};
  State state_ = State::moved_from;
};

/// Owns the GPU buffer pool and shared-memory slot reservations used for
/// publishing.
class GpuBufferManager {
 public:
  /// Configuration for a GPU buffer manager.
  struct Config {
    /// Prefix used to create a publisher-instance-specific shared-memory name.
    std::string shm_name_prefix;

    /// Number of reusable GPU buffer slots.
    std::size_t slot_count = 0;

    /// Size in bytes of each GPU buffer.
    uint64_t byte_size = 0;

    /// CUDA device index on which the buffers are allocated.
    int device_index = 0;

    /// Memory backend used to allocate and export the buffers.
    transport::MemoryBackendKind backend =
        transport::MemoryBackendKind::CUDA_IPC;
  };

  /// Construct a manager with the given configuration.
  explicit GpuBufferManager(Config config);

  /// Release manager-owned resources.
  ~GpuBufferManager();

  GpuBufferManager(const GpuBufferManager&) = delete;
  GpuBufferManager& operator=(const GpuBufferManager&) = delete;
  GpuBufferManager(GpuBufferManager&&) = delete;
  GpuBufferManager& operator=(GpuBufferManager&&) = delete;

  /// Initialize the shared-memory lease pool and GPU buffer pool.
  ///
  /// @return true when both pools are initialized successfully.
  bool initialise();

  /// Release pool resources and reset the manager to an uninitialized state.
  void reset() noexcept;

  /// Check whether both the lease pool and buffer pool are initialized.
  bool is_initialised() const noexcept;

  /// Return the current instance-specific shared-memory name.
  std::string shm_name() const;

  /// Return the current publisher instance identity.
  PublisherInstanceId publisher_instance_id() const;

  /// Reserve a slot for a new publish attempt.
  /// @return A publish slot when a reservation is available; std::nullopt
  /// otherwise.
  [[nodiscard]] std::optional<PublishSlot> acquire_for_publish();

 private:
  /// Allow a slot to delegate resource operations to its owning manager
  /// without exposing those operations as part of the public manager API.
  friend class PublishSlot;

  void* device_ptr(const LeaseManager::Reservation& reservation) const noexcept;
  detail::CudaResult<void> record_ready(
      const LeaseManager::Reservation& reservation, CUstream stream) noexcept;
  std::optional<transport::BufferDescriptor> descriptor(
      const LeaseManager::Reservation& reservation) const;
  bool commit(const LeaseManager::Reservation& reservation) noexcept;
  bool cancel(const LeaseManager::Reservation& reservation) noexcept;

  Config config_;
  GpuBufferPool buffer_pool_;
  LeaseManager lease_manager_;
};

}  // namespace ros2_cuda_ipc_core::publisher
