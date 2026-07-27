// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <cstdint>
#include <optional>
#include <string>

#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
#include "ros2_cuda_ipc_core/publisher/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_core/publisher/lease_manager.hpp"
#include "ros2_cuda_ipc_core/transport/buffer_descriptor.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::publisher {

class GpuBufferManager;
class PublishSlot;

/// Provides scoped write access to one reserved GPU buffer.
///
/// The associated PublishSlot must outlive this handle and must not be moved
/// while the handle is active. Destruction records the ready event on the
/// borrowed CUDA stream.
class WriteHandle {
 public:
  WriteHandle(WriteHandle&& other) noexcept;
  WriteHandle& operator=(WriteHandle&& other) noexcept;

  WriteHandle(const WriteHandle&) = delete;
  WriteHandle& operator=(const WriteHandle&) = delete;

  ~WriteHandle() noexcept;

  /// Return the device pointer while this handle owns the write scope.
  void* data() const noexcept;

  /// Return the device pointer cast to the requested element type.
  template <typename T>
  T* data() const noexcept {
    return static_cast<T*>(data());
  }

  /// Check whether this handle owns an active write scope.
  bool valid() const noexcept { return slot_ != nullptr; }

 private:
  friend class PublishSlot;

  WriteHandle(PublishSlot* slot, CUstream stream) noexcept;
  void finish() noexcept;

  PublishSlot* slot_ = nullptr;
  CUstream stream_ = nullptr;
};

/// Represents one active publish attempt.
///
/// The owning GpuBufferManager must outlive every PublishSlot created from
/// it. Calling GpuBufferManager::reset() invalidates slot resource
/// operations, but slot destruction can still release its shared-memory
/// reservation while the manager object remains alive.
class PublishSlot {
 public:
  /// Move a publish slot while transferring ownership of its reservation.
  PublishSlot(PublishSlot&& other) noexcept;

  /// Complete the current reservation, if any, before taking ownership of
  /// other.
  PublishSlot& operator=(PublishSlot&& other) noexcept;

  PublishSlot(const PublishSlot&) = delete;
  PublishSlot& operator=(const PublishSlot&) = delete;

  /// Record the publish timestamp and release the Publisher reservation.
  ~PublishSlot();

  /// Begin one scoped write on the given stream.
  ///
  /// The returned handle borrows this PublishSlot. This slot must outlive the
  /// handle and must not be moved while the handle is active.
  ///
  /// @return A write handle when the slot is reserved; std::nullopt otherwise.
  std::optional<WriteHandle> write(CUstream stream) noexcept;

  /// Build the transport descriptor after the ready event has been recorded.
  ///
  /// @return Descriptor when the slot is ready; std::nullopt otherwise.
  std::optional<transport::BufferDescriptor> descriptor() const;

  /// Return the ready-event recording error, when recording failed.
  std::optional<detail::CudaDriverError> ready_error() const noexcept;

  /// Check whether the slot can still be used for publishing.
  bool valid() const noexcept;

 private:
  /// Allow the manager to construct slots only from valid reservations.
  friend class GpuBufferManager;
  friend class WriteHandle;

  enum class State {
    reserved,
    writing,
    ready_recorded,
    ready_failed,
    released,
    moved_from
  };

  PublishSlot(GpuBufferManager* owner,
              LeaseManager::Reservation reservation) noexcept;
  void move_from(PublishSlot&& other) noexcept;
  void* write_data() const noexcept;
  void finish_write(CUstream stream) noexcept;
  void release() noexcept;

  GpuBufferManager* owner_ = nullptr;
  LeaseManager::Reservation reservation_{};
  State state_ = State::moved_from;
  std::optional<detail::CudaDriverError> ready_error_;
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
  std::optional<PublishSlot> acquire_for_publish();

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

  Config config_;
  GpuBufferPool buffer_pool_;
  LeaseManager lease_manager_;
};

}  // namespace ros2_cuda_ipc_core::publisher
