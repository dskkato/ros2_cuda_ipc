// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <cstdint>
#include <optional>
#include <string>

#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
#include "ros2_cuda_ipc_core/publisher/buffer_metadata_manager.hpp"
#include "ros2_cuda_ipc_core/publisher/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_core/transport/buffer_descriptor.hpp"

namespace ros2_cuda_ipc_core::publisher {

class GpuBufferManager;

/// Represents one active publish attempt.
///
/// The owning GpuBufferManager must outlive every PublishBlock created from
/// it. Calling GpuBufferManager::reset() invalidates block resource
/// operations. Destruction cancels an ordinary uncommitted reservation, while
/// a failed preparation intentionally retains its reservation until reset.
class PublishBlock {
 public:
  /// Move a publish block while transferring ownership of its reservation.
  PublishBlock(PublishBlock&& other) noexcept;

  /// Cancel the current reservation, if any, before taking ownership of other.
  PublishBlock& operator=(PublishBlock&& other) noexcept;

  PublishBlock(const PublishBlock&) = delete;
  PublishBlock& operator=(const PublishBlock&) = delete;

  /// Cancel an ordinary uncommitted reservation on destruction.
  ~PublishBlock() noexcept;

  /// Return the device pointer associated with the reserved block.
  ///
  /// @return Device pointer when the block is usable; nullptr otherwise.
  void* device_ptr() const noexcept;

  /// Build the descriptor, record the ready event, and commit the reservation.
  ///
  /// A successful return commits the Publisher reservation. A failed
  /// preparation leaves the block unavailable until GpuBufferManager::reset().
  /// The subsequent middleware publish result does not affect block lifecycle.
  ///
  /// @return The descriptor when preparation and commit succeed; a failed
  /// result otherwise.
  [[nodiscard]] detail::CudaResult<transport::BufferDescriptor> prepare_publish(
      CUstream stream) noexcept;

  /// Cancel an active reservation. A failed preparation cannot be cancelled.
  void cancel() noexcept;

  /// Check whether the block can still be used for publishing.
  bool valid() const noexcept;

 private:
  /// Allow the manager to construct blocks only from valid reservations.
  friend class GpuBufferManager;

  PublishBlock(GpuBufferManager* owner,
               BufferMetadataManager::Reservation reservation) noexcept;
  void move_from(PublishBlock&& other) noexcept;
  void quarantine(const char* step,
                  const detail::CudaDriverError* error = nullptr) noexcept;

  GpuBufferManager* owner_ = nullptr;
  BufferMetadataManager::Reservation reservation_{};
};

/// Owns the GPU buffer pool and shared-memory block reservations used for
/// publishing.
class GpuBufferManager {
 public:
  /// Configuration for a GPU buffer manager.
  struct Config {
    /// Retained for source compatibility; block metadata names are derived
    /// exclusively from publisher_pid and process-unique block_id.
    std::string shm_name_prefix;

    /// Number of reusable GPU buffer blocks.
    std::size_t block_count = 0;

    /// Size in bytes of each GPU buffer.
    uint64_t byte_size = 0;

    /// CUDA device index on which the buffers are allocated.
    int device_index = 0;
  };

  /// Construct a manager with the given configuration.
  explicit GpuBufferManager(Config config);

  /// Release manager-owned resources.
  ~GpuBufferManager();

  GpuBufferManager(const GpuBufferManager&) = delete;
  GpuBufferManager& operator=(const GpuBufferManager&) = delete;
  GpuBufferManager(GpuBufferManager&&) = delete;
  GpuBufferManager& operator=(GpuBufferManager&&) = delete;

  /// Initialize one shared BlockMetadata object per GPU block and the GPU pool.
  ///
  /// @return true when both pools are initialized successfully.
  bool initialise();

  /// Release pool resources and reset the manager to an uninitialized state.
  void reset() noexcept;

  /// Check whether both the buffer_ref pool and buffer pool are initialized.
  bool is_initialised() const noexcept;

  /// Return the process locator carried by every block descriptor.
  uint32_t publisher_pid() const noexcept;

  /// Reserve a block for a new publish attempt.
  /// @return A publish block when a reservation is available; std::nullopt
  /// otherwise.
  [[nodiscard]] std::optional<PublishBlock> acquire_for_publish();

 private:
  /// Allow a block to delegate resource operations to its owning manager
  /// without exposing those operations as part of the public manager API.
  friend class PublishBlock;

  void* device_ptr(
      const BufferMetadataManager::Reservation& reservation) const noexcept;
  detail::CudaResult<void> record_ready(
      const BufferMetadataManager::Reservation& reservation,
      CUstream stream) noexcept;
  std::optional<transport::BufferDescriptor> try_build_descriptor(
      const BufferMetadataManager::Reservation& reservation) const noexcept;
  bool commit(const BufferMetadataManager::Reservation& reservation) noexcept;
  bool cancel(const BufferMetadataManager::Reservation& reservation) noexcept;

  Config config_;
  GpuBufferPool buffer_pool_;
  BufferMetadataManager buffer_metadata_manager_;
};

}  // namespace ros2_cuda_ipc_core::publisher
