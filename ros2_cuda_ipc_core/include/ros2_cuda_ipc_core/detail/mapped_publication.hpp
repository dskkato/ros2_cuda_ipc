// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <memory>

#include "ros2_cuda_ipc_core/backend/vmm_fd_memory_importer.hpp"
#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"

namespace ros2_cuda_ipc_core::subscriber::detail {

struct ReadHandleFactory;

/// Owns a mapped publication before it is associated with a consumer stream.
///
/// This is deliberately not a ReadHandle.  In particular, it has no
/// completion event and does not represent a GPU read operation yet.
class MappedPublication {
 public:
  ~MappedPublication() noexcept;

  MappedPublication(MappedPublication&& other) noexcept;
  MappedPublication& operator=(MappedPublication&& other) noexcept;
  MappedPublication(const MappedPublication&) = delete;
  MappedPublication& operator=(const MappedPublication&) = delete;

  bool valid() const noexcept;
  void* device_ptr() const noexcept;
  std::size_t byte_size() const noexcept;
  int device_id() const noexcept;

 private:
  MappedPublication(std::shared_ptr<const backend::ImportedResources> resource,
                    std::unique_ptr<buffer_metadata::BufferRef> buffer_ref,
                    std::size_t byte_size, int device_id) noexcept;

  std::shared_ptr<const backend::ImportedResources> resource_;
  std::unique_ptr<buffer_metadata::BufferRef> buffer_ref_;
  std::size_t byte_size_ = 0;
  int device_id_ = -1;

  friend struct ReadHandleFactory;
};

}  // namespace ros2_cuda_ipc_core::subscriber::detail
