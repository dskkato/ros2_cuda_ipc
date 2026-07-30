// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <memory>
#include <optional>

#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"
#include "ros2_cuda_ipc_core/detail/mapped_publication.hpp"
#include "ros2_cuda_ipc_core/lease/lease_handle.hpp"
#include "ros2_cuda_ipc_core/subscriber/read_handle.hpp"

namespace ros2_cuda_ipc_core::subscriber::detail {

/// Internal construction hook used by the core's mappers and typed adapters.
struct ReadHandleFactory {
  static std::unique_ptr<MappedPublication> make_publication(
      std::shared_ptr<const backend::ImportedResources> resource,
      std::unique_ptr<lease::LeaseHandle> lease, std::size_t byte_size,
      int device_id);

  /// Enqueue the producer wait and commit the publication into a ReadHandle.
  /// The publication retains ownership if any operation before the wait
  /// succeeds, or if enqueueing the wait fails.
  static std::optional<ReadHandle> make_bound(MappedPublication& publication,
                                              CUstream consumer_stream);
};

}  // namespace ros2_cuda_ipc_core::subscriber::detail
