// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <cstddef>
#include <memory>
#include <optional>

#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"
#include "ros2_cuda_ipc_core/lease/lease_handle.hpp"
#include "ros2_cuda_ipc_core/subscriber/read_handle.hpp"

namespace ros2_cuda_ipc_core::subscriber::detail {

// Source-only construction hook.  Keeping this declaration out of the
// installed headers prevents the lease and import types from becoming part of
// the subscriber API.
struct ReadHandleFactory {
  static std::optional<ReadHandle> make(
      std::shared_ptr<const backend::ImportedResources> resource,
      std::unique_ptr<lease::LeaseHandle> lease, std::size_t byte_size,
      int device_id, CUstream consumer_stream);
  static std::optional<ReadHandle> make_unbound(
      std::shared_ptr<const backend::ImportedResources> resource,
      std::unique_ptr<lease::LeaseHandle> lease, std::size_t byte_size,
      int device_id);
};

}  // namespace ros2_cuda_ipc_core::subscriber::detail
