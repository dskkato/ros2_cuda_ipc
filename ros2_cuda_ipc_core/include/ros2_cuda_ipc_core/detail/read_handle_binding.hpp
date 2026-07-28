// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include "ros2_cuda_ipc_core/subscriber/read_handle.hpp"

namespace ros2_cuda_ipc_core::subscriber::detail {

// Internal hook used by typed adapters.  It deliberately exposes neither the
// imported-resource type nor the publication-lease type.
struct ReadHandleAccess {
  static bool bind(ReadHandle& handle, CUstream consumer_stream) noexcept;
  static void unbind(ReadHandle& handle) noexcept;
};

}  // namespace ros2_cuda_ipc_core::subscriber::detail
