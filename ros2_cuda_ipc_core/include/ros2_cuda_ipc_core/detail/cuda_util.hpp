// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>
#include <cuda_runtime_api.h>

#include <string>

namespace ros2_cuda_ipc_core::detail {

/// Convert a Runtime error for publisher-side callers.  It is inline so a
/// Subscriber-only binary does not acquire a libcudart dependency merely by
/// linking this common utility header.
inline std::string cuda_error_to_string(cudaError_t err) {
  return std::string(cudaGetErrorName(err)) + ": " + cudaGetErrorString(err);
}

/// Convert a CUDA driver API error into "<name>: <message>".
std::string cu_result_to_string(CUresult result);

}  // namespace ros2_cuda_ipc_core::detail
