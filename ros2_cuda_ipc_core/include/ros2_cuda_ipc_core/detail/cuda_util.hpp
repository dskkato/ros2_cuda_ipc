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

/// Preserve the public Runtime error type while executing Driver API calls.
/// Driver and Runtime error enums do not have a one-to-one ABI mapping, so
/// failures intentionally collapse to cudaErrorUnknown.
cudaError_t cuda_error_from_driver(CUresult result) noexcept;

}  // namespace ros2_cuda_ipc_core::detail
