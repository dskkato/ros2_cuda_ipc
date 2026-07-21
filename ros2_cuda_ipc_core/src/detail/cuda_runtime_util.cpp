// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/detail/cuda_runtime_util.hpp"

namespace ros2_cuda_ipc_core::detail {

std::string cuda_error_to_string(cudaError_t error) {
  return std::string(cudaGetErrorName(error)) + ": " +
         cudaGetErrorString(error);
}

}  // namespace ros2_cuda_ipc_core::detail
