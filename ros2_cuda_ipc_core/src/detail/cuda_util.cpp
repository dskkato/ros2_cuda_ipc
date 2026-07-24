// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/detail/cuda_util.hpp"

#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"

namespace ros2_cuda_ipc_core::detail {

std::string cu_result_to_string(CUresult result) {
  return CudaDriverError(result).to_string();
}

}  // namespace ros2_cuda_ipc_core::detail
