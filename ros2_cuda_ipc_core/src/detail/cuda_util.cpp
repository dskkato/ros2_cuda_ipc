// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/detail/cuda_util.hpp"

namespace ros2_cuda_ipc_core::detail {

std::string cu_result_to_string(CUresult result) {
  const char* name = nullptr;
  const char* desc = nullptr;

  // Attempt to retrieve human-readable error name and description.
  // Fall back to default strings if the CUDA calls fail or return null.
  CUresult name_result = cuGetErrorName(result, &name);
  if (name_result != CUDA_SUCCESS || !name) {
    name = "UNKNOWN";
  }

  CUresult desc_result = cuGetErrorString(result, &desc);
  if (desc_result != CUDA_SUCCESS || !desc) {
    desc = "unknown";
  }
  return std::string(name) + ": " + desc;
}

}  // namespace ros2_cuda_ipc_core::detail
