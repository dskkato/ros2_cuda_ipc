// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda_runtime_api.h>

#include <stdexcept>
#include <string>

#include "ros2_cuda_ipc_core/cuda/cuda_util.hpp"

namespace multi_process_image_fanout {

inline void throw_on_cuda_error(cudaError_t err, const char* what) {
  if (err == cudaSuccess) {
    return;
  }

  throw std::runtime_error(std::string(what) + ": " +
                           ros2_cuda_ipc_core::cuda::cuda_error_to_string(err));
}

}  // namespace multi_process_image_fanout
