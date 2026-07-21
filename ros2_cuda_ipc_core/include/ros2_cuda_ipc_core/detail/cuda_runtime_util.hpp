// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda_runtime_api.h>

#include <string>

namespace ros2_cuda_ipc_core::detail {

std::string cuda_error_to_string(cudaError_t error);

}  // namespace ros2_cuda_ipc_core::detail
