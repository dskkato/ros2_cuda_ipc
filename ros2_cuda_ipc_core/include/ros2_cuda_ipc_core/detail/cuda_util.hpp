// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <string>

namespace ros2_cuda_ipc_core::detail {

/// Convert a CUDA driver API error into "<name>: <message>".
std::string cu_result_to_string(CUresult result);

}  // namespace ros2_cuda_ipc_core::detail
