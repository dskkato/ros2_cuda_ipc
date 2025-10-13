#pragma once

#include <cuda_runtime_api.h>

#include <string>

namespace ros2_cuda_ipc_core::cuda {

/// Convert a CUDA error code into a descriptive "<name>: <message>" string.
std::string cuda_error_to_string(cudaError_t err);

}  // namespace ros2_cuda_ipc_core::cuda
