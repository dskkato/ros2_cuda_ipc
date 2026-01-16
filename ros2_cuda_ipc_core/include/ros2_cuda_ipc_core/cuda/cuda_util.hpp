#pragma once

#include <cuda.h>
#include <cuda_runtime_api.h>

#include <string>

namespace ros2_cuda_ipc_core::cuda {

/// Convert a CUDA error code into a descriptive "<name>: <message>" string.
std::string cuda_error_to_string(cudaError_t err);

/// Convert a CUDA driver API error into "<name>: <message>".
std::string cu_result_to_string(CUresult result);

}  // namespace ros2_cuda_ipc_core::cuda
