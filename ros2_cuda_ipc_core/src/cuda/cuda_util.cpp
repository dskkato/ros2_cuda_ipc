#include "ros2_cuda_ipc_core/cuda/cuda_util.hpp"

namespace ros2_cuda_ipc_core::cuda {

std::string cuda_error_to_string(cudaError_t err) {
  return std::string(cudaGetErrorName(err)) + ": " + cudaGetErrorString(err);
}

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

}  // namespace ros2_cuda_ipc_core::cuda
