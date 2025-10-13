#include "ros2_cuda_ipc_core/cuda/cuda_util.hpp"

namespace ros2_cuda_ipc_core::cuda {

std::string cuda_error_to_string(cudaError_t err) {
  return std::string(cudaGetErrorName(err)) + ": " + cudaGetErrorString(err);
}

}  // namespace ros2_cuda_ipc_core::cuda
