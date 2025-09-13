// Fallback (non-CUDA) implementations
#include "ros2_cuda_ipc_core/cuda_support.hpp"

namespace ros2_cuda_ipc_core {

bool cuda_is_available() { return false; }

void* cuda_allocate(std::size_t /*bytes*/) { return nullptr; }

bool cuda_free(void* /*ptr*/) { return false; }

}  // namespace ros2_cuda_ipc_core
