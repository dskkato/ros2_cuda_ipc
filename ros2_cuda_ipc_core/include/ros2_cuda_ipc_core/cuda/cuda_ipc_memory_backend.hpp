#pragma once

#include <memory>

#include "ros2_cuda_ipc_core/cuda/gpu_lease_pool.hpp"

namespace ros2_cuda_ipc_core::cuda {

std::unique_ptr<GpuLeasePool::MemoryBackend> make_cuda_ipc_memory_backend();

}  // namespace ros2_cuda_ipc_core::cuda
