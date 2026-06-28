// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <memory>

#include "ros2_cuda_ipc_core/cuda/gpu_lease_pool.hpp"

namespace ros2_cuda_ipc_core::cuda {

std::unique_ptr<GpuLeasePool::MemoryBackend> make_vmm_fd_memory_backend();

}  // namespace ros2_cuda_ipc_core::cuda
