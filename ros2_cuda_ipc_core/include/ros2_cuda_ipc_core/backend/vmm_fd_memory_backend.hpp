// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <memory>

#include "ros2_cuda_ipc_core/backend/memory_backend.hpp"

namespace ros2_cuda_ipc_core::backend {

std::unique_ptr<MemoryBackend> make_vmm_fd_memory_backend();

}  // namespace ros2_cuda_ipc_core::backend
