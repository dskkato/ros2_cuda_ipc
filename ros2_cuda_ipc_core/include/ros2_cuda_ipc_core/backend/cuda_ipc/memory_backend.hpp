// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <memory>

#include "ros2_cuda_ipc_core/backend/memory_backend.hpp"

namespace ros2_cuda_ipc_core::backend::cuda_ipc {

std::unique_ptr<backend::MemoryBackend> make_cuda_ipc_memory_backend();

}  // namespace ros2_cuda_ipc_core::backend::cuda_ipc
