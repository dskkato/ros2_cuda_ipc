// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda_runtime.h>

#include <cstddef>

namespace cuda_ipc_poc::cuda_ipc {

struct IpcMsg {
  int dev;
  size_t bytes;
  cudaIpcMemHandle_t handle;
};

}  // namespace cuda_ipc_poc::cuda_ipc
