// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda_runtime.h>

#include <cstddef>

namespace cuda_ipc_poc::vmm_fd {

struct IpcMsg {
  int dev;
  size_t logical_bytes;
  size_t alloc_bytes;
  cudaIpcEventHandle_t event_handle;
};

}  // namespace cuda_ipc_poc::vmm_fd
