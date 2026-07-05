// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>

namespace cuda_ipc_poc::vmm_fd {

struct IpcMsg {
  int dev;
  size_t logical_bytes;
  size_t alloc_bytes;
};

}  // namespace cuda_ipc_poc::vmm_fd
