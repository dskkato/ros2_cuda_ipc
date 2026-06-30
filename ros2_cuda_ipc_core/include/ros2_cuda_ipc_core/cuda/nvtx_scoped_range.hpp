// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <nvtx3/nvToolsExt.h>

namespace ros2_cuda_ipc_core::cuda {

class NvtxScopedRange {
 public:
  explicit NvtxScopedRange(const char* name) : id_(nvtxRangeStartA(name)) {}

  NvtxScopedRange(const NvtxScopedRange&) = delete;
  NvtxScopedRange& operator=(const NvtxScopedRange&) = delete;
  NvtxScopedRange(NvtxScopedRange&&) = delete;
  NvtxScopedRange& operator=(NvtxScopedRange&&) = delete;

  ~NvtxScopedRange() { nvtxRangeEnd(id_); }

 private:
  nvtxRangeId_t id_{};
};

}  // namespace ros2_cuda_ipc_core::cuda
