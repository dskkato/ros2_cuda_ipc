// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

namespace ros2_cuda_ipc_core::detail {

// Makes a device's primary context current and restores the caller's context
// when it goes out of scope.  This is deliberately a Driver API utility: a
// subscriber may be used in a process which already owns the primary context
// through cudart (for example PyTorch).
class PrimaryContextGuard {
 public:
  explicit PrimaryContextGuard(int device_id) noexcept;
  ~PrimaryContextGuard();
  PrimaryContextGuard(const PrimaryContextGuard&) = delete;
  PrimaryContextGuard& operator=(const PrimaryContextGuard&) = delete;

  bool ok() const noexcept { return result_ == CUDA_SUCCESS; }
  CUresult result() const noexcept { return result_; }

 private:
  CUcontext previous_ = nullptr;
  CUdevice device_ = 0;
  bool retained_ = false;
  CUresult result_ = CUDA_ERROR_NOT_INITIALIZED;
};

}  // namespace ros2_cuda_ipc_core::detail
