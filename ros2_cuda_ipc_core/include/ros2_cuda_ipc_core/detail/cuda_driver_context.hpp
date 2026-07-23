// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

namespace ros2_cuda_ipc_core::detail {

// Initializes the Driver API exactly once.  This deliberately does not reset
// the primary context; applications such as PyTorch may own it as well.
CUresult ensure_cuda_driver_initialized() noexcept;

// Makes the device primary context current for the duration of a Driver API
// operation and restores the previous context on destruction.
class ScopedPrimaryContext {
 public:
  explicit ScopedPrimaryContext(int device_id) noexcept;
  ~ScopedPrimaryContext() noexcept;

  ScopedPrimaryContext(const ScopedPrimaryContext&) = delete;
  ScopedPrimaryContext& operator=(const ScopedPrimaryContext&) = delete;

  bool ok() const noexcept { return status_ == CUDA_SUCCESS; }
  CUresult status() const noexcept { return status_; }

 private:
  CUcontext previous_ = nullptr;
  CUcontext primary_ = nullptr;
  CUdevice device_ = 0;
  bool pushed_ = false;
  CUresult status_ = CUDA_ERROR_NOT_INITIALIZED;
};

}  // namespace ros2_cuda_ipc_core::detail
