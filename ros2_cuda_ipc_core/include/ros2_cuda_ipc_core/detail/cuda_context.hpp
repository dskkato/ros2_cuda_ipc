// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

namespace ros2_cuda_ipc_core::detail {

/// Temporarily makes a device's shared primary context current.
///
/// Primary contexts retained by this utility intentionally live until process
/// exit.  In particular, this class never resets, destroys, or releases a
/// primary context because it may be shared with CUDA Runtime or PyTorch.
class CudaContextGuard {
 public:
  CudaContextGuard() = default;
  CudaContextGuard(const CudaContextGuard&) = delete;
  CudaContextGuard& operator=(const CudaContextGuard&) = delete;
  CudaContextGuard(CudaContextGuard&& other) noexcept;
  CudaContextGuard& operator=(CudaContextGuard&& other) noexcept;
  ~CudaContextGuard();

  /// Retains (once per device) and pushes the device primary context.
  static CudaContextGuard for_device(int device_id, CUresult* status) noexcept;

  /// Pushes an already retained context.  Used for resource cleanup.
  static CudaContextGuard for_context(CUcontext context,
                                      CUresult* status) noexcept;

  explicit operator bool() const noexcept { return pushed_; }
  CUcontext context() const noexcept { return context_; }

 private:
  explicit CudaContextGuard(CUcontext context) noexcept : context_(context) {}
  void reset() noexcept;

  CUcontext context_ = nullptr;
  bool pushed_ = false;
};

}  // namespace ros2_cuda_ipc_core::detail
