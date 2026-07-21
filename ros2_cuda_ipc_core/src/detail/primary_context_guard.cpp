// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/detail/primary_context_guard.hpp"

namespace ros2_cuda_ipc_core::detail {

PrimaryContextGuard::PrimaryContextGuard(int device_id) noexcept {
  result_ = cuInit(0);
  if (result_ != CUDA_SUCCESS) return;
  result_ = cuCtxGetCurrent(&previous_);
  if (result_ != CUDA_SUCCESS) return;
  result_ = cuDeviceGet(&device_, device_id);
  if (result_ != CUDA_SUCCESS) return;
  CUcontext primary = nullptr;
  result_ = cuDevicePrimaryCtxRetain(&primary, device_);
  if (result_ != CUDA_SUCCESS) return;
  retained_ = true;
  result_ = cuCtxSetCurrent(primary);
}

PrimaryContextGuard::~PrimaryContextGuard() {
  if (!retained_) return;
  cuCtxSetCurrent(previous_);
  cuDevicePrimaryCtxRelease(device_);
}

}  // namespace ros2_cuda_ipc_core::detail
