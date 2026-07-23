// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"

#include <mutex>

namespace ros2_cuda_ipc_core::detail {

CUresult ensure_cuda_driver_initialized() noexcept {
  static std::once_flag once;
  static CUresult result = CUDA_ERROR_NOT_INITIALIZED;
  std::call_once(once, []() { result = cuInit(0); });
  return result;
}

ScopedPrimaryContext::ScopedPrimaryContext(int device_id) noexcept {
  status_ = ensure_cuda_driver_initialized();
  if (status_ != CUDA_SUCCESS) {
    return;
  }

  status_ = cuDeviceGet(&device_, device_id);
  if (status_ != CUDA_SUCCESS) {
    return;
  }

  status_ = cuCtxGetCurrent(&previous_);
  if (status_ != CUDA_SUCCESS) {
    return;
  }

  status_ = cuDevicePrimaryCtxRetain(&primary_, device_);
  if (status_ != CUDA_SUCCESS) {
    return;
  }
  retained_ = true;

  status_ = cuCtxPushCurrent(primary_);
  if (status_ != CUDA_SUCCESS) {
    cuDevicePrimaryCtxRelease(device_);
    retained_ = false;
    return;
  }
  pushed_ = true;
}

ScopedPrimaryContext::~ScopedPrimaryContext() noexcept {
  if (pushed_) {
    CUcontext ignored = nullptr;
    (void)cuCtxPopCurrent(&ignored);
  }
  if (retained_) {
    (void)cuDevicePrimaryCtxRelease(device_);
  }
}

}  // namespace ros2_cuda_ipc_core::detail
