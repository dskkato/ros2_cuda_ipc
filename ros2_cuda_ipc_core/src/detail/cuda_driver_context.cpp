// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"

#include <array>
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

  // Keep one retain for each device for the entire process.  Releasing the
  // primary context after every operation can invalidate Driver objects when
  // no Runtime client happens to hold its own retain (and would race with
  // clients such as PyTorch).  We only push/pop per operation; reset is never
  // performed here.
  static std::mutex primary_mutex;
  static std::array<CUcontext, 256> primary_contexts{};
  if (device_id < 0 ||
      static_cast<std::size_t>(device_id) >= primary_contexts.size()) {
    status_ = CUDA_ERROR_INVALID_DEVICE;
    return;
  }
  {
    std::lock_guard<std::mutex> lock(primary_mutex);
    auto& existing = primary_contexts[static_cast<std::size_t>(device_id)];
    if (existing != nullptr) {
      primary_ = existing;
      status_ = CUDA_SUCCESS;
    } else {
      status_ = cuDevicePrimaryCtxRetain(&primary_, device_);
      if (status_ == CUDA_SUCCESS) {
        existing = primary_;
      }
    }
  }
  if (status_ != CUDA_SUCCESS) {
    return;
  }

  status_ = cuCtxPushCurrent(primary_);
  if (status_ != CUDA_SUCCESS) {
    return;
  }
  pushed_ = true;
}

ScopedPrimaryContext::~ScopedPrimaryContext() noexcept {
  if (pushed_) {
    CUcontext ignored = nullptr;
    (void)cuCtxPopCurrent(&ignored);
  }
}

}  // namespace ros2_cuda_ipc_core::detail
