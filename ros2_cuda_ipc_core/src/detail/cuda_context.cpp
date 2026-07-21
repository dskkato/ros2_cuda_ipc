// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/detail/cuda_context.hpp"

#include <map>
#include <mutex>

namespace ros2_cuda_ipc_core::detail {
namespace {

std::once_flag init_once;
CUresult init_status = CUDA_SUCCESS;
std::mutex contexts_mutex;
std::map<int, CUcontext> contexts;

CUresult primary_context_for_device(int device_id, CUcontext* context) {
  std::call_once(init_once, [] { init_status = cuInit(0); });
  if (init_status != CUDA_SUCCESS) {
    return init_status;
  }

  std::lock_guard<std::mutex> lock(contexts_mutex);
  const auto existing = contexts.find(device_id);
  if (existing != contexts.end()) {
    *context = existing->second;
    return CUDA_SUCCESS;
  }

  CUdevice device;
  CUresult result = cuDeviceGet(&device, device_id);
  if (result != CUDA_SUCCESS) {
    return result;
  }
  result = cuDevicePrimaryCtxRetain(context, device);
  if (result == CUDA_SUCCESS) {
    contexts.emplace(device_id, *context);
  }
  return result;
}

}  // namespace

CudaContextGuard::CudaContextGuard(CudaContextGuard&& other) noexcept
    : context_(other.context_), pushed_(other.pushed_) {
  other.context_ = nullptr;
  other.pushed_ = false;
}

CudaContextGuard& CudaContextGuard::operator=(
    CudaContextGuard&& other) noexcept {
  if (this != &other) {
    reset();
    context_ = other.context_;
    pushed_ = other.pushed_;
    other.context_ = nullptr;
    other.pushed_ = false;
  }
  return *this;
}

CudaContextGuard::~CudaContextGuard() { reset(); }

CudaContextGuard CudaContextGuard::for_device(int device_id,
                                              CUresult* status) noexcept {
  CUcontext context = nullptr;
  CUresult result = primary_context_for_device(device_id, &context);
  if (result != CUDA_SUCCESS) {
    *status = result;
    return {};
  }
  return for_context(context, status);
}

CudaContextGuard CudaContextGuard::for_context(CUcontext context,
                                               CUresult* status) noexcept {
  CudaContextGuard guard(context);
  if (context == nullptr) {
    *status = CUDA_ERROR_INVALID_CONTEXT;
    return guard;
  }
  *status = cuCtxPushCurrent(context);
  guard.pushed_ = *status == CUDA_SUCCESS;
  return guard;
}

void CudaContextGuard::reset() noexcept {
  if (pushed_) {
    CUcontext popped = nullptr;
    (void)cuCtxPopCurrent(&popped);
    pushed_ = false;
  }
}

}  // namespace ros2_cuda_ipc_core::detail
