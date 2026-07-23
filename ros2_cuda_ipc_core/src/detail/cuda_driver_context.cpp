// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"

#include <mutex>
#include <string>

#include "rclcpp/logging.hpp"

namespace ros2_cuda_ipc_core::detail {
namespace {

rclcpp::Logger cuda_context_logger() {
  return rclcpp::get_logger("ros2_cuda_ipc_core.cuda_context");
}

}  // namespace

std::string CudaDriverError::name() const {
  const char* value = nullptr;
  if (cuGetErrorName(result_, &value) != CUDA_SUCCESS || value == nullptr) {
    return "CUDA_ERROR_UNKNOWN";
  }
  return value;
}

std::string CudaDriverError::description() const {
  const char* value = nullptr;
  if (cuGetErrorString(result_, &value) != CUDA_SUCCESS || value == nullptr) {
    return "unknown CUDA driver error";
  }
  return value;
}

std::string CudaDriverError::to_string() const {
  return name() + ": " + description();
}

CudaResult<void> CudaDriver::initialize() {
  static std::once_flag once;
  static CUresult status = CUDA_SUCCESS;
  std::call_once(once, []() { status = cuInit(0); });
  if (status != CUDA_SUCCESS) {
    return CudaResult<void>::failure(CudaDriverError(status));
  }
  return CudaResult<void>::success();
}

CudaResult<CudaContextGuard> CudaContextGuard::push(CUcontext context) {
  auto init_result = CudaDriver::initialize();
  if (!init_result) {
    return CudaResult<CudaContextGuard>::failure(init_result.error());
  }
  if (context == nullptr) {
    return CudaResult<CudaContextGuard>::failure(
        CudaDriverError(CUDA_ERROR_INVALID_CONTEXT));
  }

  const CUresult result = cuCtxPushCurrent(context);
  if (result != CUDA_SUCCESS) {
    return CudaResult<CudaContextGuard>::failure(CudaDriverError(result));
  }
  return CudaResult<CudaContextGuard>::success(CudaContextGuard(context));
}

CudaContextGuard::~CudaContextGuard() noexcept { pop_noexcept(); }

CudaContextGuard::CudaContextGuard(CudaContextGuard&& other) noexcept
    : context_(other.context_),
      owner_thread_(other.owner_thread_),
      active_(other.active_) {
  other.context_ = nullptr;
  other.active_ = false;
}

CudaContextGuard& CudaContextGuard::operator=(
    CudaContextGuard&& other) noexcept {
  if (this != &other) {
    pop_noexcept();
    context_ = other.context_;
    owner_thread_ = other.owner_thread_;
    active_ = other.active_;
    other.context_ = nullptr;
    other.active_ = false;
  }
  return *this;
}

void CudaContextGuard::pop_noexcept() noexcept {
  if (!active_) {
    return;
  }

  try {
    const auto logger = cuda_context_logger();
    if (owner_thread_ != std::this_thread::get_id()) {
      RCLCPP_ERROR(
          logger,
          "CUDA context guard destroyed on a different thread; refusing to "
          "pop a thread-local context stack");
      active_ = false;
      context_ = nullptr;
      return;
    }

    const CUcontext expected = context_;
    CUcontext popped = nullptr;
    const CUresult result = cuCtxPopCurrent(&popped);
    active_ = false;
    context_ = nullptr;
    if (result != CUDA_SUCCESS) {
      RCLCPP_ERROR(logger, "cuCtxPopCurrent failed: %s",
                   CudaDriverError(result).to_string().c_str());
      return;
    }
    if (popped != expected) {
      RCLCPP_ERROR(logger,
                   "cuCtxPopCurrent returned an unexpected context "
                   "(expected=%p, popped=%p)",
                   static_cast<void*>(expected), static_cast<void*>(popped));
    }
  } catch (...) {
    active_ = false;
    context_ = nullptr;
    try {
      RCLCPP_ERROR(cuda_context_logger(),
                   "CUDA context guard cleanup failed while reporting an "
                   "error");
    } catch (...) {
      // Destructors must not throw, including while reporting a cleanup error.
    }
  }
}

CudaDeviceContext::~CudaDeviceContext() noexcept {
  const CUresult result = cuDevicePrimaryCtxRelease(device_);
  if (result != CUDA_SUCCESS) {
    try {
      RCLCPP_ERROR(cuda_context_logger(),
                   "cuDevicePrimaryCtxRelease(device=%d) failed: %s",
                   device_id_, CudaDriverError(result).to_string().c_str());
    } catch (...) {
      try {
        RCLCPP_ERROR(cuda_context_logger(),
                     "cuDevicePrimaryCtxRelease(device=%d) failed with an "
                     "unformattable CUDA Driver API error",
                     device_id_);
      } catch (...) {
        // A noexcept destructor must not throw while reporting cleanup.
      }
    }
  }
}

CudaResult<std::shared_ptr<CudaDeviceContext>>
CudaDeviceContext::retain_primary(int device_id) {
  auto init_result = CudaDriver::initialize();
  if (!init_result) {
    return CudaResult<std::shared_ptr<CudaDeviceContext>>::failure(
        init_result.error());
  }

  int device_count = 0;
  CUresult result = cuDeviceGetCount(&device_count);
  if (result != CUDA_SUCCESS) {
    return CudaResult<std::shared_ptr<CudaDeviceContext>>::failure(
        CudaDriverError(result));
  }
  if (device_id < 0 || device_id >= device_count) {
    return CudaResult<std::shared_ptr<CudaDeviceContext>>::failure(
        CudaDriverError(CUDA_ERROR_INVALID_DEVICE));
  }

  CUdevice device = 0;
  result = cuDeviceGet(&device, device_id);
  if (result != CUDA_SUCCESS) {
    return CudaResult<std::shared_ptr<CudaDeviceContext>>::failure(
        CudaDriverError(result));
  }

  CUcontext context = nullptr;
  result = cuDevicePrimaryCtxRetain(&context, device);
  if (result != CUDA_SUCCESS) {
    return CudaResult<std::shared_ptr<CudaDeviceContext>>::failure(
        CudaDriverError(result));
  }

  try {
    return CudaResult<std::shared_ptr<CudaDeviceContext>>::success(
        std::shared_ptr<CudaDeviceContext>(
            new CudaDeviceContext(device_id, device, context)));
  } catch (...) {
    // The retain succeeded, so balance it if allocating the wrapper fails.
    (void)cuDevicePrimaryCtxRelease(device);
    throw;
  }
}

CudaResult<CudaContextGuard> CudaDeviceContext::push_current() const {
  return CudaContextGuard::push(context_);
}

}  // namespace ros2_cuda_ipc_core::detail
