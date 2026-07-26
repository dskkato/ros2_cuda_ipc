// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/detail/interprocess_event.hpp"

#include <rcutils/logging_macros.h>

#include <cstring>
#include <utility>

namespace ros2_cuda_ipc_core::detail {

InterprocessEvent::InterprocessEvent(
    std::shared_ptr<CudaDeviceContext> context, CUevent event,
    transport::EventHandlePayload ipc_handle) noexcept
    : context_(std::move(context)),
      event_(event),
      ipc_handle_(std::move(ipc_handle)) {}

InterprocessEvent::~InterprocessEvent() noexcept { reset_noexcept(); }

CudaResult<std::unique_ptr<InterprocessEvent>> InterprocessEvent::create(
    std::shared_ptr<CudaDeviceContext> context) {
  if (!context) {
    return CudaResult<std::unique_ptr<InterprocessEvent>>::failure(
        CudaDriverError(CUDA_ERROR_INVALID_CONTEXT));
  }
  auto guard_result = context->push_current();
  if (!guard_result) {
    return CudaResult<std::unique_ptr<InterprocessEvent>>::failure(
        guard_result.error());
  }
  auto guard = std::move(guard_result).value();

  CUevent event = nullptr;
  CUresult result =
      cuEventCreate(&event, CU_EVENT_DISABLE_TIMING | CU_EVENT_INTERPROCESS);
  if (result != CUDA_SUCCESS) {
    return CudaResult<std::unique_ptr<InterprocessEvent>>::failure(
        CudaDriverError(result));
  }

  CUipcEventHandle event_handle{};
  result = cuIpcGetEventHandle(&event_handle, event);
  if (result != CUDA_SUCCESS) {
    (void)cuEventDestroy(event);
    return CudaResult<std::unique_ptr<InterprocessEvent>>::failure(
        CudaDriverError(result));
  }

  transport::EventHandlePayload payload{};
  static_assert(sizeof(event_handle) == sizeof(payload),
                "CUDA IPC event handle payload size changed");
  std::memcpy(payload.data(), &event_handle, sizeof(event_handle));

  try {
    return CudaResult<std::unique_ptr<InterprocessEvent>>::success(
        std::unique_ptr<InterprocessEvent>(
            new InterprocessEvent(std::move(context), event, payload)));
  } catch (...) {
    // The context guard is still active here, so the event can be cleaned up
    // if allocation of the owning object fails.
    (void)cuEventDestroy(event);
    throw;
  }
}

CudaResult<void> InterprocessEvent::record(CUstream stream) const noexcept {
  if (!context_ || event_ == nullptr) {
    return CudaResult<void>::failure(
        CudaDriverError(CUDA_ERROR_INVALID_HANDLE));
  }
  auto guard_result = context_->push_current();
  if (!guard_result) {
    return CudaResult<void>::failure(guard_result.error());
  }
  auto guard = std::move(guard_result).value();
  const CUresult result = cuEventRecord(event_, stream);
  if (result != CUDA_SUCCESS) {
    return CudaResult<void>::failure(CudaDriverError(result));
  }
  return CudaResult<void>::success();
}

void InterprocessEvent::reset_noexcept() noexcept {
  if (event_ == nullptr) {
    return;
  }
  try {
    if (!context_) {
      RCUTILS_LOG_ERROR_NAMED(
          "ros2_cuda_ipc_core.interprocess_event",
          "Cannot destroy interprocess event without its CUDA context");
      event_ = nullptr;
      return;
    }
    auto guard_result = context_->push_current();
    if (!guard_result) {
      RCUTILS_LOG_ERROR_NAMED(
          "ros2_cuda_ipc_core.interprocess_event",
          "Failed to activate CUDA context for event cleanup: %s",
          guard_result.error().to_string().c_str());
      event_ = nullptr;
      return;
    }
    auto guard = std::move(guard_result).value();
    const CUresult result = cuEventDestroy(event_);
    if (result != CUDA_SUCCESS) {
      RCUTILS_LOG_ERROR_NAMED("ros2_cuda_ipc_core.interprocess_event",
                              "cuEventDestroy failed: %s",
                              CudaDriverError(result).to_string().c_str());
    }
  } catch (...) {
    // Destruction and error reporting must not escape a noexcept boundary.
  }
  event_ = nullptr;
  ipc_handle_.fill(0);
}

}  // namespace ros2_cuda_ipc_core::detail
