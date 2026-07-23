// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <memory>

#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::detail {

/// Owns one publisher-side interprocess event and its exported IPC handle.
class InterprocessEvent {
 public:
  static CudaResult<std::unique_ptr<InterprocessEvent>> create(
      std::shared_ptr<CudaDeviceContext> context);

  ~InterprocessEvent() noexcept;

  InterprocessEvent(const InterprocessEvent&) = delete;
  InterprocessEvent& operator=(const InterprocessEvent&) = delete;
  InterprocessEvent& operator=(InterprocessEvent&&) = delete;

  CudaResult<void> record(CUstream stream) const noexcept;

  const transport::EventHandlePayload& ipc_handle() const noexcept {
    return ipc_handle_;
  }

 private:
  InterprocessEvent(std::shared_ptr<CudaDeviceContext> context, CUevent event,
                    transport::EventHandlePayload ipc_handle) noexcept;
  InterprocessEvent(InterprocessEvent&& other) noexcept;

  void reset_noexcept() noexcept;

  std::shared_ptr<CudaDeviceContext> context_;
  CUevent event_ = nullptr;
  transport::EventHandlePayload ipc_handle_{};
};

}  // namespace ros2_cuda_ipc_core::detail
