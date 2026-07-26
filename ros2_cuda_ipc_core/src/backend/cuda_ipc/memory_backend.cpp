// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/backend/cuda_ipc/memory_backend.hpp"

#include <rcutils/logging_macros.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
#include "ros2_cuda_ipc_core/publisher/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::backend::cuda_ipc {
namespace {

class CudaIpcMemoryBackend : public publisher::GpuBufferPool::MemoryBackend {
 public:
  bool allocate(
      uint64_t frame_size_bytes, int device_index,
      std::vector<publisher::GpuBufferPool::SlotResources>& slots) override {
    (void)device_index;
    for (auto& slot : slots) {
      CUdeviceptr device_ptr = 0;
      CUresult result = cuMemAlloc(&device_ptr, frame_size_bytes);
      if (result != CUDA_SUCCESS) {
        RCUTILS_LOG_ERROR_NAMED(
            "ros2_cuda_ipc_core.backend.cuda_ipc", "cuMemAlloc failed: %s",
            detail::CudaDriverError(result).to_string().c_str());
        destroy(slots);
        return false;
      }
      slot.device_ptr =
          reinterpret_cast<void*>(static_cast<uintptr_t>(device_ptr));

      CUipcMemHandle handle{};
      result = cuIpcGetMemHandle(&handle, device_ptr);
      if (result != CUDA_SUCCESS) {
        RCUTILS_LOG_ERROR_NAMED(
            "ros2_cuda_ipc_core.backend.cuda_ipc",
            "cuIpcGetMemHandle failed: %s",
            detail::CudaDriverError(result).to_string().c_str());
        destroy(slots);
        return false;
      }
      std::memcpy(slot.mem_handle.data(), &handle, sizeof(handle));
      slot.backend = transport::MemoryBackendKind::CUDA_IPC;
      slot.backend_state.reset();
    }
    return true;
  }

  void destroy(std::vector<publisher::GpuBufferPool::SlotResources>&
                   slots) noexcept override {
    for (auto& slot : slots) {
      if (slot.device_ptr) {
        const CUdeviceptr device_ptr = static_cast<CUdeviceptr>(
            reinterpret_cast<uintptr_t>(slot.device_ptr));
        const CUresult result = cuMemFree(device_ptr);
        if (result != CUDA_SUCCESS) {
          RCUTILS_LOG_ERROR_NAMED(
              "ros2_cuda_ipc_core.backend.cuda_ipc",
              "cuMemFree failed for slot %u: %s", slot.index,
              detail::CudaDriverError(result).to_string().c_str());
        }
        slot.device_ptr = nullptr;
      }
      slot.mem_handle.fill(0);
      slot.backend_state.reset();
      slot.backend = transport::MemoryBackendKind::CUDA_IPC;
    }
  }
};

}  // namespace

std::unique_ptr<ros2_cuda_ipc_core::backend::MemoryBackend>
make_cuda_ipc_memory_backend() {
  return std::make_unique<CudaIpcMemoryBackend>();
}

}  // namespace ros2_cuda_ipc_core::backend::cuda_ipc
