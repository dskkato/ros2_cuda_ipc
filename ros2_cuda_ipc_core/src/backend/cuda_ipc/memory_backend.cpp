// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/backend/cuda_ipc/memory_backend.hpp"

#include <cuda_runtime_api.h>

#include <cstring>
#include <memory>
#include <vector>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_util.hpp"
#include "ros2_cuda_ipc_core/publisher/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::backend::cuda_ipc {
namespace {

class CudaIpcMemoryBackend : public publisher::GpuBufferPool::MemoryBackend {
 public:
  bool allocate(uint64_t frame_size_bytes, int device_index,
                std::vector<publisher::GpuBufferPool::SlotResources>& slots,
                rclcpp::Logger logger) override {
    (void)device_index;
    for (auto& slot : slots) {
      cudaError_t err = cudaMalloc(&slot.device_ptr, frame_size_bytes);
      if (err != cudaSuccess) {
        RCLCPP_ERROR(logger, "cudaMalloc failed: %s",
                     detail::cuda_error_to_string(err).c_str());
        destroy(slots, logger);
        return false;
      }

      cudaIpcMemHandle_t handle{};
      err = cudaIpcGetMemHandle(&handle, slot.device_ptr);
      if (err != cudaSuccess) {
        RCLCPP_ERROR(logger, "cudaIpcGetMemHandle failed: %s",
                     detail::cuda_error_to_string(err).c_str());
        destroy(slots, logger);
        return false;
      }
      std::memcpy(slot.mem_handle.data(), &handle, sizeof(handle));
      slot.backend = transport::MemoryBackendKind::CUDA_IPC;
      slot.backend_state.reset();
    }
    return true;
  }

  void destroy(std::vector<publisher::GpuBufferPool::SlotResources>& slots,
               rclcpp::Logger logger) noexcept override {
    for (auto& slot : slots) {
      if (slot.device_ptr) {
        const cudaError_t free_err = cudaFree(slot.device_ptr);
        if (free_err != cudaSuccess) {
          RCLCPP_ERROR(logger, "cudaFree failed for slot %u: %s", slot.index,
                       detail::cuda_error_to_string(free_err).c_str());
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
