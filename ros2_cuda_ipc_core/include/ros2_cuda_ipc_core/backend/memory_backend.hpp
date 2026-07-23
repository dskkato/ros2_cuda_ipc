// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>
#include <cuda_runtime_api.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "rclcpp/logger.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::backend {

struct SlotBackendState {
  virtual ~SlotBackendState() = default;
};

struct SlotResources {
  uint32_t index = 0;
  void* device_ptr = nullptr;
  CUevent event = nullptr;
  transport::EventHandlePayload event_handle{};
  transport::MemoryBackendKind backend = transport::MemoryBackendKind::CUDA_IPC;
  transport::MemoryHandlePayload mem_handle{};
  std::shared_ptr<SlotBackendState> backend_state;
};

class MemoryBackend {
 public:
  virtual ~MemoryBackend() = default;
  virtual bool allocate(uint64_t byte_size, int device_index,
                        std::vector<SlotResources>& slots,
                        rclcpp::Logger logger) = 0;
  virtual void destroy(std::vector<SlotResources>& slots,
                       rclcpp::Logger logger) noexcept = 0;
};

}  // namespace ros2_cuda_ipc_core::backend
