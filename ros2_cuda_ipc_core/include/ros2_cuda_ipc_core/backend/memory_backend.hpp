// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "ros2_cuda_ipc_core/detail/interprocess_event.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::backend {

struct SlotBackendState {
  virtual ~SlotBackendState() = default;
};

struct SlotResources {
  uint32_t index = 0;
  void* device_ptr = nullptr;
  std::unique_ptr<detail::InterprocessEvent> ready_event;
  transport::MemoryHandlePayload mem_handle{};
  std::shared_ptr<SlotBackendState> backend_state;
};

class MemoryBackend {
 public:
  bool allocate(uint64_t byte_size, int device_index,
                std::vector<SlotResources>& slots);
  void destroy(std::vector<SlotResources>& slots) noexcept;
};

}  // namespace ros2_cuda_ipc_core::backend
