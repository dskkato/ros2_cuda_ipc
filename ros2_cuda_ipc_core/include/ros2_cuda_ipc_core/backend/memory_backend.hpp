// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ros2_cuda_ipc_core/detail/interprocess_event.hpp"

namespace ros2_cuda_ipc_core::backend {

/**
 * @brief Private VMM-FD allocation state owned by a publisher slot.
 *
 * The definition remains in the implementation file so Unix-socket and CUDA
 * VMM details are not exposed through this public header.  SlotResources owns
 * it with unique_ptr; consequently its destructor and move operations are
 * defined out of line, where VmmFdSlotState is complete and default_delete can
 * destroy it.
 */
struct VmmFdSlotState;

struct SlotResources {
  SlotResources();
  ~SlotResources();
  SlotResources(const SlotResources&) = delete;
  SlotResources& operator=(const SlotResources&) = delete;
  SlotResources(SlotResources&&) noexcept;
  SlotResources& operator=(SlotResources&&) noexcept;

  uint32_t index = 0;
  void* device_ptr = nullptr;
  std::unique_ptr<detail::InterprocessEvent> ready_event;
  std::string vmm_socket_path;
  std::unique_ptr<VmmFdSlotState> vmm_fd_state;
};

bool allocate_vmm_fd_memory(uint64_t byte_size, int device_index,
                            std::vector<SlotResources>& slots);
void destroy_vmm_fd_memory(std::vector<SlotResources>& slots) noexcept;

}  // namespace ros2_cuda_ipc_core::backend
