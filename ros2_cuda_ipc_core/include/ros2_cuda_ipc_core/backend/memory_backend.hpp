// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_metadata.hpp"
#include "ros2_cuda_ipc_core/detail/interprocess_event.hpp"

namespace ros2_cuda_ipc_core::backend {

/**
 * @brief Private VMM-FD allocation state owned by a publisher block.
 *
 * The definition remains in the implementation file so Unix-socket and CUDA
 * VMM details are not exposed through this public header.  GpuBufferBlock owns
 * it with unique_ptr; consequently its destructor and move operations are
 * defined out of line, where VmmFdBlockState is complete and default_delete can
 * destroy it.
 */
struct VmmFdBlockState;

struct GpuBufferBlock {
  GpuBufferBlock();
  ~GpuBufferBlock();
  GpuBufferBlock(const GpuBufferBlock&) = delete;
  GpuBufferBlock& operator=(const GpuBufferBlock&) = delete;
  GpuBufferBlock(GpuBufferBlock&&) noexcept;
  GpuBufferBlock& operator=(GpuBufferBlock&&) noexcept;

  /// Process-unique identity used by IPC. It is not the pool vector index.
  uint32_t block_id = 0;
  /// The one shared metadata object owned by this GPU block.
  std::shared_ptr<buffer_metadata::BufferMetadata> shared_metadata;
  void* device_ptr = nullptr;
  std::unique_ptr<detail::InterprocessEvent> ready_event;
  std::string vmm_socket_path;
  std::unique_ptr<VmmFdBlockState> vmm_fd_state;
};

bool allocate_vmm_fd_memory(uint64_t byte_size, int device_index,
                            std::vector<GpuBufferBlock>& blocks);
void destroy_vmm_fd_memory(std::vector<GpuBufferBlock>& blocks) noexcept;

}  // namespace ros2_cuda_ipc_core::backend
