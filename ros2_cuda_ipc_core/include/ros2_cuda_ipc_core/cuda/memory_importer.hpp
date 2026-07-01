// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>
#include <cuda_runtime_api.h>

#include <cstddef>
#include <optional>

#include "rclcpp/logger.hpp"
#include "ros2_cuda_ipc_msgs/msg/buffer_core.hpp"

namespace ros2_cuda_ipc_core::cuda {

struct ImportedMemory {
  void* dev_ptr = nullptr;
  cudaEvent_t event = nullptr;
  CUdeviceptr vmm_address = 0;
  CUmemGenericAllocationHandle vmm_allocation = 0;
  std::size_t allocation_size = 0;
};

class MemoryImporter {
 public:
  virtual ~MemoryImporter() = default;

  virtual std::optional<ImportedMemory> import(
      const ros2_cuda_ipc_msgs::msg::BufferCore& msg,
      const cudaIpcEventHandle_t& event_handle,
      const rclcpp::Logger& logger) const = 0;
};

void release_imported_memory(const ImportedMemory& imported) noexcept;

const MemoryImporter& get_memory_importer(uint8_t backend);

}  // namespace ros2_cuda_ipc_core::cuda
