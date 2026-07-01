// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include "ros2_cuda_ipc_core/cuda/memory_importer.hpp"

namespace ros2_cuda_ipc_core::cuda::vmm_fd {

class MemoryImporter final : public ros2_cuda_ipc_core::cuda::MemoryImporter {
 public:
  std::optional<ImportedMemory> import(
      const ros2_cuda_ipc_msgs::msg::BufferCore& msg,
      const cudaIpcEventHandle_t& event_handle,
      const rclcpp::Logger& logger) const override;
};

}  // namespace ros2_cuda_ipc_core::cuda::vmm_fd
