// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"

namespace ros2_cuda_ipc_core::backend::vmm_fd {

class MemoryImporter final : public backend::MemoryImporter {
 public:
  std::optional<ImportedResources> import(
      const ros2_cuda_ipc_msgs::msg::BufferCore& msg,
      const CUipcEventHandle& event_handle) const override;
};

}  // namespace ros2_cuda_ipc_core::backend::vmm_fd
