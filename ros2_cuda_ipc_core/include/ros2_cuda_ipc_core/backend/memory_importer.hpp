// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>

#include "rclcpp/logger.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"
#include "ros2_cuda_ipc_msgs/msg/buffer_core.hpp"

namespace ros2_cuda_ipc_core::backend {

// The imported memory mapping and its synchronization event must have one
// lifetime.  In particular, BufferView keeps this whole bundle alive after
// the cache entry itself has been detached.
struct ImportedResources {
  ImportedResources() = default;
  ImportedResources(const ImportedResources&) = delete;
  ImportedResources& operator=(const ImportedResources&) = delete;
  ImportedResources& operator=(ImportedResources&&) = delete;

  ImportedResources(ImportedResources&& other) noexcept
      : dev_ptr(other.dev_ptr),
        event(other.event),
        context(std::move(other.context)),
        vmm_address(other.vmm_address),
        vmm_allocation(other.vmm_allocation),
        allocation_size(other.allocation_size) {
    other.dev_ptr = nullptr;
    other.event = nullptr;
    other.vmm_address = 0;
    other.vmm_allocation = 0;
    other.allocation_size = 0;
  }

  void* dev_ptr = nullptr;
  CUevent event = nullptr;
  std::shared_ptr<detail::CudaDeviceContext> context;
  CUdeviceptr vmm_address = 0;
  CUmemGenericAllocationHandle vmm_allocation = 0;
  std::size_t allocation_size = 0;
};

class MemoryImporter {
 public:
  virtual ~MemoryImporter() = default;

  virtual std::optional<ImportedResources> import(
      const ros2_cuda_ipc_msgs::msg::BufferCore& msg,
      const CUipcEventHandle& event_handle,
      const rclcpp::Logger& logger) const = 0;
};

bool release_imported_resources(const ImportedResources& imported) noexcept;

// Cache destruction cannot propagate a cleanup error, so its deleter uses an
// explicit best-effort adapter.  The checked function above remains available
// to callers such as cross-process tests that need the result.
inline void release_imported_resources_best_effort(
    const ImportedResources& imported) noexcept {
  (void)release_imported_resources(imported);
}

// Compatibility wrapper for the pre-bundle name.
inline bool release_imported_memory(
    const ImportedResources& imported) noexcept {
  return release_imported_resources(imported);
}

const MemoryImporter& get_memory_importer(uint8_t backend);

}  // namespace ros2_cuda_ipc_core::backend
