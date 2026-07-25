// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <cstdint>
#include <memory>
#include <string>

#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
#include "ros2_cuda_ipc_core/lease/lease_handle.hpp"
#include "ros2_cuda_ipc_core/publisher_instance_id.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::subscriber {

struct BufferView {
  int device_id = 0;
  uint64_t byte_size = 0;
  uint32_t slot_id = 0;
  uint32_t generation = 0;
  std::string shm_name;
  PublisherInstanceId publisher_instance_id{};
  std::shared_ptr<lease::LeaseHandle> lease;

  BufferView() = default;
  ~BufferView();
  BufferView(const BufferView& other);
  BufferView& operator=(const BufferView& other);
  BufferView(BufferView&& other) noexcept;
  BufferView& operator=(BufferView&& other) noexcept;

  template <class T = void>
  T* data() const noexcept {
    return static_cast<T*>(device_ptr());
  }

  void* device_ptr() const noexcept {
    return imported_resource_ ? imported_resource_->dev_ptr : nullptr;
  }

  CUevent ready_event() const noexcept {
    return imported_resource_ ? imported_resource_->event : nullptr;
  }

  bool valid() const noexcept { return device_ptr() != nullptr; }

  detail::CudaResult<void> enqueue_ready_event(CUstream stream) const noexcept;

  // Validate that a consumer CUDA stream refers to the same device as the
  // imported allocation.  The test-support fixture may not attach a CUDA
  // context to its synthetic allocation; that case is intentionally treated
  // as unverifiable and left to the caller.
  detail::CudaResult<void> validate_stream(CUstream stream) const noexcept;

  int imported_device_id() const noexcept {
    return imported_resource_ && imported_resource_->context
               ? imported_resource_->context->device_id()
               : -1;
  }

  void reset() noexcept;

  void set_ipc_handles(transport::MemoryBackendKind backend,
                       const uint8_t* payload_bytes, std::size_t payload_size,
                       const transport::EventHandlePayload& evt) noexcept;
  void set_imported_resource(
      std::shared_ptr<const backend::ImportedResources> resource) noexcept;
  const transport::MemoryHandlePayload& mem_payload() const noexcept {
    return mem_payload_;
  }
  const transport::EventHandlePayload& event_handle() const noexcept {
    return event_handle_;
  }
  transport::MemoryBackendKind backend() const noexcept { return backend_; }
  bool handles_ready() const noexcept { return handles_ready_; }

 private:
  std::shared_ptr<const backend::ImportedResources> imported_resource_;
  transport::MemoryHandlePayload mem_payload_{};
  transport::EventHandlePayload event_handle_{};
  transport::MemoryBackendKind backend_ =
      transport::MemoryBackendKind::CUDA_IPC;
  bool handles_ready_ = false;
};

}  // namespace ros2_cuda_ipc_core::subscriber
