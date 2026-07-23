// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/subscriber/buffer_view.hpp"

#include <algorithm>
#include <cstring>

namespace ros2_cuda_ipc_core::subscriber {

BufferView::~BufferView() { reset(); }

BufferView::BufferView(const BufferView& other) { *this = other; }

BufferView& BufferView::operator=(const BufferView& other) {
  if (this == &other) {
    return *this;
  }

  reset();

  device_id = other.device_id;
  byte_size = other.byte_size;
  slot_id = other.slot_id;
  generation = other.generation;
  shm_name = other.shm_name;
  publisher_instance_id = other.publisher_instance_id;
  lease = other.lease;
  imported_resource_ = other.imported_resource_;
  mem_payload_ = other.mem_payload_;
  event_handle_ = other.event_handle_;
  backend_ = other.backend_;
  handles_ready_ = other.handles_ready_;

  return *this;
}

BufferView::BufferView(BufferView&& other) noexcept {
  *this = std::move(other);
}

BufferView& BufferView::operator=(BufferView&& other) noexcept {
  if (this == &other) {
    return *this;
  }

  reset();

  device_id = other.device_id;
  byte_size = other.byte_size;
  slot_id = other.slot_id;
  generation = other.generation;
  shm_name = std::move(other.shm_name);
  publisher_instance_id = other.publisher_instance_id;
  lease = std::move(other.lease);
  imported_resource_ = std::move(other.imported_resource_);
  mem_payload_ = other.mem_payload_;
  event_handle_ = other.event_handle_;
  backend_ = other.backend_;
  handles_ready_ = other.handles_ready_;
  other.byte_size = 0;
  other.slot_id = 0;
  other.generation = 0;
  other.shm_name.clear();
  other.publisher_instance_id = {};
  other.handles_ready_ = false;

  return *this;
}

detail::CudaResult<void> BufferView::enqueue_ready_event(
    cudaStream_t stream) const noexcept {
  const CUevent event = ready_event();
  if (!event) {
    return detail::CudaResult<void>::success();
  }
  const auto resource_context =
      imported_resource_ ? imported_resource_->context : nullptr;
  if (resource_context) {
    auto guard_result = resource_context->push_current();
    if (!guard_result) {
      return detail::CudaResult<void>::failure(guard_result.error());
    }
    auto guard = std::move(guard_result).value();
    const CUresult result = cuStreamWaitEvent(stream, event, 0);
    if (result != CUDA_SUCCESS) {
      return detail::CudaResult<void>::failure(detail::CudaDriverError(result));
    }
    return detail::CudaResult<void>::success();
  }
  const CUresult result = cuStreamWaitEvent(stream, event, 0);
  if (result != CUDA_SUCCESS) {
    return detail::CudaResult<void>::failure(detail::CudaDriverError(result));
  }
  return detail::CudaResult<void>::success();
}

void BufferView::reset() noexcept {
  imported_resource_.reset();
  mem_payload_.fill(0);
  event_handle_.fill(0);
  byte_size = 0;
  slot_id = 0;
  generation = 0;
  shm_name.clear();
  publisher_instance_id = {};
  handles_ready_ = false;
  backend_ = transport::MemoryBackendKind::CUDA_IPC;
  lease.reset();
}

void BufferView::set_imported_resource(
    std::shared_ptr<const backend::ImportedResources> resource) noexcept {
  imported_resource_ = std::move(resource);
}

void BufferView::set_ipc_handles(
    transport::MemoryBackendKind backend, const uint8_t* payload_bytes,
    std::size_t payload_size,
    const transport::EventHandlePayload& evt) noexcept {
  backend_ = backend;
  std::memset(mem_payload_.data(), 0, mem_payload_.size());
  if (payload_bytes != nullptr && payload_size > 0) {
    const auto copy =
        std::min(payload_size, static_cast<std::size_t>(mem_payload_.size()));
    std::memcpy(mem_payload_.data(), payload_bytes, copy);
  }
  event_handle_ = evt;
  handles_ready_ = true;
}

}  // namespace ros2_cuda_ipc_core::subscriber
