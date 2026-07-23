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

  dev_ptr = other.dev_ptr;
  ready_evt = other.ready_evt;
  context = other.context;
  device_id = other.device_id;
  byte_size = other.byte_size;
  slot_id = other.slot_id;
  generation = other.generation;
  shm_name = other.shm_name;
  publisher_instance_id = other.publisher_instance_id;
  lease = other.lease;
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

  dev_ptr = other.dev_ptr;
  ready_evt = other.ready_evt;
  context = std::move(other.context);
  device_id = other.device_id;
  byte_size = other.byte_size;
  slot_id = other.slot_id;
  generation = other.generation;
  shm_name = std::move(other.shm_name);
  publisher_instance_id = other.publisher_instance_id;
  lease = std::move(other.lease);
  mem_payload_ = other.mem_payload_;
  event_handle_ = other.event_handle_;
  backend_ = other.backend_;
  handles_ready_ = other.handles_ready_;
  other.dev_ptr = nullptr;
  other.ready_evt = nullptr;
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
  if (!ready_evt) {
    return detail::CudaResult<void>::success();
  }
  if (context) {
    auto guard_result = context->push_current();
    if (!guard_result) {
      return detail::CudaResult<void>::failure(guard_result.error());
    }
    auto guard = std::move(guard_result).value();
    const CUresult result = cuStreamWaitEvent(stream, ready_evt, 0);
    if (result != CUDA_SUCCESS) {
      return detail::CudaResult<void>::failure(detail::CudaDriverError(result));
    }
    return detail::CudaResult<void>::success();
  }
  const CUresult result = cuStreamWaitEvent(stream, ready_evt, 0);
  if (result != CUDA_SUCCESS) {
    return detail::CudaResult<void>::failure(detail::CudaDriverError(result));
  }
  return detail::CudaResult<void>::success();
}

void BufferView::reset() noexcept {
  dev_ptr = nullptr;
  ready_evt = nullptr;
  context.reset();
  byte_size = 0;
  slot_id = 0;
  generation = 0;
  shm_name.clear();
  publisher_instance_id = {};
  handles_ready_ = false;
  backend_ = transport::MemoryBackendKind::CUDA_IPC;
  lease.reset();
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
