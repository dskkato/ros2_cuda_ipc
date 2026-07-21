// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <cstdint>
#include <memory>
#include <string>

#include "ros2_cuda_ipc_core/lease/lease_handle.hpp"
#include "ros2_cuda_ipc_core/publisher_instance_id.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"

namespace ros2_cuda_ipc_core::subscriber {

struct BufferView {
  void* dev_ptr = nullptr;
  CUevent ready_evt = nullptr;
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
    return static_cast<T*>(dev_ptr);
  }

  bool valid() const noexcept { return dev_ptr != nullptr; }

  CUresult enqueue_ready_event(CUstream stream) const noexcept;

  void reset() noexcept;

  void set_ipc_handles(transport::MemoryBackendKind backend,
                       const uint8_t* payload_bytes, std::size_t payload_size,
                       const CUipcEventHandle& evt) noexcept;
  const transport::MemoryHandlePayload& mem_payload() const noexcept {
    return mem_payload_;
  }
  const CUipcEventHandle& event_handle() const noexcept {
    return event_handle_;
  }
  transport::MemoryBackendKind backend() const noexcept { return backend_; }
  bool handles_ready() const noexcept { return handles_ready_; }

 private:
  transport::MemoryHandlePayload mem_payload_{};
  CUipcEventHandle event_handle_{};
  transport::MemoryBackendKind backend_ =
      transport::MemoryBackendKind::CUDA_IPC;
  bool handles_ready_ = false;
};

}  // namespace ros2_cuda_ipc_core::subscriber
