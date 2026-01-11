#pragma once

#include <cuda_runtime_api.h>

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "ros2_cuda_ipc_core/lease_handle.hpp"
#include "ros2_cuda_ipc_core/memory_types.hpp"

namespace ros2_cuda_ipc_core {

struct BufferView {
  void *dev_ptr = nullptr;
  cudaEvent_t ready_evt = nullptr;
  int device_id = 0;
  uint64_t byte_size = 0;
  uint32_t slot_id = 0;
  uint32_t generation = 0;
  std::string shm_name;
  std::shared_ptr<LeaseHandle> lease;

  BufferView() = default;
  ~BufferView();
  BufferView(const BufferView &other);
  BufferView &operator=(const BufferView &other);
  BufferView(BufferView &&other) noexcept;
  BufferView &operator=(BufferView &&other) noexcept;

  template <class T = void>
  T *data() const noexcept {
    return static_cast<T *>(dev_ptr);
  }

  bool valid() const noexcept { return dev_ptr != nullptr; }

  cudaError_t enqueue_ready_event(cudaStream_t stream) const noexcept;

  void reset() noexcept;

  void set_ipc_handles(const cudaIpcMemHandle_t &mem,
                       const cudaIpcEventHandle_t &evt) noexcept;
  void set_memory_handles(MemoryBackendKind backend,
                          const uint8_t *payload_bytes,
                          std::size_t payload_size,
                          const cudaIpcEventHandle_t &evt) noexcept;
  const MemoryHandlePayload &mem_payload() const noexcept {
    return mem_payload_;
  }
  const cudaIpcEventHandle_t &event_handle() const noexcept {
    return event_handle_;
  }
  MemoryBackendKind backend() const noexcept { return backend_; }
  bool handles_ready() const noexcept { return handles_ready_; }

 private:
  MemoryHandlePayload mem_payload_{};
  cudaIpcEventHandle_t event_handle_{};
  MemoryBackendKind backend_ = MemoryBackendKind::CUDA_IPC;
  bool handles_ready_ = false;
};

}  // namespace ros2_cuda_ipc_core
