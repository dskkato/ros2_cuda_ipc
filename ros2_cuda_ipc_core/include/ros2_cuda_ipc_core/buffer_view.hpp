#pragma once

#include <cuda_runtime_api.h>

#include <cstdint>
#include <memory>
#include <string>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
#include "ros2_cuda_ipc_core/lease_handle.hpp"

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
  BufferView(BufferView &&other) noexcept;
  BufferView &operator=(BufferView &&other) noexcept;
  BufferView(const BufferView &) = delete;
  BufferView &operator=(const BufferView &) = delete;

  template <class T = void>
  T *data() const noexcept {
    return static_cast<T *>(dev_ptr);
  }

  bool valid() const noexcept { return dev_ptr != nullptr; }

  cudaError_t wait(cudaStream_t stream) const noexcept;

  void reset() noexcept;

  void set_ipc_handles(const cudaIpcMemHandle_t &mem,
                       const cudaIpcEventHandle_t &evt) noexcept;
  const cudaIpcMemHandle_t &mem_handle() const noexcept { return mem_handle_; }
  const cudaIpcEventHandle_t &event_handle() const noexcept {
    return event_handle_;
  }
  bool handles_ready() const noexcept { return handles_ready_; }

  void mark_opened_via_ipc(bool memory_opened, bool event_opened) noexcept {
    opened_mem_via_ipc_ = memory_opened;
    opened_event_via_ipc_ = event_opened;
  }

 private:
  cudaIpcMemHandle_t mem_handle_{};
  cudaIpcEventHandle_t event_handle_{};
  bool handles_ready_ = false;
  bool opened_mem_via_ipc_ = false;
  bool opened_event_via_ipc_ = false;
};

}  // namespace ros2_cuda_ipc_core
