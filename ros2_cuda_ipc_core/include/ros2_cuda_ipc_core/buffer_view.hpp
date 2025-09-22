#pragma once

#include <cuda_runtime_api.h>

#include <cstdint>
#include <memory>
#include <string>

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
  BufferView(const BufferView &other);
  BufferView &operator=(const BufferView &other);
  BufferView(BufferView &&other) noexcept;
  BufferView &operator=(BufferView &&other) noexcept;

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

  void mark_opened_via_ipc(bool memory_opened, bool event_opened) noexcept;

 private:
  struct ControlBlock {
    void *dev_ptr = nullptr;
    cudaEvent_t ready_evt = nullptr;
    bool opened_mem_via_ipc = false;
    bool opened_event_via_ipc = false;

    ControlBlock() = default;
    ControlBlock(void *ptr, cudaEvent_t evt, bool mem_opened, bool evt_opened)
        : dev_ptr(ptr),
          ready_evt(evt),
          opened_mem_via_ipc(mem_opened),
          opened_event_via_ipc(evt_opened) {}
    ~ControlBlock();
  };

  void ensure_control_block() noexcept;

  cudaIpcMemHandle_t mem_handle_{};
  cudaIpcEventHandle_t event_handle_{};
  bool handles_ready_ = false;
  std::shared_ptr<ControlBlock> control_;
};

}  // namespace ros2_cuda_ipc_core
