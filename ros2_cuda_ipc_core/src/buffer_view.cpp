#include "ros2_cuda_ipc_core/buffer_view.hpp"

#include <cstring>

namespace ros2_cuda_ipc_core {

BufferView::ControlBlock::~ControlBlock() {
  if (dev_ptr && opened_mem_via_ipc) {
    CudaSupport::close_ipc_memory(dev_ptr);
  }
  if (ready_evt && opened_event_via_ipc) {
    CudaSupport::destroy_event(ready_evt);
  }
}

BufferView::~BufferView() { reset(); }

BufferView::BufferView(const BufferView &other) { *this = other; }

BufferView &BufferView::operator=(const BufferView &other) {
  if (this == &other) {
    return *this;
  }

  reset();

  dev_ptr = other.dev_ptr;
  ready_evt = other.ready_evt;
  device_id = other.device_id;
  byte_size = other.byte_size;
  slot_id = other.slot_id;
  generation = other.generation;
  shm_name = other.shm_name;
  lease = other.lease;
  mem_handle_ = other.mem_handle_;
  event_handle_ = other.event_handle_;
  handles_ready_ = other.handles_ready_;
  control_ = other.control_;

  return *this;
}

BufferView::BufferView(BufferView &&other) noexcept {
  *this = std::move(other);
}

BufferView &BufferView::operator=(BufferView &&other) noexcept {
  if (this == &other) {
    return *this;
  }

  reset();

  dev_ptr = other.dev_ptr;
  ready_evt = other.ready_evt;
  device_id = other.device_id;
  byte_size = other.byte_size;
  slot_id = other.slot_id;
  generation = other.generation;
  shm_name = std::move(other.shm_name);
  lease = std::move(other.lease);
  mem_handle_ = other.mem_handle_;
  event_handle_ = other.event_handle_;
  handles_ready_ = other.handles_ready_;
  control_ = std::move(other.control_);

  other.dev_ptr = nullptr;
  other.ready_evt = nullptr;
  other.byte_size = 0;
  other.slot_id = 0;
  other.generation = 0;
  other.shm_name.clear();
  other.handles_ready_ = false;
  other.control_.reset();

  return *this;
}

cudaError_t BufferView::wait(cudaStream_t stream) const noexcept {
  if (!ready_evt) {
    return cudaSuccess;
  }
  return CudaSupport::stream_wait_event(stream, ready_evt, 0);
}

void BufferView::reset() noexcept {
  control_.reset();
  dev_ptr = nullptr;
  ready_evt = nullptr;
  byte_size = 0;
  slot_id = 0;
  generation = 0;
  shm_name.clear();
  handles_ready_ = false;
  lease.reset();
}

void BufferView::set_ipc_handles(const cudaIpcMemHandle_t &mem,
                                 const cudaIpcEventHandle_t &evt) noexcept {
  std::memcpy(&mem_handle_, &mem, sizeof(mem_handle_));
  std::memcpy(&event_handle_, &evt, sizeof(event_handle_));
  handles_ready_ = true;
}

void BufferView::ensure_control_block() noexcept {
  if (!control_) {
    control_ = std::make_shared<ControlBlock>();
  }
  control_->dev_ptr = dev_ptr;
  control_->ready_evt = ready_evt;
}

void BufferView::mark_opened_via_ipc(bool memory_opened,
                                     bool event_opened) noexcept {
  ensure_control_block();
  control_->opened_mem_via_ipc = memory_opened;
  control_->opened_event_via_ipc = event_opened;
}

}  // namespace ros2_cuda_ipc_core
