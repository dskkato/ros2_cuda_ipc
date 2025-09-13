#include "ros2_cuda_ipc_core/gpu_buffer_pool.hpp"

#include "ros2_cuda_ipc_core/cuda_support.hpp"

namespace ros2_cuda_ipc_core {

GpuBufferPool::GpuBufferPool(std::size_t size) : slots_(size) {}

GpuBufferPool::GpuBufferPool(std::size_t size, std::size_t bytes_per_slot,
                             bool use_cuda)
    : slots_(size),
      device_ptrs_(size, nullptr),
      events_(size, nullptr),
      bytes_per_slot_(bytes_per_slot) {
  if (use_cuda && bytes_per_slot_ > 0 && cuda_is_available()) {
    bool ok = true;
    // Allocate device buffers
    for (std::size_t i = 0; i < size; ++i) {
      void* p = cuda_allocate(bytes_per_slot_);
      if (!p) {
        ok = false;
        break;
      }
      device_ptrs_[i] = p;
    }
    // Create interprocess-capable events (best-effort)
    if (ok) {
      for (std::size_t i = 0; i < size; ++i) {
        void* e = cuda_event_create();
        if (!e) {
          ok = false;
          break;
        }
        events_[i] = e;
      }
    }
    if (!ok) {
      // Free any allocated resources and fall back to CPU-only mode
      for (void* e : events_) {
        if (e) cuda_event_destroy(e);
      }
      for (void* p : device_ptrs_) {
        if (p) cuda_free(p);
      }
      std::fill(events_.begin(), events_.end(), nullptr);
      std::fill(device_ptrs_.begin(), device_ptrs_.end(), nullptr);
      bytes_per_slot_ = 0;
      using_cuda_ = false;
    } else {
      using_cuda_ = true;
    }
  }
}

GpuBufferPool::~GpuBufferPool() {
  if (using_cuda_) {
    for (void* e : events_) {
      if (e) cuda_event_destroy(e);
    }
    for (void* p : device_ptrs_) {
      if (p) cuda_free(p);
    }
  }
}

std::optional<std::size_t> GpuBufferPool::borrow() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (std::size_t i = 0; i < slots_.size(); ++i) {
    if (!slots_[i].in_use) {
      slots_[i].in_use = true;
      return i;
    }
  }
  return std::nullopt;
}

bool GpuBufferPool::release(std::size_t id) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (id < slots_.size() && slots_[id].in_use) {
    slots_[id].in_use = false;
    return true;
  }
  return false;
}

void* GpuBufferPool::device_ptr(std::size_t id) const {
  if (!using_cuda_) return nullptr;
  if (id >= device_ptrs_.size()) return nullptr;
  return device_ptrs_[id];
}

bool GpuBufferPool::ipc_handle(std::size_t id,
                               CudaIpcMemHandle& out_handle) const {
  if (!using_cuda_) return false;
  if (id >= device_ptrs_.size()) return false;
  void* ptr = device_ptrs_[id];
  if (!ptr) return false;
  return cuda_ipc_get_mem_handle(ptr, &out_handle);
}

bool GpuBufferPool::record_ready(std::size_t id) {
  if (!using_cuda_) return false;
  if (id >= events_.size()) return false;
  void* evt = events_[id];
  if (!evt) return false;
  return cuda_event_record(evt);
}

bool GpuBufferPool::ipc_event_handle(std::size_t id,
                                     CudaIpcEventHandle& out_handle) const {
  if (!using_cuda_) return false;
  if (id >= events_.size()) return false;
  void* evt = events_[id];
  if (!evt) return false;
  return cuda_event_get_ipc_handle(evt, &out_handle);
}

}  // namespace ros2_cuda_ipc_core
