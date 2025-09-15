#include "ros2_cuda_ipc_core/gpu_buffer_pool.hpp"

#include <stdexcept>

#include "ros2_cuda_ipc_core/cuda_support.hpp"

namespace ros2_cuda_ipc_core {

GpuBufferPool::GpuBufferPool(std::size_t size) : slots_(size) {}

GpuBufferPool::GpuBufferPool(const PoolOptions& opts)
    : slots_(opts.pool_size),
      device_ptrs_(opts.pool_size, nullptr),
      events_(opts.pool_size, nullptr),
      bytes_per_slot_(opts.bytes_per_slot) {
  const bool needs_cuda = (bytes_per_slot_ > 0) || opts.events_enabled;
  if (needs_cuda && !cuda_is_available()) {
    throw std::runtime_error("CUDA is not available for GpuBufferPool");
  }

  bool ok = true;
  if (bytes_per_slot_ > 0) {
    for (std::size_t i = 0; i < opts.pool_size; ++i) {
      void* p = cuda_allocate(bytes_per_slot_);
      if (!p) {
        ok = false;
        break;
      }
      device_ptrs_[i] = p;
    }
  }

  if (ok && opts.events_enabled) {
    for (std::size_t i = 0; i < opts.pool_size; ++i) {
      cudaEvent_t e = cuda_event_create();
      if (!e) {
        ok = false;
        break;
      }
      events_[i] = e;
    }
  }

  if (!ok) {
    for (cudaEvent_t e : events_) {
      if (e) cuda_event_destroy(e);
    }
    for (void* p : device_ptrs_) {
      if (p) cuda_free(p);
    }
    throw std::runtime_error(
        "Failed to initialize CUDA resources for GpuBufferPool");
  }

  events_enabled_ = opts.events_enabled;
  producer_stream_ = opts.producer_stream;
}

GpuBufferPool::~GpuBufferPool() {
  for (cudaEvent_t e : events_) {
    if (e) cuda_event_destroy(e);
  }
  for (void* p : device_ptrs_) {
    if (p) cuda_free(p);
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
  if (id >= device_ptrs_.size()) return nullptr;
  return device_ptrs_[id];
}

bool GpuBufferPool::ipc_handle(std::size_t id,
                               CudaIpcMemHandle& out_handle) const {
  if (id >= device_ptrs_.size()) return false;
  void* ptr = device_ptrs_[id];
  if (!ptr) return false;
  return cuda_ipc_get_mem_handle(ptr, &out_handle);
}

bool GpuBufferPool::record_ready(std::size_t id) {
  if (!events_enabled_) return false;
  if (id >= events_.size()) return false;
  cudaEvent_t evt = events_[id];
  if (!evt) return false;
  if (producer_stream_) {
    return cuda_event_record_on_stream(evt, producer_stream_);
  }
  return cuda_event_record(evt);
}

bool GpuBufferPool::ipc_event_handle(std::size_t id,
                                     CudaIpcEventHandle& out_handle) const {
  if (!events_enabled_) return false;
  if (id >= events_.size()) return false;
  cudaEvent_t evt = events_[id];
  if (!evt) return false;
  return cuda_event_get_ipc_handle(evt, &out_handle);
}

bool GpuBufferPool::record_ready_on_stream(std::size_t id,
                                           cudaStream_t stream) {
  if (!events_enabled_) return false;
  if (id >= events_.size()) return false;
  cudaEvent_t evt = events_[id];
  if (!evt) return false;
  return cuda_event_record_on_stream(evt, stream);
}

}  // namespace ros2_cuda_ipc_core
