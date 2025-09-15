#include "ros2_cuda_ipc_core/gpu_buffer_mapper.hpp"

#include <stdexcept>

namespace ros2_cuda_ipc_core {

GpuBufferMapper::~GpuBufferMapper() { reset(); }

void* GpuBufferMapper::open_memory(uint32_t slot_id,
                                   const CudaIpcMemHandle& mem_handle) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto& e = cache_[slot_id];
  if (!e.mem) {
    try {
      e.mem = cuda_ipc_open_mem_handle(mem_handle);
    } catch (const std::exception&) {
      e.mem = nullptr;  // Leave unopened on error
    }
  }
  return e.mem;
}

cudaEvent_t GpuBufferMapper::open_event(uint32_t slot_id,
                                        const CudaIpcEventHandle& evt_handle) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto& e = cache_[slot_id];
  if (!e.evt) {
    try {
      e.evt = cuda_ipc_open_event_handle(evt_handle);
    } catch (const std::exception&) {
      e.evt = nullptr;  // Leave unopened on error
    }
  }
  return e.evt;
}

void* GpuBufferMapper::get_memory(uint32_t slot_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = cache_.find(slot_id);
  if (it == cache_.end()) return nullptr;
  return it->second.mem;
}

cudaEvent_t GpuBufferMapper::get_event(uint32_t slot_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = cache_.find(slot_id);
  if (it == cache_.end()) return nullptr;
  return it->second.evt;
}

bool GpuBufferMapper::wait_ready(uint32_t slot_id, cudaStream_t stream) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = cache_.find(slot_id);
  if (it == cache_.end()) return false;
  cudaEvent_t evt = it->second.evt;
  if (!evt) return false;
  return cuda_stream_wait_event(stream, evt);
}

void GpuBufferMapper::close_slot(uint32_t slot_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = cache_.find(slot_id);
  if (it == cache_.end()) return;
  if (it->second.mem) {
    (void)cuda_ipc_close_mem_handle(it->second.mem);
  }
  if (it->second.evt) {
    (void)cuda_event_destroy(it->second.evt);
  }
  cache_.erase(it);
}

void GpuBufferMapper::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& kv : cache_) {
    if (kv.second.mem) (void)cuda_ipc_close_mem_handle(kv.second.mem);
    if (kv.second.evt) (void)cuda_event_destroy(kv.second.evt);
  }
  cache_.clear();
}

}  // namespace ros2_cuda_ipc_core
