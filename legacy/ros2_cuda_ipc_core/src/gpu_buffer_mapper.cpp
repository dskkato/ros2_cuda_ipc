#include "ros2_cuda_ipc_core/gpu_buffer_mapper.hpp"

#include <exception>

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

void GpuBufferMapper::set_entry_metadata(Entry& e, uint32_t abi_version,
                                         std::string_view device_uuid) {
  e.abi_version = abi_version;
  e.device_uuid = std::string(device_uuid);
}

bool GpuBufferMapper::validate_handles(uint32_t slot_id, uint32_t abi_version,
                                       std::string_view device_uuid) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = cache_.find(slot_id);
  if (it == cache_.end()) {
    Entry e;
    set_entry_metadata(e, abi_version, device_uuid);
    cache_[slot_id] = std::move(e);
    return true;
  }
  Entry& e = it->second;
  if (e.abi_version != 0 &&
      (e.abi_version != abi_version || e.device_uuid != device_uuid)) {
    close_slot_locked(slot_id);
    Entry new_e;
    set_entry_metadata(new_e, abi_version, device_uuid);
    cache_[slot_id] = std::move(new_e);
    return false;
  }
  set_entry_metadata(e, abi_version, device_uuid);
  return true;
}

void GpuBufferMapper::close_slot(uint32_t slot_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  close_slot_locked(slot_id);
}

void GpuBufferMapper::close_slot_locked(uint32_t slot_id) {
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
