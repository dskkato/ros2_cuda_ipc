#include "ros2_cuda_ipc_core/scoped_mapped_frame.hpp"

namespace ros2_cuda_ipc_core {

ScopedMappedFrame::ScopedMappedFrame(GpuBufferMapper& mapper, uint32_t slot_id,
                                     const CudaIpcMemHandle* mem_handle,
                                     const CudaIpcEventHandle* evt_handle,
                                     void* stream, std::string shm_name,
                                     uint64_t seq, bool sync_on_dtor)
    : mapper_(mapper),
      slot_(slot_id),
      stream_(stream),
      shm_name_(std::move(shm_name)),
      seq_(seq),
      sync_on_dtor_(sync_on_dtor) {
  if (evt_handle) {
    evt_ptr_ = mapper_.open_event(slot_, *evt_handle);
    if (evt_ptr_ && stream_) {
      (void)cuda_stream_wait_event(stream_, evt_ptr_);
    }
  }
  if (mem_handle) {
    mem_ptr_ = mapper_.open_memory(slot_, *mem_handle);
  }
}

ScopedMappedFrame::~ScopedMappedFrame() {
  if (sync_on_dtor_ && stream_) {
    (void)cuda_stream_synchronize(stream_);
  }
  if (!shm_name_.empty()) {
    (void)shm_decrement(shm_name_, slot_, seq_);
  }
}

}  // namespace ros2_cuda_ipc_core
