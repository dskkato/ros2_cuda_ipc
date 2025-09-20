#ifndef ROS2_CUDA_IPC_CORE_SCOPED_MAPPED_FRAME_HPP_
#define ROS2_CUDA_IPC_CORE_SCOPED_MAPPED_FRAME_HPP_

#include <cstdint>
#include <string>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
#include "ros2_cuda_ipc_core/gpu_buffer_mapper.hpp"
#include "ros2_cuda_ipc_core/shm_release.hpp"

namespace ros2_cuda_ipc_core {

// RAII helper for a single received frame on the consumer side.
// - Ensures event is opened (and optionally waited on a stream)
// - Optionally opens and exposes the device pointer via GpuBufferMapper cache
// - On destruction, optionally synchronizes the stream, then decrements SHM
//   refcount if a shm_name was provided.
class ScopedMappedFrame {
 public:
  ScopedMappedFrame(GpuBufferMapper& mapper, uint32_t slot_id,
                    const CudaIpcMemHandle* mem_handle,
                    const CudaIpcEventHandle* evt_handle, cudaStream_t stream,
                    std::string shm_name, uint64_t seq,
                    bool sync_on_dtor = true);

  ~ScopedMappedFrame();

  // Returns cached device memory pointer (may be nullptr if open failed or
  // mem_handle was null).
  void* device_ptr() const { return mem_ptr_; }

 private:
  GpuBufferMapper& mapper_;
  uint32_t slot_;
  cudaStream_t stream_;
  std::string shm_name_;
  uint64_t seq_;
  bool sync_on_dtor_;
  void* mem_ptr_{nullptr};
  cudaEvent_t evt_ptr_{nullptr};
};

}  // namespace ros2_cuda_ipc_core

#endif  // ROS2_CUDA_IPC_CORE_SCOPED_MAPPED_FRAME_HPP_
