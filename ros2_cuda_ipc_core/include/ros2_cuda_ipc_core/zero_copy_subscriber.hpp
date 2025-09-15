// Convenience subscriber-side wrapper that encapsulates GpuBufferMapper
// and provides a one-shot consume() API with RAII mapping + SHM release.
#ifndef ROS2_CUDA_IPC_CORE_ZERO_COPY_SUBSCRIBER_HPP_
#define ROS2_CUDA_IPC_CORE_ZERO_COPY_SUBSCRIBER_HPP_

#include <cstdint>
#include <cstring>
#include <functional>

#include "ros2_cuda_ipc_core/gpu_buffer_mapper.hpp"
#include "ros2_cuda_ipc_core/scoped_mapped_frame.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_buffer.hpp"

namespace ros2_cuda_ipc_core {

class ZeroCopySubscriber {
 public:
  // If stream is nullptr and CUDA is available, a stream is created and owned
  // by the instance; otherwise, the provided stream is used but not owned.
  explicit ZeroCopySubscriber(cudaStream_t stream = nullptr);
  ~ZeroCopySubscriber();

  // One-shot consume: validates handles, opens event+mem, waits on the stream
  // and invokes `fn(device_ptr, width, height, channels, stream)`. On scope
  // exit, it decrements SHM refcount (when provided) and optionally syncs.
  template <typename Fn>
  void consume(const ros2_cuda_ipc_msgs::msg::GpuBuffer& msg, Fn fn,
               bool sync_on_dtor = true) {
    (void)mapper_.validate_handles(msg.pool_slot_id, msg.abi_version,
                                   msg.device_uuid);

    CudaIpcMemHandle* mem_ptr = nullptr;
    CudaIpcMemHandle mem_tmp{};
    if (msg.plane_count > 0 && msg.planes.size() >= 1) {
      std::memcpy(&mem_tmp, msg.planes[0].ipc_mem_handle.data(),
                  sizeof(mem_tmp));
      mem_ptr = &mem_tmp;
    }
    CudaIpcEventHandle evt_tmp{};
    std::memcpy(&evt_tmp, msg.ipc_event_handle.data(), sizeof(evt_tmp));

    ScopedMappedFrame frame(mapper_, msg.pool_slot_id, mem_ptr, &evt_tmp,
                            stream_, msg.shm_name, msg.seq_id, sync_on_dtor);

    if (void* dev = frame.device_ptr()) {
      fn(dev, msg.width, msg.height, msg.channels, stream_);
    } else {
      // Even when no dev ptr is provided, invoke callback with nullptr so the
      // user can handle metadata-only frames.
      fn(nullptr, msg.width, msg.height, msg.channels, stream_);
    }
  }

  cudaStream_t stream() const { return stream_; }

 private:
  GpuBufferMapper mapper_;
  cudaStream_t stream_{nullptr};
  bool own_stream_{false};
};

}  // namespace ros2_cuda_ipc_core

#endif  // ROS2_CUDA_IPC_CORE_ZERO_COPY_SUBSCRIBER_HPP_
