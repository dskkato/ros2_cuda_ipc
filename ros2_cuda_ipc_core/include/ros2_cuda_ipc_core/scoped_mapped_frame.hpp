#ifndef ROS2_CUDA_IPC_CORE_SCOPED_MAPPED_FRAME_HPP_
#define ROS2_CUDA_IPC_CORE_SCOPED_MAPPED_FRAME_HPP_

#include <cstdint>
#include <string>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
#include "ros2_cuda_ipc_core/gpu_buffer_mapper.hpp"
#include "ros2_cuda_ipc_core/shm_release.hpp"

namespace ros2_cuda_ipc_core {

/**
 * @brief RAII helper for a single GPU frame on the subscriber side.
 *
 * The constructor opens the IPC event and (optionally) waits on the supplied
 * CUDA stream. If a memory handle is present it is mapped through
 * GpuBufferMapper and cached for reuse. When the object goes out of scope the
 * destructor optionally synchronizes the stream and decrements the shared
 * memory reference count.
 *
 * Typical sequence:
 * 1. Construct with handles from the received message.
 * 2. Call device_ptr() and launch kernels on the provided stream.
 * 3. Let the object destruct at scope exit to release the SHM slot.
 *
 * @warning The destructor performs SHM decrement and may block if
 * sync_on_dtor is true. Do not copy the object or let it outlive the stream
 * that uses the mapped memory.
 */
class ScopedMappedFrame {
 public:
  /**
   * @brief Map handles for a frame and optionally wait on a CUDA stream.
   *
   * @param mapper       Cache used for mem/event handle mapping.
   * @param slot_id      Slot identifier within the pool.
   * @param mem_handle   IPC memory handle (may be nullptr).
   * @param evt_handle   IPC event handle (may be nullptr).
   * @param stream       CUDA stream to wait on; may be nullptr.
   * @param shm_name     SHM control block name for refcount decrement.
   * @param seq          Sequence number for logging only.
   * @param sync_on_dtor If true, synchronize the stream before decrementing.
   */
  ScopedMappedFrame(GpuBufferMapper& mapper, uint32_t slot_id,
                    const CudaIpcMemHandle* mem_handle,
                    const CudaIpcEventHandle* evt_handle, void* stream,
                    std::string shm_name, uint64_t seq,
                    bool sync_on_dtor = true);

  /**
   * @brief Synchronizes and releases SHM refcount if necessary.
   */
  ~ScopedMappedFrame();

  /**
   * @brief Returns cached device memory pointer.
   *
   * Pointer may be nullptr if the memory handle was null or mapping failed.
   */
  void* device_ptr() const { return mem_ptr_; }

 private:
  GpuBufferMapper& mapper_;
  uint32_t slot_;
  void* stream_;
  std::string shm_name_;
  uint64_t seq_;
  bool sync_on_dtor_;
  void* mem_ptr_{nullptr};
  void* evt_ptr_{nullptr};
};

}  // namespace ros2_cuda_ipc_core

#endif  // ROS2_CUDA_IPC_CORE_SCOPED_MAPPED_FRAME_HPP_
