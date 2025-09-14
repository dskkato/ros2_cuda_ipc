// Lightweight helper for assembling ros2_cuda_ipc_msgs::msg::GpuBuffer on the
// publisher side. Depends only on ros2_cuda_ipc_core and the message package.
#ifndef SAMPLE_NODES_GPU_BUFFER_PUBLISHER_HELPER_HPP_
#define SAMPLE_NODES_GPU_BUFFER_PUBLISHER_HELPER_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "ros2_cuda_ipc_core/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_core/lease_manager.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_buffer.hpp"

namespace sample_nodes {

/**
 * @brief Convenience wrapper for publishing GPU-backed frames.
 *
 * The helper hides the ceremony of borrowing a slot from GpuBufferPool,
 * recording a CUDA event on the producer stream and starting a lease via
 * LeaseManager.  It is intended for simple samples and single-threaded
 * publishers.
 *
 * Typical sequence:
 * 1. Call borrow_frame() to reserve a slot and obtain a device pointer.
 * 2. Write GPU data into the provided pointer.
 * 3. Call finalize_and_fill() to populate the message and optionally start a
 *    lease.
 *
 * @warning finalize_and_fill() must be called exactly once for each successful
 * borrow_frame() or the slot will remain reserved and leak.
 */
class GpuBufferPublisherHelper {
 public:
  /** Basic information about a borrowed frame. */
  struct Frame {
    uint32_t slot_id{0};       /**< Slot index in the pool. */
    void* device_ptr{nullptr}; /**< Writable device pointer. */
    uint64_t size_bytes{0};    /**< Total byte size of the frame. */
    uint64_t pitch_bytes{0};   /**< Row pitch in bytes. */
  };

  /**
   * @brief Create a helper bound to a pool, optional LeaseManager and stream.
   */
  GpuBufferPublisherHelper(ros2_cuda_ipc_core::GpuBufferPool& pool,
                           ros2_cuda_ipc_core::LeaseManager* lease_mgr,
                           void* producer_stream);

  /**
   * @brief Borrow a frame slot from the pool.
   *
   * @return Frame information including device pointer on success; std::nullopt
   * on failure.
   */
  std::optional<Frame> borrow_frame(uint32_t width, uint32_t height,
                                    uint32_t channels);

  /**
   * @brief Finalize a borrowed frame and populate a message.
   *
   * Exports IPC handles, records a ready event and optionally starts a lease.
   *
   * @return true if plane[0] was populated.
   */
  bool finalize_and_fill(const Frame& f, uint64_t seq_id,
                         int expected_consumers, std::string_view shm_owner,
                         uint32_t width, uint32_t height, uint32_t channels,
                         uint8_t layout, uint8_t format,
                         ros2_cuda_ipc_msgs::msg::GpuBuffer& out);

 private:
  ros2_cuda_ipc_core::GpuBufferPool& pool_; /**< Underlying pool. */
  ros2_cuda_ipc_core::LeaseManager* lease_mgr_{
      nullptr};           /**< Optional lease manager. */
  void* stream_{nullptr}; /**< Producer CUDA stream. */
};

}  // namespace sample_nodes

#endif  // SAMPLE_NODES_GPU_BUFFER_PUBLISHER_HELPER_HPP_
