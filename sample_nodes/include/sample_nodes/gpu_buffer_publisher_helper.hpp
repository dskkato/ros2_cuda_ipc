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

class GpuBufferPublisherHelper {
 public:
  struct Frame {
    uint32_t slot_id{0};
    void* device_ptr{nullptr};
    uint64_t size_bytes{0};
    uint64_t pitch_bytes{0};
  };

  GpuBufferPublisherHelper(ros2_cuda_ipc_core::GpuBufferPool& pool,
                           ros2_cuda_ipc_core::LeaseManager* lease_mgr,
                           ros2_cuda_ipc_core::cudaStream_t producer_stream);

  // Attempts to borrow a slot and returns basic info for filling GPU memory.
  std::optional<Frame> borrow_frame(uint32_t width, uint32_t height,
                                    uint32_t channels);

  // Fills the message fields, exports IPC handles, records the ready event and
  // optionally starts a lease. Returns true if plane[0] was populated.
  bool finalize_and_fill(const Frame& f, uint64_t seq_id,
                         int expected_consumers, std::string_view shm_owner,
                         uint32_t width, uint32_t height, uint32_t channels,
                         uint8_t layout, uint8_t format,
                         ros2_cuda_ipc_msgs::msg::GpuBuffer& out);

 private:
  ros2_cuda_ipc_core::GpuBufferPool& pool_;
  ros2_cuda_ipc_core::LeaseManager* lease_mgr_{nullptr};
  ros2_cuda_ipc_core::cudaStream_t stream_{nullptr};
};

}  // namespace sample_nodes

#endif  // SAMPLE_NODES_GPU_BUFFER_PUBLISHER_HELPER_HPP_
