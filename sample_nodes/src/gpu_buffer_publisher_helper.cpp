#include "sample_nodes/gpu_buffer_publisher_helper.hpp"

#include <cstring>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
#include "ros2_cuda_ipc_core/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_core/lease_manager.hpp"
#include "ros2_cuda_ipc_core/version.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_buffer.hpp"

namespace sample_nodes {

GpuBufferPublisherHelper::GpuBufferPublisherHelper(
    ros2_cuda_ipc_core::GpuBufferPool& pool,
    ros2_cuda_ipc_core::LeaseManager* lease_mgr,
    ros2_cuda_ipc_core::cudaStream_t producer_stream)
    : pool_(pool), lease_mgr_(lease_mgr), stream_(producer_stream) {}

std::optional<GpuBufferPublisherHelper::Frame>
GpuBufferPublisherHelper::borrow_frame(uint32_t width, uint32_t height,
                                       uint32_t channels) {
  auto slot = pool_.borrow();
  if (!slot.has_value()) return std::nullopt;
  Frame f;
  f.slot_id = static_cast<uint32_t>(*slot);
  f.device_ptr = pool_.device_ptr(*slot);
  f.pitch_bytes = static_cast<uint64_t>(width) * channels;
  f.size_bytes = static_cast<uint64_t>(width) * height * channels;
  return f;
}

bool GpuBufferPublisherHelper::finalize_and_fill(
    const Frame& f, uint64_t seq_id, int expected_consumers,
    std::string_view shm_owner, uint32_t width, uint32_t height,
    uint32_t channels, uint8_t layout, uint8_t format,
    ros2_cuda_ipc_msgs::msg::GpuBuffer& out) {
  out.abi_version = ros2_cuda_ipc_core::kAbiVersion;
  auto id = ros2_cuda_ipc_core::cuda_get_device_id_string();
  out.device_uuid = id.empty() ? std::string("unknown") : id;
  out.seq_id = seq_id;
  out.pool_slot_id = f.slot_id;
  out.layout = layout;
  out.format = format;
  out.width = width;
  out.height = height;
  out.channels = channels;
  out.shm_name.clear();

  ros2_cuda_ipc_core::CudaIpcMemHandle mh{};
  bool have_mem = pool_.ipc_handle(f.slot_id, mh);
  if (!have_mem) {
    out.plane_count = 0;
    (void)pool_.release(f.slot_id);
    return false;
  }

  out.plane_count = 1;
  out.planes.resize(1);
  out.planes[0].size_bytes = f.size_bytes;
  out.planes[0].pitch_bytes = f.pitch_bytes;
  std::memcpy(out.planes[0].ipc_mem_handle.data(), &mh, sizeof(mh));

  ros2_cuda_ipc_core::CudaIpcEventHandle eh{};
  if (pool_.ipc_event_handle(f.slot_id, eh)) {
    std::memcpy(out.ipc_event_handle.data(), &eh, sizeof(eh));
    if (stream_) {
      (void)pool_.record_ready_on_stream(f.slot_id, stream_);
    } else {
      (void)pool_.record_ready(f.slot_id);
    }
  }

  if (expected_consumers > 0 && lease_mgr_) {
    lease_mgr_->set_owner(std::string(shm_owner));
    auto created =
        lease_mgr_->start_lease(f.slot_id, seq_id, expected_consumers);
    if (created) {
      out.shm_name = *created;
    }
  } else {
    (void)pool_.release(f.slot_id);
  }

  return true;
}

}  // namespace sample_nodes
