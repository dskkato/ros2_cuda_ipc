#include "ros2_cuda_ipc_core/zero_copy_subscriber.hpp"

#include <cstring>

#include "ros2_cuda_ipc_core/scoped_mapped_frame.hpp"

namespace ros2_cuda_ipc_core {

ZeroCopySubscriber::ZeroCopySubscriber(rclcpp::Node& node,
                                       const std::string& topic, Callback cb)
    : callback_(std::move(cb)), logger_(node.get_logger()) {
  if (cuda_is_available()) {
    stream_ = cuda_stream_create();
  }
  sub_ = node.create_subscription<ros2_cuda_ipc_msgs::msg::GpuBuffer>(
      topic, 10, [this](const ros2_cuda_ipc_msgs::msg::GpuBuffer& msg) {
        handle_message(msg);
      });
}

ZeroCopySubscriber::~ZeroCopySubscriber() {
  if (stream_) {
    (void)cuda_stream_destroy(stream_);
    stream_ = nullptr;
  }
}

void ZeroCopySubscriber::handle_message(
    const ros2_cuda_ipc_msgs::msg::GpuBuffer& msg) {
  if (!mapper_.validate_handles(msg.pool_slot_id, msg.abi_version,
                                msg.device_uuid)) {
    RCLCPP_WARN(logger_, "Invalidated cached handles for slot %u",
                msg.pool_slot_id);
  }
  CudaIpcMemHandle mem_tmp{};
  CudaIpcMemHandle* mem_ptr = nullptr;
  if (msg.plane_count > 0 && msg.planes.size() >= 1) {
    std::memcpy(&mem_tmp, msg.planes[0].ipc_mem_handle.data(), sizeof(mem_tmp));
    mem_ptr = &mem_tmp;
  }
  CudaIpcEventHandle evt_tmp{};
  std::memcpy(&evt_tmp, msg.ipc_event_handle.data(), sizeof(evt_tmp));
  {
    ScopedMappedFrame frame(mapper_, msg.pool_slot_id, mem_ptr, &evt_tmp,
                            stream_, msg.shm_name, msg.seq_id,
                            /*sync_on_dtor=*/true);
    if (callback_) {
      callback_(msg, frame.device_ptr());
    }
  }
}

}  // namespace ros2_cuda_ipc_core
