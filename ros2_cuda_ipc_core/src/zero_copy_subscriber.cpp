#include "ros2_cuda_ipc_core/zero_copy_subscriber.hpp"

#include <cstring>

namespace ros2_cuda_ipc_core {

ZeroCopySubscriber::ZeroCopySubscriber(rclcpp::Node& node,
                                       const std::string& topic,
                                       const Callback& cb)
    : cb_(cb) {
  if (cuda_is_available()) {
    stream_ = cuda_stream_create();
  }
  sub_ = node.create_subscription<ros2_cuda_ipc_msgs::msg::GpuBuffer>(
      topic, 10,
      std::bind(&ZeroCopySubscriber::handle_message, this,
                std::placeholders::_1));
}

ZeroCopySubscriber::~ZeroCopySubscriber() {
  if (stream_) {
    (void)cuda_stream_destroy(stream_);
    stream_ = nullptr;
  }
  mapper_.reset();
}

void ZeroCopySubscriber::handle_message(
    const ros2_cuda_ipc_msgs::msg::GpuBuffer& msg) {
  if (prev_abi_version_ != 0 && prev_abi_version_ != msg.abi_version) {
    mapper_.reset();
  }
  if (!prev_device_id_.empty() && prev_device_id_ != msg.device_uuid) {
    mapper_.reset();
  }
  prev_abi_version_ = msg.abi_version;
  prev_device_id_ = msg.device_uuid;

  CudaIpcMemHandle mem{};
  CudaIpcMemHandle* mem_ptr = nullptr;
  if (msg.plane_count > 0 && msg.planes.size() >= 1) {
    std::memcpy(&mem, msg.planes[0].ipc_mem_handle.data(), sizeof(mem));
    mem_ptr = &mem;
  }
  CudaIpcEventHandle evt{};
  std::memcpy(&evt, msg.ipc_event_handle.data(), sizeof(evt));
  ScopedMappedFrame frame(mapper_, msg.pool_slot_id, mem_ptr, &evt, stream_,
                          msg.shm_name, msg.seq_id, true);
  if (cb_) {
    cb_(msg, frame.device_ptr());
  }
}

}  // namespace ros2_cuda_ipc_core
