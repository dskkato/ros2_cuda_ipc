#ifndef ROS2_CUDA_IPC_CORE_ZERO_COPY_SUBSCRIBER_HPP_
#define ROS2_CUDA_IPC_CORE_ZERO_COPY_SUBSCRIBER_HPP_

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
#include "ros2_cuda_ipc_core/gpu_buffer_mapper.hpp"
#include "ros2_cuda_ipc_core/scoped_mapped_frame.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_buffer.hpp"

namespace ros2_cuda_ipc_core {

// Subscriber-side helper that maps incoming GPU buffers and waits on their
// readiness events, providing the device pointer to a user callback.
class ZeroCopySubscriber {
 public:
  using Callback =
      std::function<void(const ros2_cuda_ipc_msgs::msg::GpuBuffer&, void*)>;

  ZeroCopySubscriber(rclcpp::Node& node, const std::string& topic,
                     const Callback& cb);
  ~ZeroCopySubscriber();

 private:
  void handle_message(const ros2_cuda_ipc_msgs::msg::GpuBuffer& msg);

  rclcpp::Subscription<ros2_cuda_ipc_msgs::msg::GpuBuffer>::SharedPtr sub_;
  GpuBufferMapper mapper_;
  cudaStream_t stream_{nullptr};
  Callback cb_;
  uint32_t prev_abi_version_{0};
  std::string prev_device_id_;
};

}  // namespace ros2_cuda_ipc_core

#endif  // ROS2_CUDA_IPC_CORE_ZERO_COPY_SUBSCRIBER_HPP_
