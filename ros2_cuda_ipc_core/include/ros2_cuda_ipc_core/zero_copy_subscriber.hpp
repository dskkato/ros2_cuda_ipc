#ifndef ROS2_CUDA_IPC_CORE_ZERO_COPY_SUBSCRIBER_HPP_
#define ROS2_CUDA_IPC_CORE_ZERO_COPY_SUBSCRIBER_HPP_

#include <functional>
#include <rclcpp/rclcpp.hpp>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
#include "ros2_cuda_ipc_core/gpu_buffer_mapper.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_buffer.hpp"

namespace ros2_cuda_ipc_core {

class ZeroCopySubscriber {
 public:
  using Callback =
      std::function<void(const ros2_cuda_ipc_msgs::msg::GpuBuffer&, void*)>;

  ZeroCopySubscriber(rclcpp::Node& node, const std::string& topic, Callback cb);
  ~ZeroCopySubscriber();

 private:
  void handle_message(const ros2_cuda_ipc_msgs::msg::GpuBuffer& msg);

  rclcpp::Subscription<ros2_cuda_ipc_msgs::msg::GpuBuffer>::SharedPtr sub_;
  GpuBufferMapper mapper_;
  cudaStream_t stream_{nullptr};
  Callback callback_;
  rclcpp::Logger logger_;
};

}  // namespace ros2_cuda_ipc_core

#endif  // ROS2_CUDA_IPC_CORE_ZERO_COPY_SUBSCRIBER_HPP_
