#ifndef ROS2_CUDA_IPC_CORE_ZERO_COPY_PUBLISHER_HPP_
#define ROS2_CUDA_IPC_CORE_ZERO_COPY_PUBLISHER_HPP_

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "ros2_cuda_ipc_core/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_core/lease_manager.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_buffer.hpp"

namespace ros2_cuda_ipc_core {

class ZeroCopyPublisher {
 public:
  struct Options {
    PoolOptions pool_options;
    int lease_timeout_ms{3000};
    std::string shm_owner;
  };

  ZeroCopyPublisher(rclcpp::Node& node, const std::string& topic,
                    Options options = Options());
  ~ZeroCopyPublisher();

  using FillCallback = std::function<void(void*, uint64_t, cudaStream_t)>;

  bool publish(ros2_cuda_ipc_msgs::msg::GpuBuffer& msg,
               FillCallback fill_cb = FillCallback(),
               int expected_consumers = -1);

 private:
  rclcpp::Publisher<ros2_cuda_ipc_msgs::msg::GpuBuffer>::SharedPtr pub_;
  rclcpp::Clock::SharedPtr clock_;
  GpuBufferPool pool_;
  std::unique_ptr<LeaseManager> lease_mgr_;
  cudaStream_t producer_stream_{nullptr};
  uint64_t seq_{0};
  std::string shm_owner_;
};

}  // namespace ros2_cuda_ipc_core

#endif  // ROS2_CUDA_IPC_CORE_ZERO_COPY_PUBLISHER_HPP_
