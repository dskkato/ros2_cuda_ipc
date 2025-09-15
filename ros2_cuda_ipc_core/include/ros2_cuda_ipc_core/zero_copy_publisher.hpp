#ifndef ROS2_CUDA_IPC_CORE_ZERO_COPY_PUBLISHER_HPP_
#define ROS2_CUDA_IPC_CORE_ZERO_COPY_PUBLISHER_HPP_

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
#include "ros2_cuda_ipc_core/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_core/lease_manager.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_buffer.hpp"

namespace ros2_cuda_ipc_core {

// Convenience wrapper that owns a GpuBufferPool and LeaseManager and publishes
// ros2_cuda_ipc_msgs::msg::GpuBuffer with minimal user code. The publish()
// method handles slot borrowing, filling, publishing and releasing in one call.
class ZeroCopyPublisher {
 public:
  using FillCallback = std::function<void(void*, uint64_t, cudaStream_t)>;

  ZeroCopyPublisher(rclcpp::Node& node, const std::string& topic,
                    const PoolOptions& pool_opts, int lease_timeout_ms = 3000);

  ~ZeroCopyPublisher();

  // Publishes a message. The caller fills in metadata fields of msg except for
  // plane information. If expected_consumers < 0 the current subscription
  // count of the underlying publisher is used. expected_consumers == 0 releases
  // immediately after publish. The provided callback may be empty; if
  // non-empty it is invoked with the borrowed device pointer and size before
  // the message is finalized.
  bool publish(ros2_cuda_ipc_msgs::msg::GpuBuffer& msg, int expected_consumers,
               const FillCallback& fill_cb);

 private:
  rclcpp::Publisher<ros2_cuda_ipc_msgs::msg::GpuBuffer>::SharedPtr pub_;
  GpuBufferPool pool_;
  LeaseManager lease_mgr_;
  cudaStream_t stream_{nullptr};
  bool owns_stream_{false};
  std::string shm_owner_;
};

}  // namespace ros2_cuda_ipc_core

#endif  // ROS2_CUDA_IPC_CORE_ZERO_COPY_PUBLISHER_HPP_
