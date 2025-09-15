#include <chrono>
#include <cstring>
#include <rclcpp/rclcpp.hpp>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
#include "ros2_cuda_ipc_core/zero_copy_subscriber.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_buffer.hpp"

class DummySubscriber : public rclcpp::Node {
 public:
  DummySubscriber() : Node("dummy_gpu_buffer_subscriber") {
    // Create a non-blocking CUDA stream if available
    if (ros2_cuda_ipc_core::cuda_is_available()) {
      stream_ = ros2_cuda_ipc_core::cuda_stream_create();
    }

    zcs_ = std::make_unique<ros2_cuda_ipc_core::ZeroCopySubscriber>(stream_);
    sub_ = this->create_subscription<ros2_cuda_ipc_msgs::msg::GpuBuffer>(
        "gpu_buffer", 10,
        [this](const ros2_cuda_ipc_msgs::msg::GpuBuffer& msg) {
          RCLCPP_INFO(this->get_logger(), "Received seq_id %lu", msg.seq_id);
          auto t0 = std::chrono::steady_clock::now();
          zcs_->consume(
              msg,
              [this](void* dev, uint32_t w, uint32_t h, uint32_t c,
                     cudaStream_t s) {
                (void)dev;
                (void)w;
                (void)h;
                (void)c;
                (void)s;
                // Do GPU work here if needed.
              },
              /*sync_on_dtor=*/true);
          auto t1 = std::chrono::steady_clock::now();
          auto ms =
              std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
                  .count();
          RCLCPP_INFO(this->get_logger(), "Event waited ~%ld ms",
                      static_cast<long>(ms));
        });
  }

 private:
  rclcpp::Subscription<ros2_cuda_ipc_msgs::msg::GpuBuffer>::SharedPtr sub_;
  cudaStream_t stream_{nullptr};
  std::unique_ptr<ros2_cuda_ipc_core::ZeroCopySubscriber> zcs_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummySubscriber>());
  rclcpp::shutdown();
  return 0;
}
