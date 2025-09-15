#include <rclcpp/rclcpp.hpp>

#include "ros2_cuda_ipc_core/zero_copy_subscriber.hpp"

class DummySubscriber : public rclcpp::Node {
 public:
  DummySubscriber() : Node("dummy_gpu_buffer_subscriber") {
    sub_ = std::make_unique<ros2_cuda_ipc_core::ZeroCopySubscriber>(
        *this, "gpu_buffer",
        [this](const ros2_cuda_ipc_msgs::msg::GpuBuffer& msg, void*) {
          RCLCPP_INFO(this->get_logger(), "Received seq_id %lu", msg.seq_id);
        });
  }

 private:
  std::unique_ptr<ros2_cuda_ipc_core::ZeroCopySubscriber> sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummySubscriber>());
  rclcpp::shutdown();
  return 0;
}
