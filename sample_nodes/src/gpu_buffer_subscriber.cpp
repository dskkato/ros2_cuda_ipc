#include <rclcpp/rclcpp.hpp>
#include "ros2_cuda_ipc_msgs/msg/gpu_buffer.hpp"

class DummySubscriber : public rclcpp::Node {
public:
  DummySubscriber() : Node("dummy_gpu_buffer_subscriber") {
    sub_ = this->create_subscription<ros2_cuda_ipc_msgs::msg::GpuBuffer>(
      "gpu_buffer", 10,
      [this](const ros2_cuda_ipc_msgs::msg::GpuBuffer & msg) {
        RCLCPP_INFO(this->get_logger(), "Received seq_id %lu", msg.seq_id);
      });
  }
private:
  rclcpp::Subscription<ros2_cuda_ipc_msgs::msg::GpuBuffer>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummySubscriber>());
  rclcpp::shutdown();
  return 0;
}
