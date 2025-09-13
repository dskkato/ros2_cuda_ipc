#include <rclcpp/rclcpp.hpp>
#include "ros2_cuda_ipc_msgs/msg/gpu_buffer.hpp"

class DummyPublisher : public rclcpp::Node {
public:
  DummyPublisher() : Node("dummy_gpu_buffer_publisher") {
    pub_ = this->create_publisher<ros2_cuda_ipc_msgs::msg::GpuBuffer>("gpu_buffer", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() { publish_once(); });
  }
private:
  void publish_once() {
    ros2_cuda_ipc_msgs::msg::GpuBuffer msg;
    msg.abi_version = 1;
    msg.device_uuid = "dummy";
    msg.seq_id = count_++;
    msg.pool_slot_id = 0;
    msg.plane_count = 0;
    msg.format = 1;
    msg.layout = 0;
    msg.width = 0;
    msg.height = 0;
    msg.channels = 0;
    msg.stamp = this->now();
    msg.frame_id = "frame";
    msg.shm_name = "";
    pub_->publish(msg);
  }
  rclcpp::Publisher<ros2_cuda_ipc_msgs::msg::GpuBuffer>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint64_t count_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyPublisher>());
  rclcpp::shutdown();
  return 0;
}
