#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "ros2_cuda_ipc_core/zero_copy_publisher.hpp"
#include "sample_nodes/sample_cuda_utils.hpp"

class DummyPublisher : public rclcpp::Node {
 public:
  DummyPublisher() : Node("dummy_gpu_buffer_publisher") {
    using namespace std::chrono_literals;
    ros2_cuda_ipc_core::ZeroCopyPublisher::Options opts;
    opts.pool_options.pool_size = 1;
    opts.pool_options.bytes_per_slot = 4u * 1024u * 1024u;
    opts.pool_options.events_enabled = true;
    pub_ = std::make_unique<ros2_cuda_ipc_core::ZeroCopyPublisher>(
        *this, "gpu_buffer", opts);
    timer_ = this->create_wall_timer(1s, [this]() { publish_once(); });
  }

 private:
  void publish_once() {
    ros2_cuda_ipc_msgs::msg::GpuBuffer msg;
    msg.layout = ros2_cuda_ipc_msgs::msg::GpuBuffer::LAYOUT_LINEAR;
    msg.format = ros2_cuda_ipc_msgs::msg::GpuBuffer::FORMAT_BGR8;
    msg.width = 640;
    msg.height = 480;
    msg.channels = 3;
    msg.frame_id = "frame";

    bool ok = pub_->publish(msg, [this](void* device_ptr, uint64_t size_bytes,
                                        cudaStream_t stream) {
      unsigned char pattern = static_cast<unsigned char>(count_ & 0xFF);
      (void)sample_nodes::cuda_fill_u8(device_ptr, pattern, size_bytes, stream);
    });
    if (!ok) {
      RCLCPP_WARN(this->get_logger(),
                  "CUDA IPC handle unavailable. Publishing metadata only.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Publishing seq=%lu", msg.seq_id);
    }
    ++count_;
  }

  std::unique_ptr<ros2_cuda_ipc_core::ZeroCopyPublisher> pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint64_t count_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyPublisher>());
  rclcpp::shutdown();
  return 0;
}
