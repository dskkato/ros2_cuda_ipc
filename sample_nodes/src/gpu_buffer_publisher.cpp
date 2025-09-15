#include <unistd.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
#include "ros2_cuda_ipc_core/zero_copy_publisher.hpp"
#include "sample_nodes/sample_cuda_utils.hpp"

class DummyPublisher : public rclcpp::Node {
 public:
  DummyPublisher() : Node("dummy_gpu_buffer_publisher") {
    using namespace std::chrono_literals;
    ros2_cuda_ipc_core::PoolOptions opts;
    opts.pool_size = 1;
    opts.bytes_per_slot = 4u * 1024u * 1024u;
    opts.events_enabled = true;
    if (ros2_cuda_ipc_core::cuda_is_available()) {
      producer_stream_ = ros2_cuda_ipc_core::cuda_stream_create();
      opts.producer_stream = producer_stream_;
    }
    pub_ = std::make_unique<ros2_cuda_ipc_core::ZeroCopyPublisher>(
        *this, "gpu_buffer", opts);

    timer_ = this->create_wall_timer(1s, [this]() { publish_once(); });
  }

  ~DummyPublisher() {
    if (producer_stream_) {
      (void)ros2_cuda_ipc_core::cuda_stream_destroy(producer_stream_);
      producer_stream_ = nullptr;
    }
  }

 private:
  void publish_once() {
    ros2_cuda_ipc_msgs::msg::GpuBuffer msg;
    msg.seq_id = count_++;
    msg.layout = ros2_cuda_ipc_msgs::msg::GpuBuffer::LAYOUT_LINEAR;
    msg.format = ros2_cuda_ipc_msgs::msg::GpuBuffer::FORMAT_BGR8;
    msg.width = 640;
    msg.height = 480;
    msg.channels = 3;
    msg.stamp = this->now();
    msg.frame_id = "frame";

    // -1 for expected_consumers triggers auto-detection of subscription count.
    pub_->publish(
        msg, /*expected_consumers=*/-1,
        [this](void* ptr, uint64_t size, cudaStream_t stream) {
          unsigned char pattern = static_cast<unsigned char>(count_ & 0xFF);
          (void)sample_nodes::cuda_fill_u8(ptr, pattern, size, stream);
        });
  }

  std::unique_ptr<ros2_cuda_ipc_core::ZeroCopyPublisher> pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  cudaStream_t producer_stream_{nullptr};
  uint64_t count_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyPublisher>());
  rclcpp::shutdown();
  return 0;
}
