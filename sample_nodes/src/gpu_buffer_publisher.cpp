#include <unistd.h>

#include <chrono>
#include <cstring>
#include <optional>
#include <rclcpp/rclcpp.hpp>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
#include "ros2_cuda_ipc_core/zero_copy_publisher.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_buffer.hpp"
#include "sample_nodes/sample_cuda_utils.hpp"

class DummyPublisher : public rclcpp::Node {
 public:
  DummyPublisher() : Node("dummy_gpu_buffer_publisher") {
    using namespace std::chrono_literals;
    pub_ = this->create_publisher<ros2_cuda_ipc_msgs::msg::GpuBuffer>(
        "gpu_buffer", 10);
    // Prepare a tiny pool (1 slot, 4 MiB) using PoolOptions.
    ros2_cuda_ipc_core::PoolOptions opts;
    opts.pool_size = 1;
    opts.bytes_per_slot = 4u * 1024u * 1024u;
    opts.events_enabled = true;
    if (ros2_cuda_ipc_core::cuda_is_available()) {
      producer_stream_ = ros2_cuda_ipc_core::cuda_stream_create();
      opts.producer_stream = producer_stream_;
    }
    zcp_ = std::make_unique<ros2_cuda_ipc_core::ZeroCopyPublisher>(
        opts, /*lease_timeout_ms=*/lease_timeout_ms_, /*shm_owner=*/"");

    timer_ = this->create_wall_timer(1s, [this]() { publish_once(); });

    // Parameters
    lease_timeout_ms_ = this->declare_parameter<int>("lease_timeout_ms", 3000);
    // expected_consumers: -1 means auto (use get_subscription_count()).
    expected_consumers_ =
        this->declare_parameter<int>("expected_consumers", -1);
    // Owner string for SHM names; default to sanitized(FQN)_<epoch>_<pid>
    auto owner_param = this->declare_parameter<std::string>("shm_owner", "");
    if (!owner_param.empty()) {
      shm_owner_ = ros2_cuda_ipc_core::sanitize_shm_owner(owner_param);
    } else {
      auto base = ros2_cuda_ipc_core::sanitize_shm_owner(
          this->get_fully_qualified_name());
      const auto now = std::chrono::system_clock::now();
      const auto secs = std::chrono::duration_cast<std::chrono::seconds>(
                            now.time_since_epoch())
                            .count();
      char buf[96];
      std::snprintf(buf, sizeof(buf), "%s_%lld_%d", base.c_str(),
                    static_cast<long long>(secs), static_cast<int>(::getpid()));
      shm_owner_ = ros2_cuda_ipc_core::sanitize_shm_owner(buf);
    }

    // Configure wrapper owner and timeout
    zcp_->set_owner(shm_owner_);
    zcp_->set_timeout_ms(lease_timeout_ms_);
  }

  ~DummyPublisher() {
    if (producer_stream_) {
      (void)ros2_cuda_ipc_core::cuda_stream_destroy(producer_stream_);
      producer_stream_ = nullptr;
    }
    // No explicit cleanup needed; ZeroCopyPublisher manages leases
  }

 private:
  void publish_once() {
    // Tick handled inside produce_and_publish
    ros2_cuda_ipc_msgs::msg::GpuBuffer msg;
    msg.seq_id = count_++;
    msg.layout = ros2_cuda_ipc_msgs::msg::GpuBuffer::LAYOUT_LINEAR;
    msg.format = ros2_cuda_ipc_msgs::msg::GpuBuffer::FORMAT_BGR8;
    msg.width = 640;
    msg.height = 480;
    msg.channels = 3;
    msg.stamp = this->now();
    msg.frame_id = "frame";

    const int expected_count =
        expected_consumers_ < 0
            ? static_cast<int>(pub_->get_subscription_count())
            : expected_consumers_;

    auto fill = [&](void* dev, uint32_t w, uint32_t h, uint32_t c,
                    cudaStream_t s) {
      const uint64_t size_bytes = static_cast<uint64_t>(w) * h * c;
      unsigned char pattern = static_cast<unsigned char>(count_ & 0xFF);
      (void)sample_nodes::cuda_fill_u8(dev, pattern, size_bytes, s);
    };

    bool ok = zcp_->produce_and_publish(*pub_, msg, expected_count, fill,
                                        /*blocking=*/true);
    if (!ok) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Publishing metadata only (no CUDA mem/slot)");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Published seq=%lu, expected consumers=%d", msg.seq_id,
                  expected_count);
    }
  }

  rclcpp::Publisher<ros2_cuda_ipc_msgs::msg::GpuBuffer>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<ros2_cuda_ipc_core::ZeroCopyPublisher> zcp_;
  int lease_timeout_ms_{3000};
  int expected_consumers_{-1};
  uint64_t count_{0};
  cudaStream_t producer_stream_{nullptr};
  std::string shm_owner_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyPublisher>());
  rclcpp::shutdown();
  return 0;
}
