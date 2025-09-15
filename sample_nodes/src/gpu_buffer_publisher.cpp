#include <unistd.h>

#include <chrono>
#include <cstring>
#include <optional>
#include <rclcpp/rclcpp.hpp>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
#include "ros2_cuda_ipc_core/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_core/lease_manager.hpp"
#include "ros2_cuda_ipc_core/shm_release.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_buffer.hpp"
#include "sample_nodes/gpu_buffer_publisher_helper.hpp"
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
    pool_ = std::make_unique<ros2_cuda_ipc_core::GpuBufferPool>(opts);

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

    // Initialize lease manager
    lease_mgr_ = std::make_unique<ros2_cuda_ipc_core::LeaseManager>(
        *pool_, lease_timeout_ms_);
    lease_mgr_->set_owner(shm_owner_);
    // Initialize publisher helper
    helper_ = std::make_unique<sample_nodes::GpuBufferPublisherHelper>(
        *pool_, lease_mgr_.get(), producer_stream_);
  }

  ~DummyPublisher() {
    if (producer_stream_) {
      (void)ros2_cuda_ipc_core::cuda_stream_destroy(producer_stream_);
      producer_stream_ = nullptr;
    }
    if (lease_mgr_) {
      lease_mgr_->cleanup();
    }
  }

 private:
  void publish_once() {
    // Enforce lease timeout and SHM refcount releases
    if (lease_mgr_) {
      (void)lease_mgr_->tick();
    }
    ros2_cuda_ipc_msgs::msg::GpuBuffer msg;
    msg.seq_id = count_++;
    msg.layout = ros2_cuda_ipc_msgs::msg::GpuBuffer::LAYOUT_LINEAR;
    msg.format = ros2_cuda_ipc_msgs::msg::GpuBuffer::FORMAT_BGR8;
    msg.width = 640;
    msg.height = 480;
    msg.channels = 3;
    msg.stamp = this->now();
    msg.frame_id = "frame";

    auto frame = helper_->borrow_frame(msg.width, msg.height, msg.channels);
    if (frame.has_value()) {
      // Fill device buffer with a changing pattern
      const uint64_t size_bytes = frame->size_bytes;
      unsigned char pattern = static_cast<unsigned char>(count_ & 0xFF);
      (void)sample_nodes::cuda_fill_u8(frame->device_ptr, pattern, size_bytes,
                                       producer_stream_);

      const int expected_count =
          expected_consumers_ < 0
              ? static_cast<int>(pub_->get_subscription_count())
              : expected_consumers_;

      bool ok =
          helper_->finalize_and_fill(*frame, expected_count, shm_owner_, msg);
      if (!ok) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "CUDA IPC handle unavailable. Publishing metadata only.");
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "Publishing seq=%lu (slot %u), expected consumers=%d",
                    msg.seq_id, frame->slot_id, expected_count);
      }
    } else {
      msg.plane_count = 0;
      RCLCPP_WARN(this->get_logger(),
                  "Pool exhausted; publishing metadata only");
    }

    pub_->publish(msg);
  }

  rclcpp::Publisher<ros2_cuda_ipc_msgs::msg::GpuBuffer>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<ros2_cuda_ipc_core::GpuBufferPool> pool_;
  std::unique_ptr<ros2_cuda_ipc_core::LeaseManager> lease_mgr_;
  std::unique_ptr<sample_nodes::GpuBufferPublisherHelper> helper_;
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
