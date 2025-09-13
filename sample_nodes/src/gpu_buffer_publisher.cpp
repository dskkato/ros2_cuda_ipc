#include <chrono>
#include <cstring>
#include <rclcpp/rclcpp.hpp>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
#include "ros2_cuda_ipc_core/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_buffer.hpp"

class DummyPublisher : public rclcpp::Node {
 public:
  DummyPublisher() : Node("dummy_gpu_buffer_publisher") {
    using namespace std::chrono_literals;
    pub_ = this->create_publisher<ros2_cuda_ipc_msgs::msg::GpuBuffer>(
        "gpu_buffer", 10);
    // Prepare a tiny pool (1 slot, 4 MiB) and try enabling CUDA.
    pool_ = std::make_unique<ros2_cuda_ipc_core::GpuBufferPool>(
        1 /*slots*/, 4u * 1024u * 1024u /*bytes/slot*/, true /*use_cuda*/);

    timer_ = this->create_wall_timer(1s, [this]() { publish_once(); });
  }

 private:
  void publish_once() {
    ros2_cuda_ipc_msgs::msg::GpuBuffer msg;
    msg.abi_version = 1;
    msg.device_uuid = "dummy";  // TODO: obtain real device UUID
    msg.seq_id = count_++;
    msg.pool_slot_id = 0;
    msg.format = 1;   // e.g., BGR8 (tentative)
    msg.layout = 0;   // LINEAR
    msg.width = 640;  // demo values
    msg.height = 480;
    msg.channels = 3;
    msg.stamp = this->now();
    msg.frame_id = "frame";
    msg.shm_name = "";

    // Try to borrow a slot and export an IPC memory handle for plane[0].
    const auto slot = pool_->borrow();
    if (slot.has_value()) {
      ros2_cuda_ipc_core::CudaIpcMemHandle h{};
      bool has_cuda_mem = pool_->ipc_handle(*slot, h);
      if (has_cuda_mem) {
        // Fill plane 0 with size/pitch and copy raw 64B handle
        msg.plane_count = 1;
        msg.planes.resize(1);
        const uint64_t size_bytes = static_cast<uint64_t>(msg.width) *
                                    static_cast<uint64_t>(msg.height) *
                                    static_cast<uint64_t>(msg.channels);
        msg.planes[0].size_bytes = size_bytes;
        msg.planes[0].pitch_bytes = static_cast<uint64_t>(msg.width) *
                                    static_cast<uint64_t>(msg.channels);
        std::memcpy(msg.planes[0].ipc_mem_handle.data(), &h, sizeof(h));
        // Export and embed event handle, and record event as ready
        ros2_cuda_ipc_core::CudaIpcEventHandle eh{};
        if (pool_->ipc_event_handle(*slot, eh)) {
          std::memcpy(msg.ipc_event_handle.data(), &eh, sizeof(eh));
          (void)pool_->record_ready(*slot);
        }
        RCLCPP_INFO(this->get_logger(),
                    "Publishing seq=%lu with CUDA IPC mem handle", msg.seq_id);
      } else {
        // CUDA disabled or unavailable: publish without plane data
        msg.plane_count = 0;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "CUDA IPC handle unavailable. Did you build core "
                             "with CUDA and have a device?");
      }
      // For now, immediately release so we don't exhaust the single slot.
      pool_->release(*slot);
    } else {
      // Pool exhausted; skip planes for this demo message
      msg.plane_count = 0;
      RCLCPP_WARN(this->get_logger(),
                  "Pool exhausted; publishing metadata only");
    }

    pub_->publish(msg);
  }

  rclcpp::Publisher<ros2_cuda_ipc_msgs::msg::GpuBuffer>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<ros2_cuda_ipc_core::GpuBufferPool> pool_;
  uint64_t count_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyPublisher>());
  rclcpp::shutdown();
  return 0;
}
