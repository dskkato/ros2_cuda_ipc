#include <chrono>
#include <cstring>
#include <rclcpp/rclcpp.hpp>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
#include "ros2_cuda_ipc_core/gpu_buffer_mapper.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_buffer.hpp"
#include "ros2_cuda_ipc_msgs/srv/gpu_buffer_release.hpp"

class DummySubscriber : public rclcpp::Node {
 public:
  DummySubscriber() : Node("dummy_gpu_buffer_subscriber") {
    release_client_ =
        this->create_client<ros2_cuda_ipc_msgs::srv::GpuBufferRelease>(
            "gpu_buffer_release");
    // Create a non-blocking CUDA stream if available
    if (ros2_cuda_ipc_core::cuda_is_available()) {
      stream_ = ros2_cuda_ipc_core::cuda_stream_create();
    }
    sub_ = this->create_subscription<ros2_cuda_ipc_msgs::msg::GpuBuffer>(
        "gpu_buffer", 10,
        [this](const ros2_cuda_ipc_msgs::msg::GpuBuffer& msg) {
          RCLCPP_INFO(this->get_logger(), "Received seq_id %lu", msg.seq_id);
          // Map memory once per slot and reuse
          if (msg.plane_count > 0 && msg.planes.size() >= 1) {
            ros2_cuda_ipc_core::CudaIpcMemHandle h{};
            std::memcpy(&h, msg.planes[0].ipc_mem_handle.data(), sizeof(h));
            (void)mapper_.open_memory(msg.pool_slot_id, h);
          }
          // Open or reuse the event, then wait on our stream
          ros2_cuda_ipc_core::CudaIpcEventHandle eh{};
          std::memcpy(&eh, msg.ipc_event_handle.data(), sizeof(eh));
          (void)mapper_.open_event(msg.pool_slot_id, eh);
          auto t0 = std::chrono::steady_clock::now();
          if (stream_) {
            (void)mapper_.wait_ready(msg.pool_slot_id, stream_);
            (void)ros2_cuda_ipc_core::cuda_stream_synchronize(stream_);
          }
          auto t1 = std::chrono::steady_clock::now();
          auto ms =
              std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
                  .count();
          RCLCPP_INFO(this->get_logger(), "Event waited ~%ld ms",
                      static_cast<long>(ms));

          // Notify release via service
          if (!release_client_->service_is_ready()) {
            // Try a non-blocking wait once
            (void)release_client_->wait_for_service(
                std::chrono::milliseconds(10));
          }
          auto req = std::make_shared<
              ros2_cuda_ipc_msgs::srv::GpuBufferRelease::Request>();
          req->seq_id = msg.seq_id;
          req->pool_slot_id = msg.pool_slot_id;
          req->consumer_id = this->get_fully_qualified_name();
          (void)release_client_->async_send_request(req);
        });
  }

 private:
  rclcpp::Subscription<ros2_cuda_ipc_msgs::msg::GpuBuffer>::SharedPtr sub_;
  rclcpp::Client<ros2_cuda_ipc_msgs::srv::GpuBufferRelease>::SharedPtr
      release_client_;
  void* stream_{nullptr};
  ros2_cuda_ipc_core::GpuBufferMapper mapper_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummySubscriber>());
  rclcpp::shutdown();
  return 0;
}
