#include <cstring>
#include <rclcpp/rclcpp.hpp>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
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
          if (msg.plane_count > 0 && msg.planes.size() >= 1) {
            // Reconstruct CUDA IPC handle and try open/close via core helpers
            ros2_cuda_ipc_core::CudaIpcMemHandle h{};
            static_assert(sizeof(h) == 64, "Unexpected handle size");
            std::memcpy(&h, msg.planes[0].ipc_mem_handle.data(), sizeof(h));
            void* dev_ptr = ros2_cuda_ipc_core::cuda_ipc_open_mem_handle(h);
            if (dev_ptr) {
              RCLCPP_INFO(this->get_logger(), "Opened CUDA IPC mem @%p",
                          dev_ptr);
              bool closed =
                  ros2_cuda_ipc_core::cuda_ipc_close_mem_handle(dev_ptr);
              (void)closed;
            } else {
              RCLCPP_WARN(
                  this->get_logger(),
                  "Failed to open CUDA IPC mem (CUDA disabled or no device?)");
            }
          }
          // If event handle is present, open it and wait on our stream
          {
            ros2_cuda_ipc_core::CudaIpcEventHandle eh{};
            static_assert(sizeof(eh) == 64, "Unexpected event handle size");
            std::memcpy(&eh, msg.ipc_event_handle.data(), sizeof(eh));
            void* evt = ros2_cuda_ipc_core::cuda_ipc_open_event_handle(eh);
            if (evt) {
              if (stream_) {
                bool ok =
                    ros2_cuda_ipc_core::cuda_stream_wait_event(stream_, evt);
                RCLCPP_INFO(this->get_logger(), "StreamWaitEvent: %s",
                            ok ? "ok" : "fail");
                (void)ros2_cuda_ipc_core::cuda_stream_synchronize(stream_);
              } else {
                bool ready = ros2_cuda_ipc_core::cuda_event_query(evt);
                RCLCPP_INFO(this->get_logger(), "Event query: %s",
                            ready ? "ready" : "not ready");
              }
              (void)ros2_cuda_ipc_core::cuda_event_destroy(evt);
            }
          }

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
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummySubscriber>());
  rclcpp::shutdown();
  return 0;
}
