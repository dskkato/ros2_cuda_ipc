#include <chrono>
#include <cstring>
#include <rclcpp/rclcpp.hpp>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
#include "ros2_cuda_ipc_core/gpu_buffer_mapper.hpp"
#include "ros2_cuda_ipc_core/shm_release.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_buffer.hpp"

class DummySubscriber : public rclcpp::Node {
 public:
  DummySubscriber() : Node("dummy_gpu_buffer_subscriber") {
    // Create a non-blocking CUDA stream if available
    if (ros2_cuda_ipc_core::cuda_is_available()) {
      stream_ = ros2_cuda_ipc_core::cuda_stream_create();
    }

    sub_ = this->create_subscription<ros2_cuda_ipc_msgs::msg::GpuBuffer>(
        "gpu_buffer", 10,
        [this](const ros2_cuda_ipc_msgs::msg::GpuBuffer& msg) {
          // Detect publisher restart or device mismatch and reset mapper
          if (prev_abi_version_ != 0 && prev_abi_version_ != msg.abi_version) {
            RCLCPP_WARN(
                this->get_logger(),
                "ABI version changed: %u -> %u. Resetting mapper cache.",
                prev_abi_version_, msg.abi_version);
            mapper_.reset();
          }
          if (!prev_device_id_.empty() && prev_device_id_ != msg.device_uuid) {
            RCLCPP_WARN(this->get_logger(),
                        "Device id changed: %s -> %s. Resetting mapper cache.",
                        prev_device_id_.c_str(), msg.device_uuid.c_str());
            mapper_.reset();
          }
          prev_abi_version_ = msg.abi_version;
          prev_device_id_ = msg.device_uuid;
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

          // Release via SHM decrement when a valid plane was provided
          if (msg.plane_count > 0) {
            if (!msg.shm_name.empty()) {
              auto res = ros2_cuda_ipc_core::shm_decrement(
                  msg.shm_name, msg.pool_slot_id, msg.seq_id);
              if (res.has_value()) {
                RCLCPP_INFO(this->get_logger(), "SHM release %s: new refcnt=%d",
                            msg.shm_name.c_str(), *res);
              } else {
                RCLCPP_WARN(this->get_logger(), "SHM release failed for %s",
                            msg.shm_name.c_str());
              }
            } else {
              RCLCPP_WARN(this->get_logger(),
                          "No shm_name provided; cannot release via SHM");
            }
          }
        });
  }

 private:
  rclcpp::Subscription<ros2_cuda_ipc_msgs::msg::GpuBuffer>::SharedPtr sub_;
  void* stream_{nullptr};
  ros2_cuda_ipc_core::GpuBufferMapper mapper_;
  uint32_t prev_abi_version_{0};
  std::string prev_device_id_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummySubscriber>());
  rclcpp::shutdown();
  return 0;
}
