#include <chrono>
#include <cstring>
#include <rclcpp/rclcpp.hpp>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
#include "ros2_cuda_ipc_core/gpu_buffer_mapper.hpp"
#include "ros2_cuda_ipc_core/scoped_mapped_frame.hpp"
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
          auto t0 = std::chrono::steady_clock::now();
          // RAII frame: opens event+mem, waits on stream, decrements SHM on
          // dtor
          ros2_cuda_ipc_core::CudaIpcMemHandle* mem_ptr = nullptr;
          ros2_cuda_ipc_core::CudaIpcMemHandle mem_tmp{};
          if (msg.plane_count > 0 && msg.planes.size() >= 1) {
            std::memcpy(&mem_tmp, msg.planes[0].ipc_mem_handle.data(),
                        sizeof(mem_tmp));
            mem_ptr = &mem_tmp;
          }
          ros2_cuda_ipc_core::CudaIpcEventHandle evt_tmp{};
          std::memcpy(&evt_tmp, msg.ipc_event_handle.data(), sizeof(evt_tmp));
          {
            ros2_cuda_ipc_core::ScopedMappedFrame frame(
                mapper_, msg.pool_slot_id, mem_ptr, &evt_tmp, stream_,
                msg.shm_name, msg.seq_id, /*sync_on_dtor=*/true);
            // Here we could do GPU work on stream_ using frame.device_ptr().
          }
          auto t1 = std::chrono::steady_clock::now();
          auto ms =
              std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
                  .count();
          RCLCPP_INFO(this->get_logger(), "Event waited ~%ld ms",
                      static_cast<long>(ms));
        });
  }

 private:
  rclcpp::Subscription<ros2_cuda_ipc_msgs::msg::GpuBuffer>::SharedPtr sub_;
  ros2_cuda_ipc_core::cudaStream_t stream_{nullptr};
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
