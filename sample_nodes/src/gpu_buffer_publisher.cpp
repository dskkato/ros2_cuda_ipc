#include <chrono>
#include <cstring>
#include <optional>
#include <rclcpp/rclcpp.hpp>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
#include "ros2_cuda_ipc_core/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_buffer.hpp"
#include "ros2_cuda_ipc_msgs/srv/gpu_buffer_release.hpp"
#include "sample_nodes/sample_cuda_utils.hpp"

class DummyPublisher : public rclcpp::Node {
 public:
  DummyPublisher() : Node("dummy_gpu_buffer_publisher") {
    using namespace std::chrono_literals;
    pub_ = this->create_publisher<ros2_cuda_ipc_msgs::msg::GpuBuffer>(
        "gpu_buffer", 10);
    // Prepare a tiny pool (1 slot, 4 MiB) and try enabling CUDA using
    // PoolOptions.
    ros2_cuda_ipc_core::PoolOptions opts;
    opts.pool_size = 1;
    opts.bytes_per_slot = 4u * 1024u * 1024u;
    opts.use_cuda = true;
    opts.events_enabled = true;
    if (ros2_cuda_ipc_core::cuda_is_available()) {
      producer_stream_ = ros2_cuda_ipc_core::cuda_stream_create();
      opts.producer_stream = producer_stream_;
    }
    pool_ = std::make_unique<ros2_cuda_ipc_core::GpuBufferPool>(opts);

    timer_ = this->create_wall_timer(1s, [this]() { publish_once(); });

    // Parameter for lease timeout (ms)
    lease_timeout_ms_ = this->declare_parameter<int>("lease_timeout_ms", 3000);

    // Service server to accept release notifications
    release_srv_ = this->create_service<
        ros2_cuda_ipc_msgs::srv::GpuBufferRelease>(
        "gpu_buffer_release",
        [this](
            const std::shared_ptr<
                ros2_cuda_ipc_msgs::srv::GpuBufferRelease::Request>
                req,
            std::shared_ptr<ros2_cuda_ipc_msgs::srv::GpuBufferRelease::Response>
                resp) {
          if (!held_slot_) {
            resp->ok = false;
            RCLCPP_WARN(this->get_logger(),
                        "Release requested but no slot held");
            return;
          }
          if (req->pool_slot_id == *held_slot_ && req->seq_id == held_seq_) {
            bool ok = pool_->release(*held_slot_);
            resp->ok = ok;
            if (ok) {
              RCLCPP_INFO(
                  this->get_logger(), "Released slot %u for seq %lu from %s",
                  req->pool_slot_id, req->seq_id, req->consumer_id.c_str());
              held_slot_.reset();
              held_since_ = {};
            } else {
              RCLCPP_WARN(this->get_logger(), "Release failed for slot %u",
                          req->pool_slot_id);
            }
          } else {
            resp->ok = false;
            RCLCPP_WARN(this->get_logger(),
                        "Release mismatch: req(slot=%u,seq=%lu) vs "
                        "held(slot=%zu,seq=%lu)",
                        req->pool_slot_id, req->seq_id,
                        held_slot_.value_or(static_cast<size_t>(-1)),
                        held_seq_);
          }
        });
  }

  ~DummyPublisher() {
    if (producer_stream_) {
      (void)ros2_cuda_ipc_core::cuda_stream_destroy(producer_stream_);
      producer_stream_ = nullptr;
    }
  }

 private:
  void publish_once() {
    if (held_slot_) {
      // Enforce lease timeout to avoid deadlock when subscriber never releases
      const auto now = std::chrono::steady_clock::now();
      if (held_since_.time_since_epoch().count() > 0) {
        const auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                                  held_since_);
        if (elapsed.count() > lease_timeout_ms_) {
          RCLCPP_WARN(this->get_logger(),
                      "Lease timeout (%d ms) exceeded for slot %zu (seq %lu). "
                      "Forcing release.",
                      lease_timeout_ms_, *held_slot_, held_seq_);
          (void)pool_->release(*held_slot_);
          held_slot_.reset();
          held_since_ = {};
        }
      }
      return;
    }
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
        // Simulate GPU work on producer stream to visualize event sync
        (void)sample_nodes::cuda_simulate_work_ms(50, producer_stream_);
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
        msg.pool_slot_id = static_cast<uint32_t>(*slot);
        held_slot_ = slot;
        held_seq_ = msg.seq_id;
        held_since_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(),
                    "Publishing seq=%lu with CUDA IPC mem handle (slot %zu)",
                    msg.seq_id, *slot);
      } else {
        // CUDA disabled or unavailable: publish without plane data
        msg.plane_count = 0;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "CUDA IPC handle unavailable. Did you build core "
                             "with CUDA and have a device?");
      }
      // Keep slot borrowed until release service is called.
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
  rclcpp::Service<ros2_cuda_ipc_msgs::srv::GpuBufferRelease>::SharedPtr
      release_srv_;
  std::optional<std::size_t> held_slot_;
  uint64_t held_seq_{0};
  std::chrono::steady_clock::time_point held_since_{};
  int lease_timeout_ms_{3000};
  uint64_t count_{0};
  void* producer_stream_{nullptr};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyPublisher>());
  rclcpp::shutdown();
  return 0;
}
