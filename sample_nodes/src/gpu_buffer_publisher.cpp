#include <chrono>
#include <cstring>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <unordered_set>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
#include "ros2_cuda_ipc_core/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_core/version.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_buffer.hpp"
#include "ros2_cuda_ipc_msgs/srv/gpu_buffer_release.hpp"
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
          auto it = leases_.find(req->pool_slot_id);
          if (it == leases_.end()) {
            resp->ok = false;
            RCLCPP_WARN(this->get_logger(),
                        "Release for unknown slot %u (seq=%lu) from %s",
                        req->pool_slot_id, req->seq_id,
                        req->consumer_id.c_str());
            return;
          }
          auto& lease = it->second;
          if (lease.seq != req->seq_id) {
            resp->ok = false;
            RCLCPP_WARN(this->get_logger(),
                        "Release seq mismatch for slot %u: got %lu, expect %lu",
                        req->pool_slot_id, req->seq_id, lease.seq);
            return;
          }
          // Deduplicate by consumer_id
          if (!lease.consumers.insert(req->consumer_id).second) {
            // Already released by this consumer
            resp->ok = true;
            RCLCPP_DEBUG(
                this->get_logger(),
                "Duplicate release ignored: slot %u seq %lu consumer %s",
                req->pool_slot_id, req->seq_id, req->consumer_id.c_str());
            return;
          }
          if (lease.remaining > 0) {
            lease.remaining -= 1;
          }
          RCLCPP_INFO(this->get_logger(),
                      "Release ack: slot %u seq %lu remaining %d/%d by %s",
                      req->pool_slot_id, req->seq_id, lease.remaining,
                      lease.expected, req->consumer_id.c_str());
          if (lease.remaining == 0) {
            bool ok = pool_->release(req->pool_slot_id);
            resp->ok = ok;
            if (ok) {
              RCLCPP_INFO(this->get_logger(),
                          "Slot %u freed for seq %lu (all consumers done)",
                          req->pool_slot_id, req->seq_id);
            } else {
              RCLCPP_WARN(this->get_logger(),
                          "Pool release failed for slot %u (seq %lu)",
                          req->pool_slot_id, req->seq_id);
            }
            leases_.erase(it);
          } else {
            resp->ok = true;
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
    // Enforce lease timeout to avoid deadlock when subscribers never release
    const auto now = std::chrono::steady_clock::now();
    for (auto it = leases_.begin(); it != leases_.end();) {
      const auto elapsed =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              now - it->second.since);
      if (elapsed.count() > lease_timeout_ms_) {
        RCLCPP_WARN(this->get_logger(),
                    "Lease timeout (%d ms) exceeded for slot %u (seq %lu). "
                    "Forcing release.",
                    lease_timeout_ms_, it->first, it->second.seq);
        (void)pool_->release(it->first);
        it = leases_.erase(it);
        continue;
      }
      ++it;
    }
    ros2_cuda_ipc_msgs::msg::GpuBuffer msg;
    msg.abi_version = ros2_cuda_ipc_core::kAbiVersion;
    {
      auto id = ros2_cuda_ipc_core::cuda_get_device_id_string();
      msg.device_uuid = id.empty() ? std::string("unknown") : id;
    }
    msg.seq_id = count_++;
    msg.pool_slot_id = 0;
    msg.format = ros2_cuda_ipc_msgs::msg::GpuBuffer::FORMAT_BGR8;
    msg.layout = ros2_cuda_ipc_msgs::msg::GpuBuffer::LAYOUT_LINEAR;
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
        // Choose expected consumers from parameter or ROS graph
        int expected_count = expected_consumers_;
        if (expected_count < 0) {
          expected_count = static_cast<int>(pub_->get_subscription_count());
        }
        if (expected_count > 0) {
          // Initialize lease tracking for this slot/frame
          Lease lease;
          lease.seq = msg.seq_id;
          lease.expected = expected_count;
          lease.remaining = expected_count;
          lease.since = std::chrono::steady_clock::now();
          leases_[msg.pool_slot_id] = std::move(lease);
        } else {
          // No subscribers; free immediately without creating a lease entry
          (void)pool_->release(*slot);
          RCLCPP_DEBUG(this->get_logger(),
                       "No subscribers; immediately released slot %zu", *slot);
        }
        RCLCPP_INFO(this->get_logger(),
                    "Publishing seq=%lu with CUDA IPC mem handle (slot %zu), "
                    "expected consumers=%d",
                    msg.seq_id, *slot, expected_count);
      } else {
        // CUDA disabled or unavailable: publish without plane data
        msg.plane_count = 0;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "CUDA IPC handle unavailable. Did you build core "
                             "with CUDA and have a device?");
      }
      // Keep slot borrowed until all releases are received or timeout.
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
  struct Lease {
    uint64_t seq{0};
    int expected{0};
    int remaining{0};
    std::chrono::steady_clock::time_point since{};
    std::unordered_set<std::string> consumers;
  };
  std::unordered_map<uint32_t, Lease> leases_;
  int lease_timeout_ms_{3000};
  int expected_consumers_{-1};
  uint64_t count_{0};
  void* producer_stream_{nullptr};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyPublisher>());
  rclcpp::shutdown();
  return 0;
}
