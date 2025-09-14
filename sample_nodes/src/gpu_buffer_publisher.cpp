#include <unistd.h>

#include <chrono>
#include <cstring>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <unordered_set>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
#include "ros2_cuda_ipc_core/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_core/shm_release.hpp"
#include "ros2_cuda_ipc_core/version.hpp"
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
  }

  ~DummyPublisher() {
    if (producer_stream_) {
      (void)ros2_cuda_ipc_core::cuda_stream_destroy(producer_stream_);
      producer_stream_ = nullptr;
    }
    // Cleanup slot-fixed SHM objects we created
    for (const auto& name : known_shms_) {
      (void)ros2_cuda_ipc_core::shm_unlink(name);
    }
  }

 private:
  void publish_once() {
    // Enforce lease timeout to avoid deadlock when subscribers never release
    const auto now = std::chrono::steady_clock::now();
    for (auto it = leases_.begin(); it != leases_.end();) {
      // If using SHM release and refcnt reached 0, free immediately
      if (!it->second.shm_name.empty()) {
        auto ref = ros2_cuda_ipc_core::shm_read_refcnt(
            it->second.shm_name, it->first, it->second.seq);
        if (ref.has_value() && *ref <= 0) {
          (void)pool_->release(it->first);
          RCLCPP_INFO(this->get_logger(),
                      "Slot %u freed for seq %lu via SHM (all consumers done)",
                      it->first, it->second.seq);
          it = leases_.erase(it);
          continue;
        }
      }
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
        // Fill the borrowed device buffer with a pattern on producer stream
        void* dptr = pool_->device_ptr(*slot);
        const uint64_t size_bytes = static_cast<uint64_t>(msg.width) *
                                    static_cast<uint64_t>(msg.height) *
                                    static_cast<uint64_t>(msg.channels);
        // Use a simple changing pattern so frames differ (low byte of seq)
        unsigned char pattern = static_cast<unsigned char>(count_ & 0xFF);
        (void)sample_nodes::cuda_fill_u8(dptr, pattern, size_bytes,
                                         producer_stream_);
        // Fill plane 0 with size/pitch and copy raw 64B handle
        msg.plane_count = 1;
        msg.planes.resize(1);
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
          lease.since = std::chrono::steady_clock::now();
          // Create SHM control block and pass name to message
          auto name = ros2_cuda_ipc_core::make_slot_shm_name_with_owner(
              shm_owner_, static_cast<uint32_t>(*slot));
          auto created = ros2_cuda_ipc_core::shm_create_init(
              name, static_cast<uint32_t>(*slot), msg.seq_id, expected_count);
          if (created) {
            msg.shm_name = *created;
            lease.shm_name = *created;
            known_shms_.insert(*created);
            RCLCPP_INFO(this->get_logger(), "Using SHM release: %s (expect %d)",
                        lease.shm_name.c_str(), expected_count);
          } else {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to create SHM control block; release unlikely "
                        "to complete");
            msg.shm_name.clear();
          }
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
  struct Lease {
    uint64_t seq{0};
    int expected{0};
    std::chrono::steady_clock::time_point since{};
    std::string shm_name;
  };
  std::unordered_map<uint32_t, Lease> leases_;
  int lease_timeout_ms_{3000};
  int expected_consumers_{-1};
  uint64_t count_{0};
  void* producer_stream_{nullptr};
  std::unordered_set<std::string> known_shms_;
  std::string shm_owner_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyPublisher>());
  rclcpp::shutdown();
  return 0;
}
