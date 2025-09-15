#include "ros2_cuda_ipc_core/zero_copy_publisher.hpp"

#include <unistd.h>

#include <chrono>
#include <cstring>

#include "ros2_cuda_ipc_core/cuda_support.hpp"
#include "ros2_cuda_ipc_core/shm_release.hpp"
#include "ros2_cuda_ipc_core/version.hpp"

namespace ros2_cuda_ipc_core {

ZeroCopyPublisher::ZeroCopyPublisher(rclcpp::Node& node,
                                     const std::string& topic, Options options)
    : pub_(
          node.create_publisher<ros2_cuda_ipc_msgs::msg::GpuBuffer>(topic, 10)),
      clock_(node.get_clock()),
      pool_(options.pool_options) {
  if (cuda_is_available()) {
    producer_stream_ = cuda_stream_create();
  }
  if (!options.shm_owner.empty()) {
    shm_owner_ = sanitize_shm_owner(options.shm_owner);
  } else {
    auto base = sanitize_shm_owner(node.get_fully_qualified_name());
    const auto now = std::chrono::system_clock::now();
    const auto secs =
        std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch())
            .count();
    char buf[96];
    std::snprintf(buf, sizeof(buf), "%s_%lld_%d", base.c_str(),
                  static_cast<long long>(secs), static_cast<int>(::getpid()));
    shm_owner_ = sanitize_shm_owner(buf);
  }
  lease_mgr_ = std::make_unique<LeaseManager>(pool_, options.lease_timeout_ms);
  lease_mgr_->set_owner(shm_owner_);
}

ZeroCopyPublisher::~ZeroCopyPublisher() {
  if (producer_stream_) {
    (void)cuda_stream_destroy(producer_stream_);
    producer_stream_ = nullptr;
  }
  if (lease_mgr_) {
    lease_mgr_->cleanup();
  }
}

bool ZeroCopyPublisher::publish(ros2_cuda_ipc_msgs::msg::GpuBuffer& msg,
                                FillCallback fill_cb, int expected_consumers) {
  if (lease_mgr_) {
    (void)lease_mgr_->tick();
  }
  msg.seq_id = seq_++;
  msg.stamp = clock_->now();

  auto slot = pool_.borrow(true);
  if (!slot.has_value()) {
    msg.plane_count = 0;
    pub_->publish(msg);
    return false;
  }
  uint32_t slot_id = static_cast<uint32_t>(*slot);
  void* device_ptr = pool_.device_ptr(slot_id);
  const uint64_t size_bytes =
      static_cast<uint64_t>(msg.width) * msg.height * msg.channels;
  if (fill_cb) {
    fill_cb(device_ptr, size_bytes, producer_stream_);
  }

  msg.abi_version = kAbiVersion;
  auto id = cuda_get_device_id_string();
  msg.device_uuid = id.empty() ? std::string("unknown") : id;
  msg.pool_slot_id = slot_id;
  msg.plane_count = 1;
  msg.planes.resize(1);
  msg.planes[0].size_bytes = size_bytes;
  msg.planes[0].pitch_bytes = static_cast<uint64_t>(msg.width) * msg.channels;

  CudaIpcMemHandle mh{};
  bool have_mem = pool_.ipc_handle(slot_id, mh);
  if (!have_mem) {
    msg.plane_count = 0;
    (void)pool_.release(slot_id);
    pub_->publish(msg);
    return false;
  }
  std::memcpy(msg.planes[0].ipc_mem_handle.data(), &mh, sizeof(mh));

  CudaIpcEventHandle eh{};
  if (pool_.ipc_event_handle(slot_id, eh)) {
    std::memcpy(msg.ipc_event_handle.data(), &eh, sizeof(eh));
    if (producer_stream_) {
      (void)pool_.record_ready_on_stream(slot_id, producer_stream_);
    } else {
      (void)pool_.record_ready(slot_id);
    }
  }

  if (expected_consumers < 0) {
    expected_consumers = static_cast<int>(pub_->get_subscription_count());
  }
  if (expected_consumers > 0 && lease_mgr_) {
    auto created =
        lease_mgr_->start_lease(slot_id, msg.seq_id, expected_consumers);
    if (created) {
      msg.shm_name = *created;
    }
  } else {
    (void)pool_.release(slot_id);
  }

  pub_->publish(msg);
  return true;
}

}  // namespace ros2_cuda_ipc_core
