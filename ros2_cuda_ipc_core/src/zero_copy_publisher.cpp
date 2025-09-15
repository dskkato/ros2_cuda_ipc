#include "ros2_cuda_ipc_core/zero_copy_publisher.hpp"

#include <unistd.h>

#include <chrono>
#include <cstring>

#include "ros2_cuda_ipc_core/version.hpp"

namespace ros2_cuda_ipc_core {

ZeroCopyPublisher::ZeroCopyPublisher(rclcpp::Node& node,
                                     const std::string& topic,
                                     const PoolOptions& pool_opts,
                                     int lease_timeout_ms)
    : pub_(
          node.create_publisher<ros2_cuda_ipc_msgs::msg::GpuBuffer>(topic, 10)),
      pool_(pool_opts),
      lease_mgr_(pool_, lease_timeout_ms) {
  if (pool_opts.producer_stream) {
    stream_ = pool_opts.producer_stream;
    owns_stream_ = false;
  } else {
    stream_ = cuda_stream_create();
    owns_stream_ = (stream_ != nullptr);
  }
  // Create unique SHM owner name based on node FQN, time and pid
  auto base = sanitize_shm_owner(node.get_fully_qualified_name());
  const auto now = std::chrono::system_clock::now();
  const auto secs =
      std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch())
          .count();
  char buf[96];
  std::snprintf(buf, sizeof(buf), "%s_%lld_%d", base.c_str(),
                static_cast<long long>(secs), static_cast<int>(::getpid()));
  shm_owner_ = sanitize_shm_owner(buf);
  lease_mgr_.set_owner(shm_owner_);
}

ZeroCopyPublisher::~ZeroCopyPublisher() {
  if (owns_stream_ && stream_) {
    (void)cuda_stream_destroy(stream_);
    stream_ = nullptr;
  }
  lease_mgr_.cleanup();
}

bool ZeroCopyPublisher::publish(ros2_cuda_ipc_msgs::msg::GpuBuffer& msg,
                                int expected_consumers,
                                const FillCallback& fill_cb) {
  lease_mgr_.tick();
  auto slot = pool_.borrow();
  if (!slot.has_value()) {
    msg.plane_count = 0;
    pub_->publish(msg);
    return false;
  }
  const uint32_t slot_id = static_cast<uint32_t>(*slot);
  void* device_ptr = pool_.device_ptr(*slot);
  const uint64_t size_bytes =
      static_cast<uint64_t>(msg.width) * msg.height * msg.channels;
  if (fill_cb) {
    fill_cb(device_ptr, size_bytes, stream_);
  }

  msg.abi_version = kAbiVersion;
  auto id = cuda_get_device_id_string();
  msg.device_uuid = id.empty() ? std::string("unknown") : id;
  msg.pool_slot_id = slot_id;
  msg.shm_name.clear();

  CudaIpcMemHandle mh{};
  bool have_mem = pool_.ipc_handle(slot_id, mh);
  if (!have_mem) {
    msg.plane_count = 0;
    (void)pool_.release(slot_id);
    pub_->publish(msg);
    return false;
  }
  msg.plane_count = 1;
  msg.planes.resize(1);
  msg.planes[0].size_bytes = size_bytes;
  msg.planes[0].pitch_bytes = static_cast<uint64_t>(msg.width) * msg.channels;
  std::memcpy(msg.planes[0].ipc_mem_handle.data(), &mh, sizeof(mh));

  CudaIpcEventHandle eh{};
  if (pool_.ipc_event_handle(slot_id, eh)) {
    std::memcpy(msg.ipc_event_handle.data(), &eh, sizeof(eh));
    if (stream_) {
      (void)pool_.record_ready_on_stream(slot_id, stream_);
    } else {
      (void)pool_.record_ready(slot_id);
    }
  }

  int consumers = expected_consumers;
  if (consumers < 0) {
    consumers = static_cast<int>(pub_->get_subscription_count());
  }

  if (consumers > 0) {
    auto created = lease_mgr_.start_lease(slot_id, msg.seq_id, consumers);
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
