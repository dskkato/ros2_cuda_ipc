// Convenience publisher wrapper that encapsulates GpuBufferPool + LeaseManager
// and exposes a one-shot API to borrow/fill/export/publish/release.
#ifndef ROS2_CUDA_IPC_CORE_ZERO_COPY_PUBLISHER_HPP_
#define ROS2_CUDA_IPC_CORE_ZERO_COPY_PUBLISHER_HPP_

#include <cstdint>
#include <cstring>
#include <functional>
#include <string>

#include "ros2_cuda_ipc_core/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_core/lease_manager.hpp"
#include "ros2_cuda_ipc_core/version.hpp"
#include "ros2_cuda_ipc_msgs/msg/gpu_buffer.hpp"

namespace ros2_cuda_ipc_core {

// ZeroCopyPublisher owns a pool and a lease manager and produces
// ros2_cuda_ipc_msgs::msg::GpuBuffer ready for publishing.
class ZeroCopyPublisher {
 public:
  // Construct with pool options and lease timeout. The SHM owner string will
  // be sanitized and used for SHM names.
  ZeroCopyPublisher(const PoolOptions& opts, int lease_timeout_ms,
                    std::string shm_owner);
  ~ZeroCopyPublisher();

  // Update SHM owner string.
  void set_owner(std::string owner);
  // Update lease timeout in milliseconds.
  void set_timeout_ms(int ms);
  // Run a maintenance tick to auto-release timed-out leases; returns released
  // count.
  int tick();
  // Best-effort cleanup of known SHMs.
  void cleanup();

  // One-shot produce-and-publish API. The provided `pub` must have a
  // `publish(const ros2_cuda_ipc_msgs::msg::GpuBuffer&)` method.
  // - `msg` should have seq_id, layout, format, width, height, channels,
  //    stamp, and frame_id set. Other fields are filled here.
  // - `expected_consumers`: -1 for auto (caller should pass actual), 0 to
  //    release immediately after publish, >0 to start SHM lease.
  // - `fn`: called iff a slot is borrowed; signature
  //      void(void* device_ptr, uint32_t width, uint32_t height,
  //           uint32_t channels, cudaStream_t stream)
  //   It can enqueue work on the producer stream and may ignore dims.
  // - `blocking`: if true, waits for a free slot; otherwise publishes metadata
  //   only when pool is exhausted.
  template <typename PublisherT, typename Fn>
  bool produce_and_publish(PublisherT& pub,
                           ros2_cuda_ipc_msgs::msg::GpuBuffer& msg,
                           int expected_consumers, Fn fn,
                           bool blocking = true) {
    // Maintain leases first
    (void)tick();

    msg.abi_version = kAbiVersion;
    auto id = cuda_get_device_id_string();
    msg.device_uuid = id.empty() ? std::string("unknown") : id;
    msg.pool_slot_id = 0;
    msg.shm_name.clear();
    msg.plane_count = 0;
    msg.planes.clear();

    const uint64_t pitch_bytes =
        static_cast<uint64_t>(msg.width) * msg.channels;
    const uint64_t size_bytes =
        static_cast<uint64_t>(msg.width) * msg.height * msg.channels;

    auto slot = pool_.borrow(blocking);
    if (!slot.has_value()) {
      // No slot available: publish metadata only
      pub.publish(msg);
      return false;
    }

    const uint32_t slot_id = static_cast<uint32_t>(*slot);
    void* dev_ptr = pool_.device_ptr(*slot);
    msg.pool_slot_id = slot_id;

    if (dev_ptr && size_bytes > 0) {
      // Let the user fill the buffer
      fn(dev_ptr, msg.width, msg.height, msg.channels, producer_stream_);
    }

    // Export CUDA IPC handles
    CudaIpcMemHandle mh{};
    bool have_mem = pool_.ipc_handle(slot_id, mh);
    if (!have_mem) {
      (void)pool_.release(slot_id);
      pub.publish(msg);  // metadata only
      return false;
    }

    msg.plane_count = 1;
    msg.planes.resize(1);
    msg.planes[0].size_bytes = size_bytes;
    msg.planes[0].pitch_bytes = pitch_bytes;
    std::memcpy(msg.planes[0].ipc_mem_handle.data(), &mh, sizeof(mh));

    // Event handle (if enabled in pool)
    CudaIpcEventHandle eh{};
    if (pool_.ipc_event_handle(slot_id, eh)) {
      std::memcpy(msg.ipc_event_handle.data(), &eh, sizeof(eh));
      if (producer_stream_) {
        (void)pool_.record_ready_on_stream(slot_id, producer_stream_);
      } else {
        (void)pool_.record_ready(slot_id);
      }
    }

    // Leasing logic
    if (expected_consumers > 0) {
      lease_mgr_.set_owner(owner_);
      auto created =
          lease_mgr_.start_lease(slot_id, msg.seq_id, expected_consumers);
      if (created) {
        msg.shm_name = *created;
      }
    } else {
      (void)pool_.release(slot_id);
    }

    pub.publish(msg);
    return true;
  }

  // Convenience overload: no-op filler.
  template <typename PublisherT>
  bool produce_and_publish(PublisherT& pub,
                           ros2_cuda_ipc_msgs::msg::GpuBuffer& msg,
                           int expected_consumers, bool blocking = true) {
    auto no_op = [](void*, uint32_t, uint32_t, uint32_t, cudaStream_t) {};
    return produce_and_publish(pub, msg, expected_consumers, no_op, blocking);
  }

  // Access to producer stream (may be nullptr)
  cudaStream_t producer_stream() const { return producer_stream_; }

 private:
  GpuBufferPool pool_;
  LeaseManager lease_mgr_;
  std::string owner_;
  cudaStream_t producer_stream_{nullptr};
};

}  // namespace ros2_cuda_ipc_core

#endif  // ROS2_CUDA_IPC_CORE_ZERO_COPY_PUBLISHER_HPP_
