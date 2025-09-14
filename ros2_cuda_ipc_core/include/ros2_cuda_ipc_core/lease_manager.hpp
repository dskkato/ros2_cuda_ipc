#ifndef ROS2_CUDA_IPC_CORE_LEASE_MANAGER_HPP_
#define ROS2_CUDA_IPC_CORE_LEASE_MANAGER_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "ros2_cuda_ipc_core/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_core/shm_release.hpp"

namespace ros2_cuda_ipc_core {

// Manages per-frame SHM release counters and timed auto-release for
// GpuBufferPool slots. ROS-agnostic; caller triggers tick() periodically.
class LeaseManager {
 public:
  explicit LeaseManager(GpuBufferPool& pool, int lease_timeout_ms = 3000)
      : pool_(pool), lease_timeout_ms_(lease_timeout_ms) {}

  ~LeaseManager();

  // Set an owner prefix used for SHM names: sanitized to [A-Za-z0-9_].
  void set_owner(std::string owner) { owner_ = sanitize_shm_owner(owner); }

  // Create-or-open a slot-fixed SHM control block and initialize it for the
  // given (slot, seq) with expected consumer refcount. Returns the SHM name on
  // success and starts tracking the lease until release.
  std::optional<std::string> start_lease(uint32_t slot_id, uint64_t seq,
                                         int expected_consumers);

  // Poll leases: when a lease hits refcnt==0 (via SHM) or times out, release
  // the slot back to the pool. Returns number of slots released in this tick.
  int tick();

  // Best-effort cleanup of known SHM objects.
  void cleanup();

  // Update timeout.
  void set_timeout_ms(int ms) { lease_timeout_ms_ = ms; }

 private:
  struct LeaseInfo {
    uint64_t seq{0};
    int expected{0};
    std::chrono::steady_clock::time_point since{};
    std::string shm_name;
  };

  GpuBufferPool& pool_;
  int lease_timeout_ms_{3000};
  std::string owner_;
  std::unordered_map<uint32_t, LeaseInfo> leases_;  // key: slot id
  std::unordered_set<std::string> known_shms_;
};

}  // namespace ros2_cuda_ipc_core

#endif  // ROS2_CUDA_IPC_CORE_LEASE_MANAGER_HPP_
