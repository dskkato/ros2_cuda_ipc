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

/**
 * @brief Tracks shared-memory leases for slots in a GpuBufferPool.
 *
 * A LeaseManager creates per-slot control blocks in shared memory and monitors
 * their reference counts.  When all consumers release a frame or when the
 * timeout expires, the slot is returned to the pool.  The class itself is
 * ROS-agnostic; the owner is expected to call tick() periodically.
 *
 * Typical sequence:
 * 1. Construct with a pool reference and optional timeout.
 * 2. set_owner() once to establish a SHM name prefix.
 * 3. For each published frame call start_lease().
 * 4. Periodically invoke tick() in the main loop to reclaim slots.
 * 5. Call cleanup() on shutdown to remove lingering SHM objects.
 *
 * @note start_lease() must be called exactly once per borrowed slot; otherwise
 * the pool entry may remain in use indefinitely.
 */
class LeaseManager {
 public:
  explicit LeaseManager(GpuBufferPool& pool, int lease_timeout_ms = 3000)
      : pool_(pool), lease_timeout_ms_(lease_timeout_ms) {}

  ~LeaseManager();

  /**
   * @brief Set an owner prefix used for SHM names.
   *
   * The prefix is sanitized to [A-Za-z0-9_].  Should be set before
   * start_lease() is called to avoid orphaned names.
   */
  void set_owner(std::string owner) { owner_ = sanitize_shm_owner(owner); }

  /**
   * @brief Create or open a shared-memory control block for a slot.
   *
   * Initializes the block for the given sequence and expected consumer count
   * and begins tracking the lease until release.
   *
   * @return SHM name on success; std::nullopt on failure.
   */
  std::optional<std::string> start_lease(uint32_t slot_id, uint64_t seq,
                                         int expected_consumers);

  /**
   * @brief Poll leases and release slots that are complete or expired.
   *
   * @return Number of slots returned to the pool during this call.
   */
  int tick();

  /**
   * @brief Best-effort cleanup of known SHM objects.
   */
  void cleanup();

  /**
   * @brief Update the lease timeout in milliseconds.
   */
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
