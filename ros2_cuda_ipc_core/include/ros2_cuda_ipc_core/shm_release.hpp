#pragma once

#include <atomic>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace ros2_cuda_ipc_core {

// Simple POSIX SHM control block to manage per-frame release counting.
// Layout: magic, seq, slot, refcnt
struct ShmReleaseHeader {
  static constexpr uint32_t kMagic = 0x52434950;  // 'RCIP'
  uint32_t magic;
  uint32_t slot;
  uint64_t seq;
  int32_t refcnt;
};

// Compose a slot-fixed shm object name. Must begin with '/'.
std::string make_slot_shm_name(uint32_t slot);

// Backward-compatible helper: now returns slot-fixed name, ignoring seq for
// naming.
std::string make_shm_name(uint32_t slot, uint64_t seq);

// Sanitize an owner string for SHM naming: [A-Za-z0-9_], others mapped to '_'.
std::string sanitize_shm_owner(std::string_view owner);

// Compose a slot-fixed name with owner prefix: "/rcip_<owner>_<slot>"
std::string make_slot_shm_name_with_owner(std::string_view owner,
                                          uint32_t slot);

// Create-or-open and (re)initialize the SHM control block header for a frame.
// Returns the shm name on success.
std::optional<std::string> shm_create_init(const std::string& name,
                                           uint32_t slot, uint64_t seq,
                                           int32_t refcnt);

// Atomically decrement refcnt if header matches slot/seq. Returns new value on
// success.
std::optional<int32_t> shm_decrement(const std::string& name, uint32_t slot,
                                     uint64_t seq);

// Read current refcnt (for polling). Returns nullopt on error.
std::optional<int32_t> shm_read_refcnt(const std::string& name, uint32_t slot,
                                       uint64_t seq);

// Unlink the SHM object (best-effort cleanup).
bool shm_unlink(const std::string& name);

// Portable-ish atomic decrement for shared-memory refcounts.
// Uses C++20 atomic_ref when available; otherwise falls back to GCC/Clang
// builtins.
int32_t shm_atomic_dec_seq_cst(int32_t* ptr);

}  // namespace ros2_cuda_ipc_core
