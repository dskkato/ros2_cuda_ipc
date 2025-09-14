#include "ros2_cuda_ipc_core/lease_manager.hpp"

#include <utility>

namespace ros2_cuda_ipc_core {

LeaseManager::~LeaseManager() { cleanup(); }

std::optional<std::string> LeaseManager::start_lease(uint32_t slot_id,
                                                     uint64_t seq,
                                                     int expected_consumers) {
  LeaseInfo info;
  info.seq = seq;
  info.expected = expected_consumers;
  info.since = std::chrono::steady_clock::now();
  // Compose SHM name using owner (if provided) or default slot name.
  std::string name;
  if (!owner_.empty()) {
    name = make_slot_shm_name_with_owner(owner_, slot_id);
  } else {
    name = make_slot_shm_name(slot_id);
  }
  auto created = shm_create_init(name, slot_id, seq, expected_consumers);
  if (!created.has_value()) {
    return std::nullopt;
  }
  info.shm_name = *created;
  known_shms_.insert(*created);
  leases_[slot_id] = std::move(info);
  return created;
}

int LeaseManager::tick() {
  int released = 0;
  const auto now = std::chrono::steady_clock::now();
  for (auto it = leases_.begin(); it != leases_.end();) {
    const uint32_t slot = it->first;
    const auto& info = it->second;
    // Prefer SHM refcnt reaching zero for immediate release
    if (!info.shm_name.empty()) {
      auto ref = shm_read_refcnt(info.shm_name, slot, info.seq);
      if (ref.has_value() && *ref <= 0) {
        (void)pool_.release(slot);
        it = leases_.erase(it);
        ++released;
        continue;
      }
    }
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - info.since);
    if (elapsed.count() > lease_timeout_ms_) {
      (void)pool_.release(slot);
      it = leases_.erase(it);
      ++released;
      continue;
    }
    ++it;
  }
  return released;
}

void LeaseManager::cleanup() {
  for (const auto& name : known_shms_) {
    (void)shm_unlink(name);
  }
  known_shms_.clear();
}

}  // namespace ros2_cuda_ipc_core
