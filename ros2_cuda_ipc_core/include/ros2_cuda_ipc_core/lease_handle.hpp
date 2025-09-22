#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

namespace ros2_cuda_ipc_core {

class LeaseHandle {
 public:
  static bool init(const std::string &shm_name, uint32_t capacity);
  static std::optional<uint32_t> choose_empty_slot(const std::string &shm_name);
  static std::optional<uint32_t> current_generation(const std::string &shm_name,
                                                    uint32_t slot_id);
  static std::optional<uint32_t> current_refcount(const std::string &shm_name,
                                                  uint32_t slot_id);
  static std::optional<uint32_t> bump_generation(const std::string &shm_name,
                                                 uint32_t slot_id);

  static LeaseHandle acquire(const std::string &shm_name, uint32_t slot_id,
                             uint32_t generation);

  ~LeaseHandle();

  LeaseHandle(LeaseHandle &&other) noexcept;
  LeaseHandle &operator=(LeaseHandle &&other) noexcept;
  LeaseHandle(const LeaseHandle &) = delete;
  LeaseHandle &operator=(const LeaseHandle &) = delete;

  bool valid() const noexcept { return slot_meta_ != nullptr; }
  uint32_t slot_id() const noexcept { return slot_id_; }
  uint32_t generation() const noexcept { return generation_; }

 private:
  struct Mapping;
  struct SlotMeta;

  LeaseHandle() = default;
  LeaseHandle(std::shared_ptr<Mapping> mapping, SlotMeta *slot,
              uint32_t slot_id, uint32_t generation);

  void release() noexcept;

  std::shared_ptr<Mapping> mapping_;
  SlotMeta *slot_meta_ = nullptr;
  uint32_t slot_id_ = 0;
  uint32_t generation_ = 0;

  static std::shared_ptr<Mapping> attach(const std::string &shm_name);
  static std::mutex &registry_mutex();
  static std::unordered_map<std::string, std::shared_ptr<Mapping>> &registry();
};

}  // namespace ros2_cuda_ipc_core
