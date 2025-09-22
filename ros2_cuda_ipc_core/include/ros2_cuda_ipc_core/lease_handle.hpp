#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

namespace ros2_cuda_ipc_core {

/// LeaseHandle manages the lifetime of a shared-memory slot using process-wide
/// reference counting and generation checks.
class LeaseHandle {
 public:
  /// Create or reset the shared-memory layout for a lease pool.
  ///
  /// \param shm_name Shared-memory name (POSIX shm_open identifier).
  /// \param capacity Number of slots to allocate in the pool.
  /// \return true when the memory is initialized successfully.
  static bool init(const std::string &shm_name, uint32_t capacity);

  /// Find a slot whose reference count has dropped to zero.
  ///
  /// \param shm_name Shared-memory name to query.
  /// \return slot id on success; std::nullopt when no free slot exists or the
  /// mapping cannot be attached.
  static std::optional<uint32_t> choose_empty_slot(const std::string &shm_name);

  /// Read the current generation value for a slot.
  ///
  /// \param shm_name Shared-memory name to query.
  /// \param slot_id Slot index inside the pool.
  /// \return generation number; std::nullopt if attachment fails or the slot is
  /// out of range.
  static std::optional<uint32_t> current_generation(const std::string &shm_name,
                                                    uint32_t slot_id);

  /// Read the current reference count for a slot.
  ///
  /// \param shm_name Shared-memory name to query.
  /// \param slot_id Slot index inside the pool.
  /// \return reference count; std::nullopt if attachment fails or the slot is
  /// out of range.
  static std::optional<uint32_t> current_refcount(const std::string &shm_name,
                                                  uint32_t slot_id);

  /// Advance the generation number for a slot before publishing new data.
  ///
  /// \param shm_name Shared-memory name to update.
  /// \param slot_id Slot index inside the pool.
  /// \return next generation number; std::nullopt when the slot is invalid.
  static std::optional<uint32_t> bump_generation(const std::string &shm_name,
                                                 uint32_t slot_id);

  /// Acquire a lease for a slot if the generation matches and increment its
  /// reference count.
  ///
  /// \param shm_name Shared-memory name containing the slot metadata.
  /// \param slot_id Slot index that should be leased.
  /// \param generation Expected generation for the slot.
  /// \return Valid LeaseHandle when the slot is obtained; otherwise an invalid
  /// (empty) handle.
  static LeaseHandle acquire(const std::string &shm_name, uint32_t slot_id,
                             uint32_t generation);

  /// Release any held lease on destruction.
  ~LeaseHandle();

  LeaseHandle(LeaseHandle &&other) noexcept;
  LeaseHandle &operator=(LeaseHandle &&other) noexcept;
  LeaseHandle(const LeaseHandle &) = delete;
  LeaseHandle &operator=(const LeaseHandle &) = delete;

  /// Check whether this handle currently owns a lease.
  bool valid() const noexcept { return slot_meta_ != nullptr; }

  /// Slot identifier associated with the lease (undefined when invalid()).
  uint32_t slot_id() const noexcept { return slot_id_; }

  /// Publisher generation that this lease corresponds to (undefined when
  /// invalid()).
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
