#include "ros2_cuda_ipc_core/lease_handle.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstring>
#include <mutex>
#include <optional>
#include <rclcpp/logging.hpp>
#include <unordered_map>

namespace ros2_cuda_ipc_core {
namespace {

constexpr uint32_t kShmMagic = 0x4C534531;  // 'LSE1'
constexpr uint32_t kLayoutVersion = 1;

/// Shared memory header structure stored at the start of the shared-memory
/// segment.
struct ShmHeader {
  uint32_t magic;
  uint32_t layout_version;
  uint32_t capacity;
  uint32_t reserved;
};

/// Treat a `uint32_t` reference as an `std::atomic<uint32_t>` to apply atomic
/// operations without altering the POD layout.
///
/// \param value Reference to a slot field inside shared memory.
/// \return std::atomic<uint32_t>& alias to the given reference.
inline std::atomic<uint32_t> &as_atomic(uint32_t &value) {
  return reinterpret_cast<std::atomic<uint32_t> &>(value);
}

/// Lazily instantiate and return the logger used by LeaseHandle operations.
rclcpp::Logger lease_logger() {
  static rclcpp::Logger logger =
      rclcpp::get_logger("ros2_cuda_ipc_core.LeaseHandle");
  return logger;
}

}  // namespace

struct LeaseHandle::SlotMeta {
  uint32_t generation;
  uint32_t refcnt;
};

struct LeaseHandle::Mapping {
  std::string name;
  uint32_t capacity = 0;
  size_t mapped_size = 0;
  void *addr = nullptr;
  SlotMeta *slots = nullptr;
  ShmHeader *header = nullptr;

  ~Mapping() {
    if (addr && mapped_size) {
      munmap(addr, mapped_size);
    }
  }
};

LeaseHandle::LeaseHandle(std::shared_ptr<Mapping> mapping, SlotMeta *slot,
                         uint32_t slot_id, uint32_t generation)
    : mapping_(std::move(mapping)),
      slot_meta_(slot),
      slot_id_(slot_id),
      generation_(generation) {}

LeaseHandle::LeaseHandle(LeaseHandle &&other) noexcept {
  *this = std::move(other);
}

LeaseHandle &LeaseHandle::operator=(LeaseHandle &&other) noexcept {
  if (this == &other) {
    return *this;
  }

  release();

  mapping_ = std::move(other.mapping_);
  slot_meta_ = other.slot_meta_;
  slot_id_ = other.slot_id_;
  generation_ = other.generation_;

  other.slot_meta_ = nullptr;
  other.slot_id_ = 0;
  other.generation_ = 0;

  return *this;
}

LeaseHandle::~LeaseHandle() { release(); }

std::mutex &LeaseHandle::registry_mutex() {
  static std::mutex mtx;
  return mtx;
}

std::unordered_map<std::string, std::shared_ptr<LeaseHandle::Mapping>> &
LeaseHandle::registry() {
  static std::unordered_map<std::string, std::shared_ptr<Mapping>> map;
  return map;
}

void LeaseHandle::release() noexcept {
  if (!slot_meta_) {
    return;
  }

  auto &ref = as_atomic(slot_meta_->refcnt);
  const uint32_t previous = ref.fetch_sub(1, std::memory_order_acq_rel);
  if (previous == 0) {
    RCLCPP_ERROR(lease_logger(), "lease:refcnt_underflow slot=%u", slot_id_);
    ref.store(0, std::memory_order_release);
  }

  slot_meta_ = nullptr;
  slot_id_ = 0;
  generation_ = 0;
  mapping_.reset();
}

bool LeaseHandle::init(const std::string &shm_name, uint32_t capacity) {
  if (capacity == 0) {
    RCLCPP_ERROR(lease_logger(), "lease:init capacity must be >0");
    return false;
  }

  const size_t size =
      sizeof(ShmHeader) + static_cast<size_t>(capacity) * sizeof(SlotMeta);
  const int fd = shm_open(shm_name.c_str(), O_CREAT | O_RDWR, 0660);
  if (fd < 0) {
    RCLCPP_ERROR(lease_logger(), "lease:init shm_open failed name=%s errno=%d",
                 shm_name.c_str(), errno);
    return false;
  }

  if (ftruncate(fd, size) != 0) {
    RCLCPP_ERROR(lease_logger(), "lease:init ftruncate failed name=%s errno=%d",
                 shm_name.c_str(), errno);
    close(fd);
    return false;
  }

  void *addr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (addr == MAP_FAILED) {
    RCLCPP_ERROR(lease_logger(), "lease:init mmap failed name=%s errno=%d",
                 shm_name.c_str(), errno);
    close(fd);
    return false;
  }

  auto *header = static_cast<ShmHeader *>(addr);
  header->magic = kShmMagic;
  header->layout_version = kLayoutVersion;
  header->capacity = capacity;
  header->reserved = 0;

  auto *slots = reinterpret_cast<SlotMeta *>(header + 1);
  for (uint32_t i = 0; i < capacity; ++i) {
    as_atomic(slots[i].generation).store(0u, std::memory_order_relaxed);
    as_atomic(slots[i].refcnt).store(0u, std::memory_order_relaxed);
  }

  munmap(addr, size);
  close(fd);

  {
    std::lock_guard<std::mutex> lock(registry_mutex());
    registry().erase(shm_name);
  }

  return true;
}

std::shared_ptr<LeaseHandle::Mapping> LeaseHandle::attach(
    const std::string &shm_name) {
  std::lock_guard<std::mutex> lock(registry_mutex());
  auto &map = registry();
  if (auto it = map.find(shm_name); it != map.end()) {
    return it->second;
  }

  const int fd = shm_open(shm_name.c_str(), O_RDWR, 0660);
  if (fd < 0) {
    RCLCPP_WARN(lease_logger(), "lease:attach shm_open failed name=%s errno=%d",
                shm_name.c_str(), errno);
    return nullptr;
  }

  struct stat st {};
  if (fstat(fd, &st) != 0) {
    RCLCPP_WARN(lease_logger(), "lease:attach fstat failed name=%s errno=%d",
                shm_name.c_str(), errno);
    close(fd);
    return nullptr;
  }

  void *addr =
      mmap(nullptr, st.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (addr == MAP_FAILED) {
    RCLCPP_WARN(lease_logger(), "lease:attach mmap failed name=%s errno=%d",
                shm_name.c_str(), errno);
    close(fd);
    return nullptr;
  }

  close(fd);

  auto header = static_cast<ShmHeader *>(addr);
  if (header->magic != kShmMagic || header->layout_version != kLayoutVersion) {
    RCLCPP_WARN(lease_logger(),
                "lease:attach header mismatch name=%s magic=%u ver=%u",
                shm_name.c_str(), header->magic, header->layout_version);
    munmap(addr, st.st_size);
    return nullptr;
  }

  auto mapping = std::shared_ptr<Mapping>(new Mapping);
  mapping->name = shm_name;
  mapping->capacity = header->capacity;
  mapping->mapped_size = st.st_size;
  mapping->addr = addr;
  mapping->header = header;
  mapping->slots = reinterpret_cast<SlotMeta *>(header + 1);

  map[shm_name] = mapping;
  return mapping;
}

std::optional<uint32_t> LeaseHandle::choose_empty_slot(
    const std::string &shm_name) {
  auto mapping = attach(shm_name);
  if (!mapping || mapping->capacity == 0) {
    return std::nullopt;
  }

  for (uint32_t i = 0; i < mapping->capacity; ++i) {
    auto &ref = as_atomic(mapping->slots[i].refcnt);
    if (ref.load(std::memory_order_acquire) == 0) {
      return i;
    }
  }
  return std::nullopt;
}

std::optional<uint32_t> LeaseHandle::current_generation(
    const std::string &shm_name, uint32_t slot_id) {
  auto mapping = attach(shm_name);
  if (!mapping || slot_id >= mapping->capacity) {
    return std::nullopt;
  }
  auto &gen = as_atomic(mapping->slots[slot_id].generation);
  return gen.load(std::memory_order_acquire);
}

std::optional<uint32_t> LeaseHandle::current_refcount(
    const std::string &shm_name, uint32_t slot_id) {
  auto mapping = attach(shm_name);
  if (!mapping || slot_id >= mapping->capacity) {
    return std::nullopt;
  }
  auto &ref = as_atomic(mapping->slots[slot_id].refcnt);
  return ref.load(std::memory_order_acquire);
}

std::optional<uint32_t> LeaseHandle::bump_generation(
    const std::string &shm_name, uint32_t slot_id) {
  auto mapping = attach(shm_name);
  if (!mapping || slot_id >= mapping->capacity) {
    return std::nullopt;
  }
  auto &gen = as_atomic(mapping->slots[slot_id].generation);
  const uint32_t next = gen.load(std::memory_order_relaxed) + 1;
  gen.store(next, std::memory_order_release);
  return next;
}

LeaseHandle LeaseHandle::acquire(const std::string &shm_name, uint32_t slot_id,
                                 uint32_t generation) {
  auto mapping = attach(shm_name);
  if (!mapping || slot_id >= mapping->capacity) {
    return LeaseHandle{};
  }
  SlotMeta *slot = &mapping->slots[slot_id];
  auto &gen = as_atomic(slot->generation);
  auto &ref = as_atomic(slot->refcnt);

  const uint32_t observed_gen = gen.load(std::memory_order_acquire);
  if (observed_gen != generation) {
    RCLCPP_DEBUG(lease_logger(),
                 "lease:gen_mismatch slot=%u expected=%u observed=%u", slot_id,
                 generation, observed_gen);
    return LeaseHandle{};
  }

  const uint32_t prev = ref.fetch_add(1, std::memory_order_acq_rel);
  if (prev == UINT32_MAX) {
    RCLCPP_ERROR(lease_logger(), "lease:ref_overflow slot=%u", slot_id);
    return LeaseHandle{};
  }

  const uint32_t recheck_gen = gen.load(std::memory_order_acquire);
  if (recheck_gen != generation) {
    ref.fetch_sub(1, std::memory_order_acq_rel);
    RCLCPP_DEBUG(lease_logger(),
                 "lease:gen_race slot=%u expected=%u observed=%u", slot_id,
                 generation, recheck_gen);
    return LeaseHandle{};
  }

  return LeaseHandle(std::move(mapping), slot, slot_id, generation);
}

}  // namespace ros2_cuda_ipc_core
