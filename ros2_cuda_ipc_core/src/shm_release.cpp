#include "ros2_cuda_ipc_core/shm_release.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace ros2_cuda_ipc_core {

namespace {
inline bool write_header(void* p, uint32_t slot, uint64_t seq, int32_t refcnt) {
  if (!p) return false;
  auto* h = reinterpret_cast<ShmReleaseHeader*>(p);
  h->magic = ShmReleaseHeader::kMagic;
  h->slot = slot;
  h->seq = seq;
  h->refcnt = refcnt;
  return true;
}

inline bool validate_header(const ShmReleaseHeader* h, uint32_t slot,
                            uint64_t seq) {
  if (!h) return false;
  if (h->magic != ShmReleaseHeader::kMagic) return false;
  if (h->slot != slot) return false;
  if (h->seq != seq) return false;
  return true;
}
}  // namespace

std::string make_slot_shm_name(uint32_t slot) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "/rcip_%u", slot);
  return std::string(buf);
}

std::string make_shm_name(uint32_t slot, uint64_t /*seq*/) {
  // For slot-fixed naming, ignore seq for the name itself
  return make_slot_shm_name(slot);
}

std::string sanitize_shm_owner(std::string_view owner) {
  std::string out;
  out.reserve(owner.size());
  for (char c : owner) {
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') || c == '_') {
      out.push_back(c);
    } else {
      out.push_back('_');
    }
  }
  // Avoid empty
  if (out.empty()) out = "anon";
  // Limit length to avoid exceeding typical name limits (keep room for
  // suffixes)
  if (out.size() > 48) out.resize(48);
  return out;
}

std::string make_slot_shm_name_with_owner(std::string_view owner,
                                          uint32_t slot) {
  auto s = sanitize_shm_owner(owner);
  char buf[80];
  std::snprintf(buf, sizeof(buf), "/rcip_%s_%u", s.c_str(), slot);
  return std::string(buf);
}

std::optional<std::string> shm_create_init(const std::string& name,
                                           uint32_t slot, uint64_t seq,
                                           int32_t refcnt) {
  // Create or open existing slot SHM
  int fd = ::shm_open(name.c_str(), O_CREAT | O_RDWR, 0600);
  if (fd < 0) {
    return std::nullopt;
  }
  const size_t sz = sizeof(ShmReleaseHeader);
  // Ensure size is at least header size
  struct ::stat st;
  if (::fstat(fd, &st) == 0) {
    if (st.st_size < static_cast<off_t>(sz)) {
      if (::ftruncate(fd, static_cast<off_t>(sz)) != 0) {
        ::close(fd);
        return std::nullopt;
      }
    }
  }
  void* p = ::mmap(nullptr, sz, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (p == MAP_FAILED) {
    ::close(fd);
    return std::nullopt;
  }
  (void)write_header(p, slot, seq, refcnt);
  ::msync(p, sz, MS_SYNC);
  ::munmap(p, sz);
  ::close(fd);
  return name;
}

std::optional<int32_t> shm_decrement(const std::string& name, uint32_t slot,
                                     uint64_t seq) {
  int fd = ::shm_open(name.c_str(), O_RDWR, 0600);
  if (fd < 0) {
    return std::nullopt;
  }
  const size_t sz = sizeof(ShmReleaseHeader);
  void* p = ::mmap(nullptr, sz, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (p == MAP_FAILED) {
    ::close(fd);
    return std::nullopt;
  }
  auto* h = reinterpret_cast<ShmReleaseHeader*>(p);
  if (!validate_header(h, slot, seq)) {
    ::munmap(p, sz);
    ::close(fd);
    return std::nullopt;
  }
  // Use GCC atomic builtins on the shared int32_t memory.
  int32_t newv = __atomic_sub_fetch(&h->refcnt, 1, __ATOMIC_SEQ_CST);
  ::msync(p, sz, MS_SYNC);
  ::munmap(p, sz);
  ::close(fd);
  return newv;
}

std::optional<int32_t> shm_read_refcnt(const std::string& name, uint32_t slot,
                                       uint64_t seq) {
  int fd = ::shm_open(name.c_str(), O_RDONLY, 0600);
  if (fd < 0) {
    return std::nullopt;
  }
  const size_t sz = sizeof(ShmReleaseHeader);
  void* p = ::mmap(nullptr, sz, PROT_READ, MAP_SHARED, fd, 0);
  if (p == MAP_FAILED) {
    ::close(fd);
    return std::nullopt;
  }
  auto* h = reinterpret_cast<const ShmReleaseHeader*>(p);
  if (!validate_header(h, slot, seq)) {
    ::munmap(p, sz);
    ::close(fd);
    return std::nullopt;
  }
  int32_t v = h->refcnt;
  ::munmap(p, sz);
  ::close(fd);
  return v;
}

bool shm_unlink(const std::string& name) {
  int ret = ::shm_unlink(name.c_str());
  int err = errno;
  return ret == 0 || err == ENOENT;
}

}  // namespace ros2_cuda_ipc_core
