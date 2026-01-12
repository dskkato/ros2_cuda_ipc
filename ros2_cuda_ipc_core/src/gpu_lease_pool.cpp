#include "ros2_cuda_ipc_core/cuda/gpu_lease_pool.hpp"

#include <cuda.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <uuid/uuid.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <memory>
#include <thread>
#include <utility>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/cuda/cuda_util.hpp"
#include "ros2_cuda_ipc_core/lease_handle.hpp"
#include "ros2_cuda_ipc_core/memory_types.hpp"

namespace ros2_cuda_ipc_core::cuda {

struct GpuLeasePool::SlotBackendState {
  virtual ~SlotBackendState() = default;
};

namespace {
using Clock = std::chrono::steady_clock;

bool deadline_reached(const Clock::time_point &deadline,
                      const Clock::time_point &now) {
  return deadline.time_since_epoch().count() != 0 && now >= deadline;
}

uint64_t align_up(uint64_t value, uint64_t alignment) {
  if (alignment == 0) {
    return value;
  }
  const uint64_t remainder = value % alignment;
  if (remainder == 0) {
    return value;
  }
  return value + alignment - remainder;
}

class UnixFdServer {
 public:
  UnixFdServer(std::string socket_path, int fd_to_send, rclcpp::Logger logger)
      : path_(std::move(socket_path)),
        fd_(fd_to_send),
        logger_(std::move(logger)) {
    if (fd_ < 0) {
      RCLCPP_ERROR(logger_, "Cannot start UnixFdServer at %s with invalid fd",
                   path_.c_str());
      throw std::invalid_argument("Invalid fd for UnixFdServer");
    }
  }

  ~UnixFdServer() { stop(); }

  bool start() {
    stop();
    int sock = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (sock < 0) {
      if (errno == EINVAL || errno == EPROTOTYPE) {
        sock = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (sock >= 0) {
          fcntl(sock, F_SETFD, FD_CLOEXEC);
        }
      }
    }
    if (sock < 0) {
      RCLCPP_ERROR(logger_, "socket(AF_UNIX) failed: %s", strerror(errno));
      return false;
    }

    ::unlink(path_.c_str());

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, path_.c_str(), sizeof(addr.sun_path) - 1);
    if (::bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(logger_, "bind(%s) failed: %s", path_.c_str(),
                   strerror(errno));
      ::close(sock);
      return false;
    }

    if (::listen(sock, 16) < 0) {
      RCLCPP_ERROR(logger_, "listen(%s) failed: %s", path_.c_str(),
                   strerror(errno));
      ::close(sock);
      return false;
    }

    listen_fd_ = sock;
    running_.store(true, std::memory_order_release);
    worker_ = std::thread([this]() { run(); });
    return true;
  }

  void stop() {
    running_.store(false, std::memory_order_release);
    if (listen_fd_ >= 0) {
      ::shutdown(listen_fd_, SHUT_RDWR);
      ::close(listen_fd_);
      listen_fd_ = -1;
    }
    if (worker_.joinable()) {
      worker_.join();
    }
    ::unlink(path_.c_str());
  }

 private:
  void run() {
    while (running_.load(std::memory_order_acquire)) {
      int client = ::accept(listen_fd_, nullptr, nullptr);
      if (client < 0) {
        if (errno == EINTR) {
          continue;
        }
        if (running_.load(std::memory_order_acquire)) {
          RCLCPP_WARN(logger_, "accept(%s) failed: %s", path_.c_str(),
                      strerror(errno));
        }
        continue;
      }
      int flags = fcntl(client, F_GETFD);
      if (flags >= 0) {
        fcntl(client, F_SETFD, flags | FD_CLOEXEC);
      }
      send_fd(client);
      ::close(client);
    }
  }

  void send_fd(int client) {
    char buf = 'F';
    struct iovec iov {
      &buf, 1
    };
    alignas(struct cmsghdr) char cmsg_buf[CMSG_SPACE(sizeof(int))];
    struct msghdr msg {};
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = cmsg_buf;
    msg.msg_controllen = sizeof(cmsg_buf);
    cmsghdr *cmsg = CMSG_FIRSTHDR(&msg);
    cmsg->cmsg_level = SOL_SOCKET;
    cmsg->cmsg_type = SCM_RIGHTS;
    cmsg->cmsg_len = CMSG_LEN(sizeof(int));
    std::memcpy(CMSG_DATA(cmsg), &fd_, sizeof(int));
    if (::sendmsg(client, &msg, 0) < 0) {
      RCLCPP_WARN(logger_, "sendmsg failed on %s: %s", path_.c_str(),
                  strerror(errno));
    }
  }

  std::string path_;
  const int fd_ = -1;
  int listen_fd_ = -1;
  std::atomic<bool> running_{false};
  std::thread worker_;
  rclcpp::Logger logger_;
};

class CudaIpcMemoryBackend : public GpuLeasePool::MemoryBackend {
 public:
  bool allocate(uint64_t frame_size_bytes, int device_index,
                std::vector<GpuLeasePool::Slot> &slots,
                rclcpp::Logger logger) override {
    (void)device_index;
    for (auto &slot : slots) {
      cudaError_t err = cudaMalloc(&slot.device_ptr, frame_size_bytes);
      if (err != cudaSuccess) {
        RCLCPP_ERROR(logger, "cudaMalloc failed: %s",
                     cuda_error_to_string(err).c_str());
        destroy(slots, logger);
        return false;
      }

      cudaIpcMemHandle_t handle{};
      err = cudaIpcGetMemHandle(&handle, slot.device_ptr);
      if (err != cudaSuccess) {
        RCLCPP_ERROR(logger, "cudaIpcGetMemHandle failed: %s",
                     cuda_error_to_string(err).c_str());
        destroy(slots, logger);
        return false;
      }
      std::memcpy(slot.mem_handle.data(), &handle, sizeof(handle));
      slot.backend = ros2_cuda_ipc_core::MemoryBackendKind::CUDA_IPC;
      slot.backend_state.reset();
    }
    return true;
  }

  void destroy(std::vector<GpuLeasePool::Slot> &slots,
               rclcpp::Logger logger) noexcept override {
    for (auto &slot : slots) {
      if (slot.device_ptr) {
        const cudaError_t free_err = cudaFree(slot.device_ptr);
        if (free_err != cudaSuccess) {
          RCLCPP_ERROR(logger, "cudaFree failed for slot %u: %s", slot.index,
                       cuda_error_to_string(free_err).c_str());
        }
        slot.device_ptr = nullptr;
      }
      slot.mem_handle.fill(0);
      slot.backend_state.reset();
      slot.backend = ros2_cuda_ipc_core::MemoryBackendKind::CUDA_IPC;
    }
  }
};

struct VmmSlotState : public GpuLeasePool::SlotBackendState {
  ~VmmSlotState() override {
    if (server) {
      server->stop();
      server.reset();
    }
    if (shareable_fd >= 0) {
      ::close(shareable_fd);
      shareable_fd = -1;
    }
    if (address && allocation_size > 0) {
      cuMemUnmap(address, allocation_size);
      cuMemAddressFree(address, allocation_size);
    }
    if (allocation) {
      cuMemRelease(allocation);
    }
  }

  CUmemGenericAllocationHandle allocation = 0;
  CUdeviceptr address = 0;
  std::size_t allocation_size = 0;
  int shareable_fd = -1;
  std::string uuid;
  std::unique_ptr<UnixFdServer> server;
};

class VmmFdMemoryBackend : public GpuLeasePool::MemoryBackend {
 public:
  bool allocate(uint64_t frame_size_bytes, int device_index,
                std::vector<GpuLeasePool::Slot> &slots,
                rclcpp::Logger logger) override {
    if (!ensure_driver(logger)) {
      return false;
    }

    CUmemAllocationProp prop{};
    prop.type = CU_MEM_ALLOCATION_TYPE_PINNED;
    prop.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
    prop.location.id = device_index;
    prop.requestedHandleTypes = CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR;

    size_t granularity = 0;
    CUresult res = cuMemGetAllocationGranularity(
        &granularity, &prop, CU_MEM_ALLOC_GRANULARITY_MINIMUM);
    if (res != CUDA_SUCCESS) {
      RCLCPP_ERROR(logger, "cuMemGetAllocationGranularity failed: %s",
                   cu_result_to_string(res).c_str());
      return false;
    }

    const uint64_t aligned_size = align_up(frame_size_bytes, granularity);
    for (auto &slot : slots) {
      auto state = std::make_shared<VmmSlotState>();
      state->allocation_size = aligned_size;

      CUdeviceptr address = 0;
      res = cuMemAddressReserve(&address, aligned_size, 0, 0, 0);
      if (res != CUDA_SUCCESS) {
        RCLCPP_ERROR(logger, "cuMemAddressReserve failed: %s",
                     cu_result_to_string(res).c_str());
        destroy(slots, logger);
        return false;
      }
      state->address = address;

      res = cuMemCreate(&state->allocation, aligned_size, &prop, 0);
      if (res != CUDA_SUCCESS) {
        RCLCPP_ERROR(logger, "cuMemCreate failed: %s",
                     cu_result_to_string(res).c_str());
        destroy(slots, logger);
        return false;
      }

      res = cuMemMap(address, aligned_size, 0, state->allocation, 0);
      if (res != CUDA_SUCCESS) {
        RCLCPP_ERROR(logger, "cuMemMap failed: %s",
                     cu_result_to_string(res).c_str());
        destroy(slots, logger);
        return false;
      }

      CUmemAccessDesc access_desc{};
      access_desc.location = prop.location;
      access_desc.flags = CU_MEM_ACCESS_FLAGS_PROT_READWRITE;
      res = cuMemSetAccess(address, aligned_size, &access_desc, 1);
      if (res != CUDA_SUCCESS) {
        RCLCPP_ERROR(logger, "cuMemSetAccess failed: %s",
                     cu_result_to_string(res).c_str());
        destroy(slots, logger);
        return false;
      }

      res = cuMemExportToShareableHandle(
          &state->shareable_fd, state->allocation,
          CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR, 0);
      if (res != CUDA_SUCCESS) {
        RCLCPP_ERROR(logger, "cuMemExportToShareableHandle failed: %s",
                     cu_result_to_string(res).c_str());
        destroy(slots, logger);
        return false;
      }
      if (state->shareable_fd >= 0) {
        int flags = fcntl(state->shareable_fd, F_GETFD);
        if (flags >= 0) {
          fcntl(state->shareable_fd, F_SETFD, flags | FD_CLOEXEC);
        }
      }

      uuid_t uuid_bytes;
      uuid_generate(uuid_bytes);
      char uuid_str[37] = {};
      uuid_unparse_lower(uuid_bytes, uuid_str);
      state->uuid = uuid_str;

      const auto socket_path = build_memory_socket_path(state->uuid);
      auto child_logger = logger.get_child("FdServer");
      state->server = std::make_unique<UnixFdServer>(
          socket_path, state->shareable_fd, child_logger);
      if (!state->server->start()) {
        destroy(slots, logger);
        return false;
      }

      slot.device_ptr = reinterpret_cast<void *>(state->address);
      slot.backend = ros2_cuda_ipc_core::MemoryBackendKind::VMM_FD;
      slot.backend_state = state;
      if (!ros2_cuda_ipc_core::encode_uuid_payload(state->uuid,
                                                   slot.mem_handle)) {
        RCLCPP_ERROR(logger, "Failed to encode UUID payload for slot %u",
                     slot.index);
        destroy(slots, logger);
        return false;
      }
    }
    return true;
  }

  void destroy(std::vector<GpuLeasePool::Slot> &slots,
               rclcpp::Logger logger) noexcept override {
    (void)logger;
    for (auto &slot : slots) {
      slot.device_ptr = nullptr;
      if (slot.backend == ros2_cuda_ipc_core::MemoryBackendKind::VMM_FD) {
        slot.backend_state.reset();
      }
      slot.mem_handle.fill(0);
      slot.backend = ros2_cuda_ipc_core::MemoryBackendKind::CUDA_IPC;
    }
  }

 private:
  bool ensure_driver(const rclcpp::Logger &logger) {
    static std::once_flag once;
    static CUresult status = CUDA_SUCCESS;
    std::call_once(once, [&]() { status = cuInit(0); });
    if (status != CUDA_SUCCESS) {
      RCLCPP_ERROR(logger, "cuInit failed: %s",
                   cu_result_to_string(status).c_str());
      return false;
    }
    return true;
  }
};

std::unique_ptr<GpuLeasePool::MemoryBackend> make_backend(
    ros2_cuda_ipc_core::MemoryBackendKind backend) {
  if (backend == ros2_cuda_ipc_core::MemoryBackendKind::VMM_FD) {
    return std::make_unique<VmmFdMemoryBackend>();
  }
  return std::make_unique<CudaIpcMemoryBackend>();
}

}  // namespace

using ros2_cuda_ipc_core::LeaseHandle;

GpuLeasePool::GpuLeasePool(Config config, rclcpp::Logger logger)
    : config_(std::move(config)), logger_(std::move(logger)) {}

GpuLeasePool::~GpuLeasePool() { destroy_slots(); }

bool GpuLeasePool::initialise(uint64_t frame_size_bytes, int device_index) {
  if (config_.slot_count == 0) {
    RCLCPP_ERROR(logger_, "GpuLeasePool requires slot_count > 0");
    return false;
  }

  destroy_slots();

  cudaError_t err = cudaSetDevice(device_index);
  if (err != cudaSuccess) {
    RCLCPP_ERROR(logger_, "cudaSetDevice failed: %s",
                 cuda_error_to_string(err).c_str());
    return false;
  }

  if (!LeaseHandle::init(config_.shm_name,
                         static_cast<uint32_t>(config_.slot_count))) {
    RCLCPP_ERROR(logger_, "Failed to initialise lease shared memory %s",
                 config_.shm_name.c_str());
    return false;
  }

  frame_size_bytes_ = frame_size_bytes;
  device_index_ = device_index;
  slots_.assign(config_.slot_count, {});
  for (std::size_t i = 0; i < slots_.size(); ++i) {
    slots_[i].index = static_cast<uint32_t>(i);
  }

  if (!allocate_slots()) {
    destroy_slots();
    return false;
  }

  initialised_ = true;
  return true;
}

void GpuLeasePool::reset() noexcept { destroy_slots(); }

bool GpuLeasePool::matches(uint64_t frame_size_bytes,
                           int device_index) const noexcept {
  return initialised_ && frame_size_bytes_ == frame_size_bytes &&
         device_index_ == device_index;
}

std::optional<GpuLeasePool::Slot *> GpuLeasePool::acquire(
    std::size_t subscriber_count) {
  if (!initialised_) {
    return std::nullopt;
  }

  auto free_slot = LeaseHandle::choose_empty_slot(config_.shm_name);
  if (!free_slot.has_value()) {
    return std::nullopt;
  }
  if (free_slot.value() >= slots_.size()) {
    RCLCPP_ERROR(logger_, "LeaseHandle returned invalid slot index %u",
                 free_slot.value());
    return std::nullopt;
  }

  Slot &slot = slots_[free_slot.value()];

  auto generation = LeaseHandle::bump_generation(
      config_.shm_name, slot.index, static_cast<uint32_t>(subscriber_count));
  if (!generation.has_value()) {
    RCLCPP_WARN(logger_, "Failed to bump generation for slot %u", slot.index);
    return std::nullopt;
  }
  slot.generation = generation.value();

  if (subscriber_count > 0 && config_.pending_ttl.count() > 0) {
    slot.pending_deadline = Clock::now() + config_.pending_ttl;
  } else {
    slot.pending_deadline = {};
  }

  return &slot;
}

void GpuLeasePool::reclaim_stale_pending() {
  if (!initialised_ || config_.pending_ttl.count() <= 0) {
    return;
  }

  const auto now = Clock::now();

  for (auto &slot : slots_) {
    if (!deadline_reached(slot.pending_deadline, now)) {
      continue;
    }

    auto pending = LeaseHandle::current_pending(config_.shm_name, slot.index);
    if (!pending.has_value()) {
      continue;
    }
    if (pending.value() == 0) {
      slot.pending_deadline = {};
      continue;
    }

    auto refcnt = LeaseHandle::current_refcount(config_.shm_name, slot.index);
    if (!refcnt.has_value() || refcnt.value() != 0) {
      continue;
    }

    if (LeaseHandle::force_clear_pending(config_.shm_name, slot.index)) {
      RCLCPP_WARN(
          logger_, "Force-cleared pending lease slot=%u after %lld ms timeout",
          slot.index, static_cast<long long>(config_.pending_ttl.count()));
      slot.pending_deadline = {};
    }
  }
}

bool GpuLeasePool::cancel_pending(Slot &slot) {
  slot.pending_deadline = {};
  if (!initialised_) {
    return false;
  }

  if (LeaseHandle::force_clear_pending(config_.shm_name, slot.index)) {
    RCLCPP_DEBUG(logger_, "Cleared pending lease for slot %u", slot.index);
    return true;
  }
  return false;
}

bool GpuLeasePool::allocate_slots() {
  memory_backend_ = make_backend(config_.backend);
  if (!memory_backend_) {
    RCLCPP_ERROR(logger_, "Failed to create memory backend");
    return false;
  }

  if (!memory_backend_->allocate(frame_size_bytes_, device_index_, slots_,
                                 logger_)) {
    memory_backend_.reset();
    return false;
  }

  for (auto &slot : slots_) {
    cudaError_t err = cudaEventCreateWithFlags(
        &slot.event, cudaEventDisableTiming | cudaEventInterprocess);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(logger_, "cudaEventCreateWithFlags failed: %s",
                   cuda_error_to_string(err).c_str());
      return false;
    }

    err = cudaIpcGetEventHandle(&slot.event_handle, slot.event);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(logger_, "cudaIpcGetEventHandle failed: %s",
                   cuda_error_to_string(err).c_str());
      return false;
    }
  }

  return true;
}

void GpuLeasePool::destroy_slots() noexcept {
  if (device_index_ >= 0) {
    cudaSetDevice(device_index_);
  }

  for (auto &slot : slots_) {
    if (slot.event) {
      const cudaError_t event_err = cudaEventDestroy(slot.event);
      if (event_err != cudaSuccess) {
        RCLCPP_ERROR(logger_, "cudaEventDestroy failed for slot %u: %s",
                     slot.index, cuda_error_to_string(event_err).c_str());
      }
      slot.event = nullptr;
    }
    slot.pending_deadline = {};
  }
  if (memory_backend_) {
    memory_backend_->destroy(slots_, logger_);
    memory_backend_.reset();
  }
  slots_.clear();
  frame_size_bytes_ = 0;
  device_index_ = -1;
  initialised_ = false;
}

}  // namespace ros2_cuda_ipc_core::cuda
