#include "ros2_cuda_ipc_core/backend/vmm_fd_importer.hpp"

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <uuid/uuid.h>

#include <cerrno>
#include <cstring>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/logging.hpp"
#include "ros2_cuda_ipc_core/cuda/cuda_util.hpp"
#include "ros2_cuda_ipc_core/memory_types.hpp"
#include "ros2_cuda_ipc_core/posix_error.hpp"

namespace ros2_cuda_ipc_core::backend {

namespace {

std::size_t align_up_size(std::size_t value, std::size_t alignment) {
  if (alignment == 0) {
    return value;
  }
  const std::size_t remainder = value % alignment;
  if (remainder == 0) {
    return value;
  }
  return value + alignment - remainder;
}

bool ensure_cuda_driver_initialised(const rclcpp::Logger& logger) {
  static std::once_flag once;
  static CUresult init_status = CUDA_SUCCESS;
  std::call_once(once, []() { init_status = cuInit(0); });
  if (init_status != CUDA_SUCCESS) {
    RCLCPP_ERROR(
        logger, "cuInit failed: %s",
        ros2_cuda_ipc_core::cuda::cu_result_to_string(init_status).c_str());
    return false;
  }
  return true;
}

struct VmmPayload {
  std::string uuid;
};

std::optional<VmmPayload> parse_vmm_payload(const MemoryHandlePayload& payload,
                                            const rclcpp::Logger& logger) {
  const uint32_t length = load_u32_le(payload.data());
  if (length == 0 || length > ros2_cuda_ipc_core::kVmmUuidMaxLength ||
      length + 4 > payload.size()) {
    RCLCPP_WARN(logger, "Received invalid VMM_FD payload length=%u (max=%zu)",
                length, ros2_cuda_ipc_core::kVmmUuidMaxLength);
    return std::nullopt;
  }

  uuid_t uuid;
  const char* uuid_str = reinterpret_cast<const char*>(payload.data() + 4);
  if (uuid_parse_range(uuid_str, uuid_str + length, uuid) != 0) {
    RCLCPP_WARN(logger, "Failed to parse UUID from VMM_FD payload");
    return std::nullopt;
  }

  VmmPayload result;
  result.uuid.assign(uuid_str, length);
  return result;
}

std::optional<int> request_fd_from_publisher(const std::string& path,
                                             const rclcpp::Logger& logger) {
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
    RCLCPP_WARN(logger, "socket(AF_UNIX) failed: %s",
                errno_to_string().c_str());
    return std::nullopt;
  }

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  if (path.size() >= sizeof(addr.sun_path)) {
    RCLCPP_WARN(logger, "Socket path %s is too long", path.c_str());
    ::close(sock);
    return std::nullopt;
  }
  std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
  if (::connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    RCLCPP_WARN(logger, "connect(%s) failed: %s", path.c_str(),
                errno_to_string().c_str());
    ::close(sock);
    return std::nullopt;
  }

  char buf = 0;
  struct iovec iov{&buf, 1};
  alignas(struct cmsghdr) char cmsg_buf[CMSG_SPACE(sizeof(int))];
  struct msghdr msg{};
  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;
  msg.msg_control = cmsg_buf;
  msg.msg_controllen = sizeof(cmsg_buf);
  const ssize_t received = ::recvmsg(sock, &msg, 0);
  if (received <= 0) {
    RCLCPP_WARN(logger, "recvmsg on %s failed: %s", path.c_str(),
                errno_to_string().c_str());
    ::close(sock);
    return std::nullopt;
  }

  cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
  if (!cmsg || cmsg->cmsg_level != SOL_SOCKET ||
      cmsg->cmsg_type != SCM_RIGHTS || cmsg->cmsg_len < CMSG_LEN(sizeof(int))) {
    RCLCPP_WARN(logger, "recvmsg on %s missing SCM_RIGHTS payload",
                path.c_str());
    ::close(sock);
    return std::nullopt;
  }

  int fd = -1;
  std::memcpy(&fd, CMSG_DATA(cmsg), sizeof(int));
  ::close(sock);
  if (fd < 0) {
    RCLCPP_WARN(logger, "recvmsg returned invalid fd for %s", path.c_str());
    return std::nullopt;
  }
  return fd;
}

}  // namespace

std::optional<ImportedBuffer> VmmFdImporter::import(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg,
    const cudaIpcEventHandle_t& event_handle,
    const rclcpp::Logger& logger) const {
  const auto meta = parse_vmm_payload(msg.mem_handle, logger);
  if (!meta.has_value()) {
    return std::nullopt;
  }

  const std::string socket_path = build_memory_socket_path(meta->uuid);
  if (socket_path.size() >= sizeof(sockaddr_un::sun_path)) {
    RCLCPP_WARN(logger, "UUID %s is too long for AF_UNIX path",
                socket_path.c_str());
    return std::nullopt;
  }

  const auto fd_opt = request_fd_from_publisher(socket_path, logger);
  if (!fd_opt.has_value()) {
    return std::nullopt;
  }

  if (!ensure_cuda_driver_initialised(logger)) {
    ::close(fd_opt.value());
    return std::nullopt;
  }

  ImportedBuffer imported;

  void* os_handle =
      reinterpret_cast<void*>(static_cast<intptr_t>(fd_opt.value()));
  CUresult cu_res =
      cuMemImportFromShareableHandle(&imported.vmm_allocation, os_handle,
                                     CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR);
  ::close(fd_opt.value());
  if (cu_res != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "cuMemImportFromShareableHandle failed: %s",
                ros2_cuda_ipc_core::cuda::cu_result_to_string(cu_res).c_str());
    return std::nullopt;
  }

  CUmemAllocationProp prop{};
  prop.type = CU_MEM_ALLOCATION_TYPE_PINNED;
  prop.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
  prop.location.id = static_cast<int>(msg.device_id);
  prop.requestedHandleTypes = CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR;
  std::size_t granularity = 0;
  cu_res = cuMemGetAllocationGranularity(&granularity, &prop,
                                         CU_MEM_ALLOC_GRANULARITY_MINIMUM);
  if (cu_res != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "cuMemGetAllocationGranularity failed: %s",
                ros2_cuda_ipc_core::cuda::cu_result_to_string(cu_res).c_str());
    cuMemRelease(imported.vmm_allocation);
    return std::nullopt;
  }

  imported.allocation_size =
      align_up_size(static_cast<std::size_t>(msg.byte_size), granularity);
  if (imported.allocation_size == 0) {
    imported.allocation_size = granularity;
  }

  cu_res = cuMemAddressReserve(&imported.vmm_address, imported.allocation_size,
                               0, 0, 0);
  if (cu_res != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "cuMemAddressReserve failed: %s",
                ros2_cuda_ipc_core::cuda::cu_result_to_string(cu_res).c_str());
    cuMemRelease(imported.vmm_allocation);
    return std::nullopt;
  }

  cu_res = cuMemMap(imported.vmm_address, imported.allocation_size, 0,
                    imported.vmm_allocation, 0);
  if (cu_res != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "cuMemMap failed: %s",
                ros2_cuda_ipc_core::cuda::cu_result_to_string(cu_res).c_str());
    cuMemAddressFree(imported.vmm_address, imported.allocation_size);
    cuMemRelease(imported.vmm_allocation);
    return std::nullopt;
  }

  CUmemAccessDesc access_desc{};
  access_desc.location = prop.location;
  access_desc.flags = CU_MEM_ACCESS_FLAGS_PROT_READWRITE;
  cu_res = cuMemSetAccess(imported.vmm_address, imported.allocation_size,
                          &access_desc, 1);
  if (cu_res != CUDA_SUCCESS) {
    RCLCPP_WARN(logger, "cuMemSetAccess failed: %s",
                ros2_cuda_ipc_core::cuda::cu_result_to_string(cu_res).c_str());
    cuMemUnmap(imported.vmm_address, imported.allocation_size);
    cuMemAddressFree(imported.vmm_address, imported.allocation_size);
    cuMemRelease(imported.vmm_allocation);
    return std::nullopt;
  }

  auto err = cudaIpcOpenEventHandle(&imported.event, event_handle);
  if (err != cudaSuccess) {
    RCLCPP_WARN(logger, "cudaIpcOpenEventHandle failed: %s",
                cudaGetErrorString(err));
    cuMemUnmap(imported.vmm_address, imported.allocation_size);
    cuMemAddressFree(imported.vmm_address, imported.allocation_size);
    cuMemRelease(imported.vmm_allocation);
    return std::nullopt;
  }

  imported.dev_ptr = reinterpret_cast<void*>(imported.vmm_address);
  return imported;
}

}  // namespace ros2_cuda_ipc_core::backend
