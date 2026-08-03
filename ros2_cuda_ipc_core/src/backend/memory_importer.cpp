// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"

#include <fcntl.h>
#include <rcutils/logging_macros.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <optional>
#include <string>

#include "ros2_cuda_ipc_core/detail/cuda_util.hpp"
#include "ros2_cuda_ipc_core/detail/posix_error.hpp"

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

std::optional<int> request_fd_from_publisher(const std::string& path) {
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
    RCUTILS_LOG_WARN_NAMED(
        "ros2_cuda_ipc_core.backend.vmm_fd", "socket(AF_UNIX) failed: %s",
        ros2_cuda_ipc_core::detail::errno_to_string().c_str());
    return std::nullopt;
  }

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  if (path.size() >= sizeof(addr.sun_path)) {
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.backend.vmm_fd",
                           "Socket path %s is too long", path.c_str());
    ::close(sock);
    return std::nullopt;
  }
  std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
  if (::connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    RCUTILS_LOG_WARN_NAMED(
        "ros2_cuda_ipc_core.backend.vmm_fd", "connect(%s) failed: %s",
        path.c_str(), ros2_cuda_ipc_core::detail::errno_to_string().c_str());
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
    RCUTILS_LOG_WARN_NAMED(
        "ros2_cuda_ipc_core.backend.vmm_fd", "recvmsg on %s failed: %s",
        path.c_str(), ros2_cuda_ipc_core::detail::errno_to_string().c_str());
    ::close(sock);
    return std::nullopt;
  }

  cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
  if (!cmsg || cmsg->cmsg_level != SOL_SOCKET ||
      cmsg->cmsg_type != SCM_RIGHTS || cmsg->cmsg_len < CMSG_LEN(sizeof(int))) {
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.backend.vmm_fd",
                           "recvmsg on %s missing SCM_RIGHTS payload",
                           path.c_str());
    ::close(sock);
    return std::nullopt;
  }

  int fd = -1;
  std::memcpy(&fd, CMSG_DATA(cmsg), sizeof(int));
  ::close(sock);
  if (fd < 0) {
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.backend.vmm_fd",
                           "recvmsg returned invalid fd for %s", path.c_str());
    return std::nullopt;
  }
  return fd;
}

}  // namespace

std::optional<ImportedResources> VmmFdMemoryImporter::import(
    const ros2_cuda_ipc_msgs::msg::BufferCore& msg,
    const CUipcEventHandle& event_handle) const {
  const std::string& socket_path = msg.vmm_socket_path;
  if (socket_path.size() >= sizeof(sockaddr_un::sun_path)) {
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.backend.vmm_fd",
                           "VMM socket path %s is too long for AF_UNIX path",
                           socket_path.c_str());
    return std::nullopt;
  }

  const auto fd_opt = request_fd_from_publisher(socket_path);
  if (!fd_opt.has_value()) {
    return std::nullopt;
  }

  auto context_result = detail::CudaDeviceContext::retain_primary(
      static_cast<int>(msg.device_id));
  if (!context_result) {
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.backend.vmm_fd",
                           "Failed to retain CUDA primary context: %s",
                           context_result.error().to_string().c_str());
    ::close(fd_opt.value());
    return std::nullopt;
  }
  auto context = std::move(context_result).value();
  auto guard_result = context->push_current();
  if (!guard_result) {
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.backend.vmm_fd",
                           "Failed to activate CUDA context: %s",
                           guard_result.error().to_string().c_str());
    ::close(fd_opt.value());
    return std::nullopt;
  }
  auto guard = std::move(guard_result).value();

  ImportedResources imported;
  imported.context = context;

  void* os_handle =
      reinterpret_cast<void*>(static_cast<intptr_t>(fd_opt.value()));
  CUresult cu_res =
      cuMemImportFromShareableHandle(&imported.vmm_allocation, os_handle,
                                     CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR);
  ::close(fd_opt.value());
  if (cu_res != CUDA_SUCCESS) {
    RCUTILS_LOG_WARN_NAMED(
        "ros2_cuda_ipc_core.backend.vmm_fd",
        "cuMemImportFromShareableHandle failed: %s",
        ros2_cuda_ipc_core::detail::cu_result_to_string(cu_res).c_str());
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
    RCUTILS_LOG_WARN_NAMED(
        "ros2_cuda_ipc_core.backend.vmm_fd",
        "cuMemGetAllocationGranularity failed: %s",
        ros2_cuda_ipc_core::detail::cu_result_to_string(cu_res).c_str());
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
    RCUTILS_LOG_WARN_NAMED(
        "ros2_cuda_ipc_core.backend.vmm_fd", "cuMemAddressReserve failed: %s",
        ros2_cuda_ipc_core::detail::cu_result_to_string(cu_res).c_str());
    cuMemRelease(imported.vmm_allocation);
    return std::nullopt;
  }

  cu_res = cuMemMap(imported.vmm_address, imported.allocation_size, 0,
                    imported.vmm_allocation, 0);
  if (cu_res != CUDA_SUCCESS) {
    RCUTILS_LOG_WARN_NAMED(
        "ros2_cuda_ipc_core.backend.vmm_fd", "cuMemMap failed: %s",
        ros2_cuda_ipc_core::detail::cu_result_to_string(cu_res).c_str());
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
    RCUTILS_LOG_WARN_NAMED(
        "ros2_cuda_ipc_core.backend.vmm_fd", "cuMemSetAccess failed: %s",
        ros2_cuda_ipc_core::detail::cu_result_to_string(cu_res).c_str());
    cuMemUnmap(imported.vmm_address, imported.allocation_size);
    cuMemAddressFree(imported.vmm_address, imported.allocation_size);
    cuMemRelease(imported.vmm_allocation);
    return std::nullopt;
  }

  cu_res = cuIpcOpenEventHandle(&imported.event, event_handle);
  if (cu_res != CUDA_SUCCESS) {
    RCUTILS_LOG_WARN_NAMED("ros2_cuda_ipc_core.backend.vmm_fd",
                           "cuIpcOpenEventHandle failed: %s",
                           ros2_cuda_ipc_core::detail::CudaDriverError(cu_res)
                               .to_string()
                               .c_str());
    cuMemUnmap(imported.vmm_address, imported.allocation_size);
    cuMemAddressFree(imported.vmm_address, imported.allocation_size);
    cuMemRelease(imported.vmm_allocation);
    return std::nullopt;
  }

  imported.dev_ptr = reinterpret_cast<void*>(imported.vmm_address);
  return imported;
}

bool release_imported_resources(const ImportedResources& imported) noexcept {
  if (!imported.context) {
    return true;
  }

  auto guard_result = imported.context->push_current();
  if (!guard_result) {
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.backend.vmm_fd",
        "Failed to activate CUDA context for imported resource cleanup: %s",
        guard_result.error().to_string().c_str());
    return false;
  }
  auto guard = std::move(guard_result).value();
  bool success = true;
  const auto report_cleanup_failure = [&success](const char* operation,
                                                 CUresult result) {
    if (result == CUDA_SUCCESS) {
      return;
    }
    success = false;
    RCUTILS_LOG_ERROR_NAMED(
        "ros2_cuda_ipc_core.backend.vmm_fd",
        "%s failed during imported resource cleanup: %s", operation,
        detail::CudaDriverError(result).to_string().c_str());
  };

  if (imported.vmm_address != 0 && imported.allocation_size != 0) {
    report_cleanup_failure("cuMemUnmap", cuMemUnmap(imported.vmm_address,
                                                    imported.allocation_size));
    report_cleanup_failure(
        "cuMemAddressFree",
        cuMemAddressFree(imported.vmm_address, imported.allocation_size));
  }
  if (imported.vmm_allocation != 0) {
    report_cleanup_failure("cuMemRelease",
                           cuMemRelease(imported.vmm_allocation));
  }
  if (imported.event != nullptr) {
    report_cleanup_failure("cuEventDestroy", cuEventDestroy(imported.event));
  }
  return success;
}

}  // namespace ros2_cuda_ipc_core::backend
