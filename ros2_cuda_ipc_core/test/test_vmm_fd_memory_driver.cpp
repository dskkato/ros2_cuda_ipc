// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <gtest/gtest.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
#include "ros2_cuda_ipc_core/publisher/gpu_buffer_pool.hpp"
#include "vmm_fd_memory_test_protocol.hpp"

#ifndef VMM_FD_IMPORT_HELPER_PATH
#error "VMM_FD_IMPORT_HELPER_PATH is not defined"
#endif

extern char** environ;

namespace {

class ScopedSigpipeIgnore {
 public:
  ScopedSigpipeIgnore() : previous_(std::signal(SIGPIPE, SIG_IGN)) {}
  ~ScopedSigpipeIgnore() { std::signal(SIGPIPE, previous_); }

  ScopedSigpipeIgnore(const ScopedSigpipeIgnore&) = delete;
  ScopedSigpipeIgnore& operator=(const ScopedSigpipeIgnore&) = delete;

 private:
  using SignalHandler = void (*)(int);
  SignalHandler previous_;
};

bool write_all(int fd, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  std::size_t offset = 0;
  while (offset < size) {
    const ssize_t written = ::write(fd, bytes + offset, size - offset);
    if (written > 0) {
      offset += static_cast<std::size_t>(written);
      continue;
    }
    if (written < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
  return true;
}

}  // namespace

TEST(VmmFdMemoryDriverTest, ImportsReadsAndReleasesInChildProcess) {
  using ros2_cuda_ipc_core::detail::CudaDeviceContext;
  using ros2_cuda_ipc_core::publisher::GpuBufferPool;

  auto context_result = CudaDeviceContext::retain_primary(0);
  if (!context_result) {
    GTEST_SKIP() << "CUDA device not available: "
                 << context_result.error().to_string();
  }
  auto context = std::move(context_result).value();

  constexpr uint64_t kByteSize = 4096;
  constexpr uint8_t kExpectedValue = 0x5a;
  GpuBufferPool pool(1);
  ASSERT_TRUE(pool.initialise(kByteSize, 0));
  const auto* resources = pool.resources(0);
  ASSERT_NE(resources, nullptr);
  ASSERT_NE(pool.device_ptr(0), nullptr);

  {
    auto guard_result = context->push_current();
    ASSERT_TRUE(guard_result);
    auto guard = std::move(guard_result).value();
    const CUdeviceptr device_ptr = static_cast<CUdeviceptr>(
        reinterpret_cast<uintptr_t>(pool.device_ptr(0)));
    ASSERT_EQ(cuMemsetD8(device_ptr, kExpectedValue, kByteSize), CUDA_SUCCESS);
  }
  ASSERT_TRUE(pool.record_ready(0, nullptr));

  ros2_cuda_ipc_core::test::VmmFdMemoryTestPayload payload;
  payload.device_id = 0;
  payload.byte_size = kByteSize;
  payload.expected_value = kExpectedValue;
  ASSERT_LT(resources->vmm_socket_path.size(), payload.vmm_socket_path.size());
  std::copy(resources->vmm_socket_path.begin(),
            resources->vmm_socket_path.end(), payload.vmm_socket_path.begin());
  payload.event_handle = resources->ready_event->ipc_handle();

  int payload_pipe[2] = {-1, -1};
  ASSERT_EQ(::pipe(payload_pipe), 0);
  const std::string fd_arg = std::to_string(payload_pipe[0]);
  char* const child_argv[] = {const_cast<char*>(VMM_FD_IMPORT_HELPER_PATH),
                              const_cast<char*>(fd_arg.c_str()), nullptr};
  posix_spawn_file_actions_t actions;
  ASSERT_EQ(::posix_spawn_file_actions_init(&actions), 0);
  ASSERT_EQ(::posix_spawn_file_actions_addclose(&actions, payload_pipe[1]), 0);
  pid_t child = -1;
  const int spawn_result = posix_spawn(&child, VMM_FD_IMPORT_HELPER_PATH,
                                       &actions, nullptr, child_argv, environ);
  ASSERT_EQ(::posix_spawn_file_actions_destroy(&actions), 0);
  ASSERT_EQ(spawn_result, 0);
  ::close(payload_pipe[0]);

  bool wrote_payload = false;
  {
    ScopedSigpipeIgnore sigpipe_ignore;
    wrote_payload = write_all(payload_pipe[1], &payload, sizeof(payload));
  }
  ::close(payload_pipe[1]);

  int status = 0;
  EXPECT_TRUE(wrote_payload);
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
}
