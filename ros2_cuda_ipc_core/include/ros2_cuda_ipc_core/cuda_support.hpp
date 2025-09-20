#pragma once

#include <cuda_runtime_api.h>

#include <mutex>

namespace ros2_cuda_ipc_core {

// Lightweight indirection over CUDA runtime entry points so tests can
// substitute stub implementations. All functions are thread-safe.
class CudaSupport {
 public:
  struct Hooks {
    cudaError_t (*open_mem_handle)(void **, const cudaIpcMemHandle_t &,
                                   unsigned int) = nullptr;
    cudaError_t (*close_mem_handle)(void *) = nullptr;
    cudaError_t (*open_event_handle)(cudaEvent_t *,
                                     const cudaIpcEventHandle_t &) = nullptr;
    cudaError_t (*destroy_event)(cudaEvent_t) = nullptr;
    cudaError_t (*stream_wait_event)(cudaStream_t, cudaEvent_t,
                                     unsigned int) = nullptr;
    const char *(*get_error_string)(cudaError_t) = nullptr;
  };

  static void set_hooks(const Hooks &hooks);
  static void reset_hooks();

  static cudaError_t open_ipc_memory(void **ptr,
                                     const cudaIpcMemHandle_t &handle,
                                     unsigned int flags);
  static cudaError_t close_ipc_memory(void *ptr);
  static cudaError_t open_ipc_event(cudaEvent_t *evt,
                                    const cudaIpcEventHandle_t &handle);
  static cudaError_t destroy_event(cudaEvent_t evt);
  static cudaError_t stream_wait_event(cudaStream_t stream, cudaEvent_t evt,
                                       unsigned int flags);
  static const char *error_string(cudaError_t err);

 private:
  static Hooks &hooks();
  static std::mutex &mutex();
};

}  // namespace ros2_cuda_ipc_core
