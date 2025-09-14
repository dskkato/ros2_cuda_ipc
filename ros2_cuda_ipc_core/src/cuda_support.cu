#include <cuda_runtime.h>

#include <cstdio>
#include <cstring>
#include <experimental/source_location>
#include <memory>
#include <stdexcept>
#include <string>

#include "ros2_cuda_ipc_core/cuda_support.hpp"

namespace ros2_cuda_ipc_core {

inline void check_cuda_error(
    cudaError_t err, const std::experimental::source_location& location =
                         std::experimental::source_location::current()) {
  if (err != cudaSuccess) {
    std::fprintf(stderr, "CUDA error at %s:%u in %s(): %s\n",
                 location.file_name(), location.line(),
                 location.function_name(), cudaGetErrorString(err));
    throw std::runtime_error("CUDA error");
  }
}

bool cuda_is_available() {
  int count = 0;
  check_cuda_error(cudaGetDeviceCount(&count));

  return (count > 0);
}

void* cuda_allocate(std::size_t bytes) {
  void* ptr = nullptr;
  check_cuda_error(cudaMalloc(&ptr, bytes));
  return ptr;
}

bool cuda_free(void* ptr) {
  if (!ptr) return false;
  check_cuda_error(cudaFree(ptr));
  return true;
}

bool cuda_ipc_get_mem_handle(void* device_ptr, CudaIpcMemHandle* out_handle) {
  if (!device_ptr || !out_handle) return false;
  cudaIpcMemHandle_t h{};
  check_cuda_error(cudaIpcGetMemHandle(&h, device_ptr));
  static_assert(sizeof(CudaIpcMemHandle) == sizeof(cudaIpcMemHandle_t),
                "IPC handle size mismatch");
  std::memcpy(out_handle, &h, sizeof(h));
  return true;
}

void* cuda_ipc_open_mem_handle(const CudaIpcMemHandle& handle) {
  cudaIpcMemHandle_t h{};
  static_assert(sizeof(CudaIpcMemHandle) == sizeof(cudaIpcMemHandle_t),
                "IPC handle size mismatch");
  std::memcpy(&h, &handle, sizeof(h));
  void* ptr = nullptr;
  check_cuda_error(
      cudaIpcOpenMemHandle(&ptr, h, cudaIpcMemLazyEnablePeerAccess));
  return ptr;
}

bool cuda_ipc_close_mem_handle(void* device_ptr) {
  if (!device_ptr) return false;
  check_cuda_error(cudaIpcCloseMemHandle(device_ptr));
  return true;
}

void* cuda_event_create() {
  cudaEvent_t evt = nullptr;
  check_cuda_error(cudaEventCreateWithFlags(
      &evt, cudaEventDisableTiming | cudaEventInterprocess));
  return reinterpret_cast<void*>(evt);
}

bool cuda_event_destroy(void* evt) {
  if (!evt) return false;
  check_cuda_error(cudaEventDestroy(reinterpret_cast<cudaEvent_t>(evt)));
  return true;
}

bool cuda_event_record(void* evt) {
  if (!evt) return false;
  check_cuda_error(
      cudaEventRecord(reinterpret_cast<cudaEvent_t>(evt), /*stream=*/0));
  return true;
}

bool cuda_event_get_ipc_handle(void* evt, CudaIpcEventHandle* out_handle) {
  if (!evt || !out_handle) return false;
  cudaIpcEventHandle_t h{};
  check_cuda_error(
      cudaIpcGetEventHandle(&h, reinterpret_cast<cudaEvent_t>(evt)));
  static_assert(sizeof(CudaIpcEventHandle) == sizeof(cudaIpcEventHandle_t),
                "IPC event handle size mismatch");
  std::memcpy(out_handle, &h, sizeof(h));
  return true;
}

void* cuda_ipc_open_event_handle(const CudaIpcEventHandle& handle) {
  cudaIpcEventHandle_t h{};
  static_assert(sizeof(CudaIpcEventHandle) == sizeof(cudaIpcEventHandle_t),
                "IPC event handle size mismatch");
  std::memcpy(&h, &handle, sizeof(h));
  cudaEvent_t evt = nullptr;
  check_cuda_error(cudaIpcOpenEventHandle(&evt, h));
  return reinterpret_cast<void*>(evt);
}

bool cuda_event_query(void* evt) {
  if (!evt) return false;
  auto err = cudaEventQuery(reinterpret_cast<cudaEvent_t>(evt));
  if (err == cudaSuccess) return true;
  if (err == cudaErrorNotReady) return false;
  return false;
}

bool cuda_event_record_on_stream(void* evt, void* stream) {
  if (!evt) return false;
  auto s = stream ? reinterpret_cast<cudaStream_t>(stream)
                  : static_cast<cudaStream_t>(0);
  check_cuda_error(cudaEventRecord(reinterpret_cast<cudaEvent_t>(evt), s));
  return true;
}

void* cuda_stream_create() {
  cudaStream_t s = nullptr;
  check_cuda_error(cudaStreamCreateWithFlags(&s, cudaStreamNonBlocking));
  return reinterpret_cast<void*>(s);
}

bool cuda_stream_destroy(void* stream) {
  if (!stream) return false;
  check_cuda_error(cudaStreamDestroy(reinterpret_cast<cudaStream_t>(stream)));
  return true;
}

bool cuda_stream_wait_event(void* stream, void* evt) {
  if (!stream || !evt) return false;
  check_cuda_error(cudaStreamWaitEvent(reinterpret_cast<cudaStream_t>(stream),
                                       reinterpret_cast<cudaEvent_t>(evt), 0));
  return true;
}

bool cuda_stream_synchronize(void* stream) {
  if (!stream) return false;
  check_cuda_error(
      cudaStreamSynchronize(reinterpret_cast<cudaStream_t>(stream)));
  return true;
}

std::string cuda_get_device_id_string() {
  int dev = 0;
  check_cuda_error(cudaGetDevice(&dev));
  // Try device UUID from properties if available
  cudaDeviceProp prop{};
  if (cudaGetDeviceProperties(&prop, dev) == cudaSuccess) {
#if defined(__CUDACC_VER_MAJOR__) && (__CUDACC_VER_MAJOR__ >= 11)
    // Many toolkits expose prop.uuid.bytes (16 bytes). Serialize to hex.
    const unsigned char* u = reinterpret_cast<const unsigned char*>(&prop.uuid);
    char buf[64];
    int off = 0;
    for (int i = 0; i < 16 && off + 2 < static_cast<int>(sizeof(buf)); ++i) {
      off += std::snprintf(buf + off, sizeof(buf) - off, "%02x", u[i]);
    }
    if (off > 0) return std::string(buf, buf + off);
#endif
  }
  // Fallback to PCI bus id string when UUID isn't available
  char busid[64] = {0};
  check_cuda_error(cudaDeviceGetPCIBusId(busid, sizeof(busid), dev));
  return std::string(busid);
}
}  // namespace ros2_cuda_ipc_core
