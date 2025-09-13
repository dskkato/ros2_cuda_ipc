#include <cuda_runtime.h>

#include <cstring>

#include "ros2_cuda_ipc_core/cuda_support.hpp"

namespace ros2_cuda_ipc_core {

bool cuda_is_available() {
  int count = 0;
  auto err = cudaGetDeviceCount(&count);

  if (err == cudaSuccess) {
    return (count > 0);
  } else {
    return false;
  }
}

void* cuda_allocate(std::size_t bytes) {
  void* ptr = nullptr;
  auto err = cudaMalloc(&ptr, bytes);
  if (err != cudaSuccess) {
    return nullptr;
  }
  return ptr;
}

bool cuda_free(void* ptr) {
  if (!ptr) return false;
  auto err = cudaFree(ptr);
  return err == cudaSuccess;
}

bool cuda_ipc_get_mem_handle(void* device_ptr, CudaIpcMemHandle* out_handle) {
  if (!device_ptr || !out_handle) return false;
  cudaIpcMemHandle_t h{};
  auto err = cudaIpcGetMemHandle(&h, device_ptr);
  if (err != cudaSuccess) return false;
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
  auto err = cudaIpcOpenMemHandle(&ptr, h, cudaIpcMemLazyEnablePeerAccess);
  if (err != cudaSuccess) return nullptr;
  return ptr;
}

bool cuda_ipc_close_mem_handle(void* device_ptr) {
  if (!device_ptr) return false;
  auto err = cudaIpcCloseMemHandle(device_ptr);
  return err == cudaSuccess;
}

void* cuda_event_create() {
  cudaEvent_t evt = nullptr;
  auto err = cudaEventCreateWithFlags(
      &evt, cudaEventDisableTiming | cudaEventInterprocess);
  if (err != cudaSuccess) return nullptr;
  return reinterpret_cast<void*>(evt);
}

bool cuda_event_destroy(void* evt) {
  if (!evt) return false;
  auto err = cudaEventDestroy(reinterpret_cast<cudaEvent_t>(evt));
  return err == cudaSuccess;
}

bool cuda_event_record(void* evt) {
  if (!evt) return false;
  auto err = cudaEventRecord(reinterpret_cast<cudaEvent_t>(evt), /*stream=*/0);
  return err == cudaSuccess;
}

bool cuda_event_get_ipc_handle(void* evt, CudaIpcEventHandle* out_handle) {
  if (!evt || !out_handle) return false;
  cudaIpcEventHandle_t h{};
  auto err = cudaIpcGetEventHandle(&h, reinterpret_cast<cudaEvent_t>(evt));
  if (err != cudaSuccess) return false;
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
  auto err = cudaIpcOpenEventHandle(&evt, h);
  if (err != cudaSuccess) return nullptr;
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
  auto err = cudaEventRecord(reinterpret_cast<cudaEvent_t>(evt), s);
  return err == cudaSuccess;
}

void* cuda_stream_create() {
  cudaStream_t s = nullptr;
  auto err = cudaStreamCreateWithFlags(&s, cudaStreamNonBlocking);
  if (err != cudaSuccess) return nullptr;
  return reinterpret_cast<void*>(s);
}

bool cuda_stream_destroy(void* stream) {
  if (!stream) return false;
  auto err = cudaStreamDestroy(reinterpret_cast<cudaStream_t>(stream));
  return err == cudaSuccess;
}

bool cuda_stream_wait_event(void* stream, void* evt) {
  if (!stream || !evt) return false;
  auto err = cudaStreamWaitEvent(reinterpret_cast<cudaStream_t>(stream),
                                 reinterpret_cast<cudaEvent_t>(evt), 0);
  return err == cudaSuccess;
}

bool cuda_stream_synchronize(void* stream) {
  if (!stream) return false;
  auto err = cudaStreamSynchronize(reinterpret_cast<cudaStream_t>(stream));
  return err == cudaSuccess;
}
}  // namespace ros2_cuda_ipc_core
