// Fallback (non-CUDA) implementations
#include "ros2_cuda_ipc_core/cuda_support.hpp"

namespace ros2_cuda_ipc_core {

bool cuda_is_available() { return false; }

void* cuda_allocate(std::size_t /*bytes*/) { return nullptr; }

bool cuda_free(void* /*ptr*/) { return false; }

bool cuda_ipc_get_mem_handle(void* /*device_ptr*/,
                             CudaIpcMemHandle* /*out_handle*/) {
  return false;
}

void* cuda_ipc_open_mem_handle(const CudaIpcMemHandle& /*handle*/) {
  return nullptr;
}

bool cuda_ipc_close_mem_handle(void* /*device_ptr*/) { return false; }

void* cuda_event_create() { return nullptr; }

bool cuda_event_destroy(void* /*evt*/) { return false; }

bool cuda_event_record(void* /*evt*/) { return false; }

bool cuda_event_get_ipc_handle(void* /*evt*/,
                               CudaIpcEventHandle* /*out_handle*/) {
  return false;
}

void* cuda_ipc_open_event_handle(const CudaIpcEventHandle& /*handle*/) {
  return nullptr;
}

bool cuda_event_query(void* /*evt*/) { return false; }

void* cuda_stream_create() { return nullptr; }
bool cuda_stream_destroy(void* /*stream*/) { return false; }
bool cuda_stream_wait_event(void* /*stream*/, void* /*evt*/) { return false; }
bool cuda_stream_synchronize(void* /*stream*/) { return false; }

}  // namespace ros2_cuda_ipc_core
