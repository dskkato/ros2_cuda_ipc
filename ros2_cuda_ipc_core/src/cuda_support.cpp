#include "ros2_cuda_ipc_core/cuda_support.hpp"

#include <utility>

namespace ros2_cuda_ipc_core {

namespace {

CudaSupport::Hooks &default_hooks() {
  static CudaSupport::Hooks hooks{
      /*open_mem_handle=*/[](void **dev_ptr, const cudaIpcMemHandle_t &handle,
                             unsigned int flags) {
        return cudaIpcOpenMemHandle(dev_ptr, handle, flags);
      },
      /*close_mem_handle=*/
      [](void *dev_ptr) { return cudaIpcCloseMemHandle(dev_ptr); },
      /*open_event_handle=*/
      [](cudaEvent_t *evt, const cudaIpcEventHandle_t &handle) {
        return cudaIpcOpenEventHandle(evt, handle);
      },
      /*destroy_event=*/[](cudaEvent_t evt) { return cudaEventDestroy(evt); },
      /*stream_wait_event=*/
      [](cudaStream_t stream, cudaEvent_t evt, unsigned int flags) {
        return cudaStreamWaitEvent(stream, evt, flags);
      },
      /*get_error_string=*/
      [](cudaError_t err) { return cudaGetErrorString(err); }};
  return hooks;
}

}  // namespace

void CudaSupport::set_hooks(const Hooks &hooks_in) {
  std::lock_guard<std::mutex> lock(mutex());
  hooks() = hooks_in;
}

void CudaSupport::reset_hooks() {
  std::lock_guard<std::mutex> lock(mutex());
  hooks() = default_hooks();
}

cudaError_t CudaSupport::open_ipc_memory(void **ptr,
                                         const cudaIpcMemHandle_t &handle,
                                         unsigned int flags) {
  return hooks().open_mem_handle ? hooks().open_mem_handle(ptr, handle, flags)
                                 : cudaIpcOpenMemHandle(ptr, handle, flags);
}

cudaError_t CudaSupport::close_ipc_memory(void *ptr) {
  return hooks().close_mem_handle ? hooks().close_mem_handle(ptr)
                                  : cudaIpcCloseMemHandle(ptr);
}

cudaError_t CudaSupport::open_ipc_event(cudaEvent_t *evt,
                                        const cudaIpcEventHandle_t &handle) {
  return hooks().open_event_handle ? hooks().open_event_handle(evt, handle)
                                   : cudaIpcOpenEventHandle(evt, handle);
}

cudaError_t CudaSupport::destroy_event(cudaEvent_t evt) {
  return hooks().destroy_event ? hooks().destroy_event(evt)
                               : cudaEventDestroy(evt);
}

cudaError_t CudaSupport::stream_wait_event(cudaStream_t stream, cudaEvent_t evt,
                                           unsigned int flags) {
  return hooks().stream_wait_event
             ? hooks().stream_wait_event(stream, evt, flags)
             : cudaStreamWaitEvent(stream, evt, flags);
}

const char *CudaSupport::error_string(cudaError_t err) {
  return hooks().get_error_string ? hooks().get_error_string(err)
                                  : cudaGetErrorString(err);
}

CudaSupport::Hooks &CudaSupport::hooks() {
  static Hooks hooks = default_hooks();
  return hooks;
}

std::mutex &CudaSupport::mutex() {
  static std::mutex mtx;
  return mtx;
}

}  // namespace ros2_cuda_ipc_core
