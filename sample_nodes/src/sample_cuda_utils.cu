#include <cuda_runtime.h>

namespace sample_nodes {

__global__ void busy_kernel(unsigned long long iters) {
  unsigned long long acc = 0;
  for (unsigned long long i = 0; i < iters; ++i) {
    acc += (i ^ (acc << 1));
  }
  if (threadIdx.x == 0) {
    volatile unsigned long long* sink = &acc;
    (void)*sink;
  }
}

bool cuda_simulate_work_ms(int ms, void* stream) {
  if (ms <= 0) return true;
  // Rough scaling factor; device-dependent, just for demo visibility
  // Increase if the GPU is very fast.
  const unsigned long long iters =
      static_cast<unsigned long long>(ms) * 200000ULL;
  cudaStream_t s = stream ? reinterpret_cast<cudaStream_t>(stream)
                          : static_cast<cudaStream_t>(0);
  busy_kernel<<<1, 1, 0, s>>>(iters);
  return cudaGetLastError() == cudaSuccess;
}

}  // namespace sample_nodes
