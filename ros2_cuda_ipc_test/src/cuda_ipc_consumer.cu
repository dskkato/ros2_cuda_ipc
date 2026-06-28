// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

// cuda_ipc_consumer.cu
// CUDA IPC consumer using cudaIpcOpenMemHandle (Runtime API)
#include <cuda_runtime.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#define CUDA_CHECK(call)                                                   \
  do {                                                                     \
    cudaError_t _e = (call);                                               \
    if (_e != cudaSuccess) {                                               \
      fprintf(stderr, "[CUDA ERROR] %s:%d: %s (%d)\n", __FILE__, __LINE__, \
              cudaGetErrorString(_e), (int)_e);                            \
      exit(1);                                                             \
    }                                                                      \
  } while (0)

__global__ void add_kernel(int* p, int n, int v) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) p[i] += v;
}

struct IpcMsg {
  int dev;
  size_t bytes;
  cudaIpcMemHandle_t handle;
};

int main() {
  const char* fifo_handle = "/tmp/cuda_ipc_handle.fifo";
  const char* fifo_done = "/tmp/cuda_ipc_done.fifo";

  // receive handle
  int fdr = open(fifo_handle, O_RDONLY);
  if (fdr < 0) {
    perror("[consumer] open fifo_handle");
    return 2;
  }

  IpcMsg msg{};
  ssize_t r = read(fdr, &msg, sizeof(msg));
  close(fdr);
  if (r != (ssize_t)sizeof(msg)) {
    fprintf(stderr, "[consumer] read msg failed: got %zd\n", r);
    return 3;
  }
  printf("[consumer] received IPC handle (dev=%d, bytes=%zu)\n", msg.dev,
         msg.bytes);

  CUDA_CHECK(cudaSetDevice(msg.dev));
  CUDA_CHECK(cudaFree(nullptr));  // init runtime

  cudaDeviceProp prop{};
  CUDA_CHECK(cudaGetDeviceProperties(&prop, msg.dev));
  printf("[consumer] device=%d %s (integrated=%d)\n", msg.dev, prop.name,
         prop.integrated);

  // open handle
  void* opened = nullptr;
  cudaError_t e =
      cudaIpcOpenMemHandle(&opened, msg.handle, cudaIpcMemLazyEnablePeerAccess);
  if (e != cudaSuccess) {
    fprintf(stderr, "[consumer] cudaIpcOpenMemHandle FAILED: %s (%d)\n",
            cudaGetErrorString(e), (int)e);
    // notify producer even on failure (optional)
    int fdw_fail = open(fifo_done, O_WRONLY);
    if (fdw_fail >= 0) {
      char d = 'F';
      write(fdw_fail, &d, 1);
      close(fdw_fail);
    }
    return 10;
  }
  printf("[consumer] cudaIpcOpenMemHandle OK (opened=%p)\n", opened);

  // read first ints
  int host[4] = {0, 0, 0, 0};
  CUDA_CHECK(cudaMemcpy(host, opened, sizeof(host), cudaMemcpyDeviceToHost));
  printf("[consumer] opened first 4 ints (before): %d %d %d %d\n", host[0],
         host[1], host[2], host[3]);

  // modify
  const int N = (int)(msg.bytes / sizeof(int));
  add_kernel<<<(N + 255) / 256, 256>>>((int*)opened, N, 7);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());

  CUDA_CHECK(cudaMemcpy(host, opened, sizeof(host), cudaMemcpyDeviceToHost));
  printf("[consumer] opened first 4 ints (after +7): %d %d %d %d\n", host[0],
         host[1], host[2], host[3]);

  CUDA_CHECK(cudaIpcCloseMemHandle(opened));
  printf("[consumer] cudaIpcCloseMemHandle OK\n");

  // notify producer done
  int fdw = open(fifo_done, O_WRONLY);
  if (fdw < 0) {
    perror("[consumer] open fifo_done");
    return 4;
  }
  char done = 'D';
  if (write(fdw, &done, 1) != 1) {
    fprintf(stderr, "[consumer] write done failed\n");
    close(fdw);
    return 5;
  }
  close(fdw);
  printf("[consumer] sent done signal\n");

  printf("[consumer] DONE\n");
  return 0;
}
