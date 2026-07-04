// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

// cuda_ipc_producer.cu
// CUDA IPC producer using cudaIpcGetMemHandle (Runtime API)
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

__global__ void fill_kernel(int* p, int n, int v) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) p[i] = v;
}

struct IpcMsg {
  int dev;
  size_t bytes;
  cudaIpcMemHandle_t handle;
};

static void ensure_fifo(const char* path) {
  // recreate FIFO to avoid stale state
  unlink(path);
  if (mkfifo(path, 0666) != 0) {
    perror("mkfifo");
    exit(2);
  }
}

int main(int argc, char** argv) {
  const char* fifo_handle = "/tmp/cuda_ipc_handle.fifo";
  const char* fifo_done = "/tmp/cuda_ipc_done.fifo";

  int dev = 0;
  if (argc >= 2) dev = atoi(argv[1]);
  CUDA_CHECK(cudaSetDevice(dev));
  CUDA_CHECK(cudaFree(nullptr));  // init runtime

  cudaDeviceProp prop{};
  CUDA_CHECK(cudaGetDeviceProperties(&prop, dev));
  printf("[producer] device=%d %s (integrated=%d)\n", dev, prop.name,
         prop.integrated);

  ensure_fifo(fifo_handle);
  ensure_fifo(fifo_done);

  const int N = 1024;
  const size_t bytes = N * sizeof(int);

  int* buf = nullptr;
  CUDA_CHECK(cudaMalloc(&buf, bytes));

  fill_kernel<<<(N + 255) / 256, 256>>>(buf, N, 123);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());

  cudaIpcMemHandle_t h{};
  cudaError_t e = cudaIpcGetMemHandle(&h, (void*)buf);
  if (e != cudaSuccess) {
    fprintf(stderr, "[producer] cudaIpcGetMemHandle FAILED: %s (%d)\n",
            cudaGetErrorString(e), (int)e);
    CUDA_CHECK(cudaFree(buf));
    return 10;
  }
  printf("[producer] cudaIpcGetMemHandle OK\n");

  // send handle
  IpcMsg msg{};
  msg.dev = dev;
  msg.bytes = bytes;
  msg.handle = h;

  int fdw = open(fifo_handle, O_WRONLY);
  if (fdw < 0) {
    perror("[producer] open fifo_handle");
    return 3;
  }

  ssize_t w = write(fdw, &msg, sizeof(msg));
  if (w != (ssize_t)sizeof(msg)) {
    fprintf(stderr, "[producer] write msg failed: wrote %zd\n", w);
    close(fdw);
    return 4;
  }
  close(fdw);
  printf("[producer] sent IPC handle (%zu bytes)\n", sizeof(msg));

  // wait done signal
  int fdr = open(fifo_done, O_RDONLY);
  if (fdr < 0) {
    perror("[producer] open fifo_done");
    return 5;
  }

  char done = 0;
  ssize_t r = read(fdr, &done, 1);
  close(fdr);
  if (r != 1) {
    fprintf(stderr, "[producer] read done failed: got %zd\n", r);
    return 6;
  }
  printf("[producer] got done signal from consumer\n");

  // verify buffer content changed (+7 expected => 130)
  int host[4] = {0, 0, 0, 0};
  CUDA_CHECK(cudaMemcpy(host, buf, sizeof(host), cudaMemcpyDeviceToHost));
  printf("[producer] buf first 4 ints after consumer: %d %d %d %d\n", host[0],
         host[1], host[2], host[3]);

  CUDA_CHECK(cudaFree(buf));
  printf("[producer] DONE\n");
  return 0;
}
