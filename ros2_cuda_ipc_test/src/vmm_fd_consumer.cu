// vmm_fd_consumer.cu
// VMM-FD consumer using Driver API Virtual Memory Management with POSIX FD
// import
#include <cuda.h>
#include <cuda_runtime.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#define CU_CHECK(call)                                                  \
  do {                                                                  \
    CUresult _e = (call);                                               \
    if (_e != CUDA_SUCCESS) {                                           \
      const char* name = nullptr;                                       \
      const char* str = nullptr;                                        \
      cuGetErrorName(_e, &name);                                        \
      cuGetErrorString(_e, &str);                                       \
      fprintf(stderr, "[CU ERROR] %s:%d: %s: %s\n", __FILE__, __LINE__, \
              name ? name : "?", str ? str : "?");                      \
      exit(1);                                                          \
    }                                                                   \
  } while (0)

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

struct MsgHeader {
  int dev;
  size_t logical_bytes;  // 4096
  size_t alloc_bytes;    // Example: 65536 allocated bytes
};

static int connect_socket(const char* path) {
  int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) {
    perror("socket");
    exit(2);
  }

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);

  if (connect(fd, (sockaddr*)&addr, sizeof(addr)) != 0) {
    perror("connect");
    exit(2);
  }
  return fd;
}

static void recv_fd_and_header(int sock, int* out_fd, MsgHeader* out_hdr) {
  struct msghdr msg {};
  struct iovec iov {};
  iov.iov_base = out_hdr;
  iov.iov_len = sizeof(*out_hdr);
  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;

  char cmsgbuf[CMSG_SPACE(sizeof(int))];
  std::memset(cmsgbuf, 0, sizeof(cmsgbuf));
  msg.msg_control = cmsgbuf;
  msg.msg_controllen = sizeof(cmsgbuf);

  ssize_t n = recvmsg(sock, &msg, 0);
  if (n < 0) {
    perror("recvmsg");
    exit(3);
  }
  if ((size_t)n != sizeof(*out_hdr)) {
    fprintf(stderr, "[consumer] recv header size mismatch: got %zd\n", n);
    exit(3);
  }

  int fd = -1;
  struct cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
  if (!cmsg || cmsg->cmsg_type != SCM_RIGHTS) {
    fprintf(stderr, "[consumer] no SCM_RIGHTS fd received\n");
    exit(3);
  }
  std::memcpy(&fd, CMSG_DATA(cmsg), sizeof(int));
  if (fd < 0) {
    fprintf(stderr, "[consumer] received invalid fd\n");
    exit(3);
  }
  *out_fd = fd;
}

static void send_done(int sock, char code) {
  if (write(sock, &code, 1) != 1) {
    fprintf(stderr, "[consumer] failed to send done byte\n");
  }
}

int main() {
  setbuf(stdout, nullptr);
  setbuf(stderr, nullptr);

  const char* sock_path = "/tmp/cuda_fd_share.sock";
  int sock = connect_socket(sock_path);
  printf("[consumer] connected to %s\n", sock_path);

  int recv_fd = -1;
  MsgHeader hdr{};
  recv_fd_and_header(sock, &recv_fd, &hdr);
  printf("[consumer] received fd=%d dev=%d bytes=%zu\n", recv_fd, hdr.dev,
         hdr.alloc_bytes);

  // Initialize runtime / primary context
  CUDA_CHECK(cudaSetDevice(hdr.dev));
  CUDA_CHECK(cudaFree(nullptr));

  CU_CHECK(cuInit(0));
  CUdevice cu_dev;
  CU_CHECK(cuDeviceGet(&cu_dev, hdr.dev));

  CUcontext ctx;
  CU_CHECK(cuDevicePrimaryCtxRetain(&ctx, cu_dev));
  CU_CHECK(cuCtxSetCurrent(ctx));

  cudaDeviceProp rprop{};
  CUDA_CHECK(cudaGetDeviceProperties(&rprop, hdr.dev));
  printf("[consumer] device=%d %s (integrated=%d)\n", hdr.dev, rprop.name,
         rprop.integrated);

  // Import from POSIX FD
  CUmemGenericAllocationHandle allocHandle;
  CUresult ie =
      cuMemImportFromShareableHandle(&allocHandle, (void*)(intptr_t)recv_fd,
                                     CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR);
  if (ie != CUDA_SUCCESS) {
    const char* name = nullptr;
    const char* str = nullptr;
    cuGetErrorName(ie, &name);
    cuGetErrorString(ie, &str);
    fprintf(stderr,
            "[consumer] cuMemImportFromShareableHandle FAILED: %s: %s\n",
            name ? name : "?", str ? str : "?");
    send_done(sock, 'F');
    close(recv_fd);
    close(sock);
    return 10;
  }
  printf("[consumer] cuMemImportFromShareableHandle OK\n");

  // After import, receiver owns its fd; can close it.
  close(recv_fd);

  // Reserve VA and map
  CUdeviceptr dptr = 0;
  CU_CHECK(cuMemAddressReserve(&dptr, hdr.alloc_bytes, 0, 0, 0));
  CU_CHECK(cuMemMap(dptr, hdr.alloc_bytes, 0, allocHandle, 0));

  CUmemAccessDesc access{};
  access.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
  access.location.id = hdr.dev;
  access.flags = CU_MEM_ACCESS_FLAGS_PROT_READWRITE;
  CU_CHECK(cuMemSetAccess(dptr, hdr.alloc_bytes, &access, 1));
  printf("[consumer] mapped imported memory (dptr=0x%llx)\n",
         (unsigned long long)dptr);

  // Verify read
  int host_before[4] = {0, 0, 0, 0};
  CUDA_CHECK(cudaMemcpy(host_before, (void*)dptr, sizeof(host_before),
                        cudaMemcpyDeviceToHost));
  printf("[consumer] first 4 ints before: %d %d %d %d\n", host_before[0],
         host_before[1], host_before[2], host_before[3]);

  // Modify
  const int N = (int)(hdr.logical_bytes / sizeof(int));
  add_kernel<<<(N + 255) / 256, 256>>>((int*)dptr, N, 7);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());

  int host_after[4] = {0, 0, 0, 0};
  CUDA_CHECK(cudaMemcpy(host_after, (void*)dptr, sizeof(host_after),
                        cudaMemcpyDeviceToHost));
  printf("[consumer] first 4 ints after +7: %d %d %d %d\n", host_after[0],
         host_after[1], host_after[2], host_after[3]);

  // Cleanup mapping and handle
  CU_CHECK(cuMemUnmap(dptr, hdr.alloc_bytes));
  CU_CHECK(cuMemAddressFree(dptr, hdr.alloc_bytes));
  CU_CHECK(cuMemRelease(allocHandle));

  CU_CHECK(cuDevicePrimaryCtxRelease(cu_dev));

  send_done(sock, 'D');
  close(sock);

  printf("[consumer] DONE\n");
  return 0;
}
