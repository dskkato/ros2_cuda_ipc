// vmm_fd_producer.cu
// VMM-FD producer using Driver API Virtual Memory Management with POSIX FD
// export
#include <cuda.h>
#include <cuda_runtime.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

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

__global__ void fill_kernel(int* p, int n, int v) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) p[i] = v;
}

struct MsgHeader {
  int dev;
  size_t logical_bytes;
  size_t alloc_bytes;
};

static int make_server_socket(const char* path) {
  int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) {
    perror("socket");
    exit(2);
  }

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);

  unlink(path);
  if (bind(fd, (sockaddr*)&addr, sizeof(addr)) != 0) {
    perror("bind");
    close(fd);
    exit(2);
  }
  if (listen(fd, 1) != 0) {
    perror("listen");
    close(fd);
    exit(2);
  }
  return fd;
}

static int accept_client(int server_fd) {
  int c = accept(server_fd, nullptr, nullptr);
  if (c < 0) {
    perror("accept");
    exit(2);
  }
  return c;
}

static void send_fd_with_header(int sock, int fd_to_send,
                                const MsgHeader& hdr) {
  // send header bytes + fd via SCM_RIGHTS
  struct msghdr msg{};
  struct iovec iov{};
  iov.iov_base = (void*)&hdr;
  iov.iov_len = sizeof(hdr);
  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;

  char cmsgbuf[CMSG_SPACE(sizeof(int))];
  std::memset(cmsgbuf, 0, sizeof(cmsgbuf));
  msg.msg_control = cmsgbuf;
  msg.msg_controllen = sizeof(cmsgbuf);

  struct cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
  cmsg->cmsg_level = SOL_SOCKET;
  cmsg->cmsg_type = SCM_RIGHTS;
  cmsg->cmsg_len = CMSG_LEN(sizeof(int));
  std::memcpy(CMSG_DATA(cmsg), &fd_to_send, sizeof(int));

  msg.msg_controllen = cmsg->cmsg_len;

  if (sendmsg(sock, &msg, 0) < 0) {
    perror("sendmsg");
    exit(3);
  }
}

static void recv_done(int sock) {
  char b = 0;
  ssize_t r = read(sock, &b, 1);
  if (r != 1) {
    fprintf(stderr, "[producer] failed to read done byte, got %zd\n", r);
    exit(4);
  }
  printf("[producer] got done signal: '%c'\n", b);
}

int main(int argc, char** argv) {
  setbuf(stdout, nullptr);
  setbuf(stderr, nullptr);

  const char* sock_path = "/tmp/cuda_fd_share.sock";
  int dev = 0;
  if (argc >= 2) dev = std::atoi(argv[1]);

  // Initialize runtime first (creates/uses primary context typically)
  CUDA_CHECK(cudaSetDevice(dev));
  CUDA_CHECK(cudaFree(nullptr));

  CU_CHECK(cuInit(0));
  CUdevice cu_dev;
  CU_CHECK(cuDeviceGet(&cu_dev, dev));

  // Ensure we operate on primary context to keep runtime and driver consistent
  CUcontext ctx;
  CU_CHECK(cuDevicePrimaryCtxRetain(&ctx, cu_dev));
  CU_CHECK(cuCtxSetCurrent(ctx));

  cudaDeviceProp rprop{};
  CUDA_CHECK(cudaGetDeviceProperties(&rprop, dev));
  printf("[producer] device=%d %s (integrated=%d)\n", dev, rprop.name,
         rprop.integrated);

  // Setup server socket
  int sfd = make_server_socket(sock_path);
  printf("[producer] listening on %s\n", sock_path);

  // Allocate using Driver VMM + export as POSIX FD
  const int N = 1024;
  const size_t bytes = N * sizeof(int);

  size_t logical_bytes = bytes;
  CUmemAllocationProp allocProp{};
  allocProp.type = CU_MEM_ALLOCATION_TYPE_PINNED;
  allocProp.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
  allocProp.location.id = dev;
  allocProp.requestedHandleTypes = CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR;

  size_t gran = 0;
  CU_CHECK(cuMemGetAllocationGranularity(&gran, &allocProp,
                                         CU_MEM_ALLOC_GRANULARITY_MINIMUM));
  size_t alloc_bytes = ((logical_bytes + gran - 1) / gran) * gran;

  printf("[producer] logical_bytes=%zu, granularity=%zu, alloc_bytes=%zu\n",
         logical_bytes, gran, alloc_bytes);

  CUmemGenericAllocationHandle allocHandle;
  CU_CHECK(cuMemCreate(&allocHandle, alloc_bytes, &allocProp, 0));
  printf("[producer] cuMemCreate OK\n");

  // Reserve VA and map
  CUdeviceptr dptr = 0;
  CU_CHECK(cuMemAddressReserve(&dptr, alloc_bytes, 0 /*align*/, 0, 0));
  CU_CHECK(cuMemMap(dptr, alloc_bytes, 0, allocHandle, 0));

  CUmemAccessDesc access{};
  access.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
  access.location.id = dev;
  access.flags = CU_MEM_ACCESS_FLAGS_PROT_READWRITE;
  CU_CHECK(cuMemSetAccess(dptr, alloc_bytes, &access, 1));
  printf("[producer] cuMemMap + cuMemSetAccess OK (dptr=0x%llx)\n",
         (unsigned long long)dptr);

  // Fill via runtime kernel (same primary context)
  fill_kernel<<<(N + 255) / 256, 256>>>((int*)dptr, N, 123);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());
  printf("[producer] filled buffer with 123\n");

  // Export to POSIX FD
  int share_fd = -1;
  CU_CHECK(cuMemExportToShareableHandle(
      &share_fd, allocHandle, CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR, 0));
  if (share_fd < 0) {
    fprintf(stderr, "[producer] export returned invalid fd\n");
    exit(5);
  }
  printf("[producer] cuMemExportToShareableHandle OK (fd=%d)\n", share_fd);

  // Accept client and send fd + header
  int cfd = accept_client(sfd);
  printf("[producer] consumer connected\n");

  MsgHeader hdr{};
  hdr.dev = dev;
  hdr.logical_bytes = logical_bytes;
  hdr.alloc_bytes = alloc_bytes;
  send_fd_with_header(cfd, share_fd, hdr);
  printf("[producer] sent fd + header (bytes=%zu)\n", bytes);

  // Wait for done
  recv_done(cfd);

  // Verify result (expect +7 => 130 if consumer succeeded)
  int host[4] = {0, 0, 0, 0};
  CUDA_CHECK(
      cudaMemcpy(host, (void*)dptr, sizeof(host), cudaMemcpyDeviceToHost));
  printf("[producer] first 4 ints after consumer: %d %d %d %d\n", host[0],
         host[1], host[2], host[3]);

  // Cleanup
  close(cfd);
  close(sfd);
  close(share_fd);

  CU_CHECK(cuMemUnmap(dptr, alloc_bytes));
  CU_CHECK(cuMemAddressFree(dptr, alloc_bytes));
  CU_CHECK(cuMemRelease(allocHandle));

  CU_CHECK(cuDevicePrimaryCtxRelease(cu_dev));
  unlink(sock_path);

  printf("[producer] DONE\n");
  return 0;
}
