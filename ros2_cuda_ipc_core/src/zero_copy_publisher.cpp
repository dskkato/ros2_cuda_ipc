#include "ros2_cuda_ipc_core/zero_copy_publisher.hpp"

#include <cstring>

namespace ros2_cuda_ipc_core {

ZeroCopyPublisher::ZeroCopyPublisher(const PoolOptions& opts,
                                     int lease_timeout_ms,
                                     std::string shm_owner)
    : pool_(opts),
      lease_mgr_(pool_, lease_timeout_ms),
      owner_(sanitize_shm_owner(shm_owner)) {
  producer_stream_ = opts.producer_stream;
}

ZeroCopyPublisher::~ZeroCopyPublisher() { cleanup(); }

void ZeroCopyPublisher::set_owner(std::string owner) {
  owner_ = sanitize_shm_owner(owner);
}

void ZeroCopyPublisher::set_timeout_ms(int ms) {
  lease_mgr_.set_timeout_ms(ms);
}

int ZeroCopyPublisher::tick() { return lease_mgr_.tick(); }

void ZeroCopyPublisher::cleanup() { lease_mgr_.cleanup(); }

}  // namespace ros2_cuda_ipc_core
