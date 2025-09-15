#include "ros2_cuda_ipc_core/zero_copy_subscriber.hpp"

namespace ros2_cuda_ipc_core {

ZeroCopySubscriber::ZeroCopySubscriber(cudaStream_t stream) {
  if (stream) {
    stream_ = stream;
    own_stream_ = false;
  } else if (cuda_is_available()) {
    stream_ = cuda_stream_create();
    own_stream_ = (stream_ != nullptr);
  }
}

ZeroCopySubscriber::~ZeroCopySubscriber() {
  if (own_stream_ && stream_) {
    (void)cuda_stream_destroy(stream_);
    stream_ = nullptr;
  }
}

}  // namespace ros2_cuda_ipc_core
