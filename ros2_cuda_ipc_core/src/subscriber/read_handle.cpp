// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/subscriber/read_handle.hpp"

#include <rcutils/logging_macros.h>

#include <condition_variable>
#include <deque>
#include <mutex>
#include <new>
#include <thread>
#include <utility>

#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
#include "ros2_cuda_ipc_core/detail/read_handle_binding.hpp"
#include "ros2_cuda_ipc_core/detail/read_handle_factory.hpp"
#include "ros2_cuda_ipc_core/lease/lease_handle.hpp"

namespace ros2_cuda_ipc_core::subscriber {
namespace {

struct CompletionEvent {
  std::shared_ptr<ros2_cuda_ipc_core::detail::CudaDeviceContext> context;
  CUevent event = nullptr;

  ~CompletionEvent() noexcept {
    if (event == nullptr || !context) {
      return;
    }
    try {
      auto guard_result = context->push_current();
      if (!guard_result) {
        RCUTILS_LOG_ERROR_NAMED(
            "ros2_cuda_ipc_core.subscriber.read_handle",
            "Failed to activate CUDA context while destroying completion "
            "event: %s",
            guard_result.error().to_string().c_str());
        return;
      }
      auto guard = std::move(guard_result).value();
      const CUresult result = cuEventDestroy(event);
      if (result != CUDA_SUCCESS) {
        RCUTILS_LOG_ERROR_NAMED(
            "ros2_cuda_ipc_core.subscriber.read_handle",
            "cuEventDestroy failed for completion event: %s",
            ros2_cuda_ipc_core::detail::CudaDriverError(result)
                .to_string()
                .c_str());
      }
    } catch (...) {
      // Cleanup must not escape a noexcept boundary.
    }
    event = nullptr;
  }
};

struct DeferredItem {
  std::shared_ptr<CompletionEvent> completion;
  std::shared_ptr<const backend::ImportedResources> resource;
  std::unique_ptr<lease::LeaseHandle> lease;
};

class DeferredReleaseQueue {
 public:
  static DeferredReleaseQueue& instance() {
    // Intentionally leaked so the worker never races static destruction with
    // CUDA context teardown or Python interpreter shutdown.
    static DeferredReleaseQueue* queue = new DeferredReleaseQueue();
    return *queue;
  }

  void enqueue(DeferredItem item) noexcept {
    try {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        items_.emplace_back(std::move(item));
      }
      condition_.notify_one();
    } catch (...) {
      // Keeping the item alive is safer than releasing a lease before the
      // consumer stream has completed.  The queue is intentionally process
      // lifetime, so an allocation failure is recorded as a held item.
      RCUTILS_LOG_ERROR_NAMED(
          "ros2_cuda_ipc_core.subscriber.read_handle",
          "Failed to enqueue deferred GPU read release; retaining lease");
      auto* retained = new (std::nothrow) DeferredItem(std::move(item));
      if (retained == nullptr) {
        // There is no safe release path after the completion handoff.  Leak
        // the lease/resource on this exceptional allocation-failure path.
        (void)item.lease.release();
      }
    }
  }

  void retain_failed(DeferredItem item) noexcept {
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      failed_.emplace_back(std::move(item));
    } catch (...) {
      RCUTILS_LOG_ERROR_NAMED(
          "ros2_cuda_ipc_core.subscriber.read_handle",
          "Failed to retain failed GPU read release; lease remains held by "
          "the process-lifetime read state");
      // There is no safe release path after completion recording failed.
      // The item is intentionally leaked on this exceptional path.
      auto* retained = new (std::nothrow) DeferredItem(std::move(item));
      if (retained == nullptr) {
        (void)item.lease.release();
      }
    }
  }

 private:
  DeferredReleaseQueue() : worker_([this]() { run(); }) {}

  void run() noexcept {
    while (true) {
      DeferredItem item;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this]() { return !items_.empty(); });
        item = std::move(items_.front());
        items_.pop_front();
      }

      if (!item.completion || !item.completion->context ||
          item.completion->event == nullptr) {
        continue;
      }

      auto guard_result = item.completion->context->push_current();
      if (!guard_result) {
        RCUTILS_LOG_ERROR_NAMED(
            "ros2_cuda_ipc_core.subscriber.read_handle",
            "Failed to activate CUDA context while waiting for completion "
            "event: %s",
            guard_result.error().to_string().c_str());
        retain_failed(std::move(item));
        continue;
      }
      auto guard = std::move(guard_result).value();
      const CUresult result = cuEventSynchronize(item.completion->event);
      if (result != CUDA_SUCCESS) {
        RCUTILS_LOG_ERROR_NAMED(
            "ros2_cuda_ipc_core.subscriber.read_handle",
            "cuEventSynchronize failed; retaining publication lease: %s",
            ros2_cuda_ipc_core::detail::CudaDriverError(result)
                .to_string()
                .c_str());
        retain_failed(std::move(item));
        continue;
      }

      // The item is destroyed only after the event has completed.  Its
      // completion event is destroyed first, then the resource deleter and
      // lease handle release their handle-specific ownership.
      item.completion.reset();
      item.resource.reset();
      item.lease.reset();
    }
  }

  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<DeferredItem> items_;
  std::deque<DeferredItem> failed_;
  std::thread worker_;
};

std::shared_ptr<CompletionEvent> create_completion_event(
    const std::shared_ptr<ros2_cuda_ipc_core::detail::CudaDeviceContext>&
        context) {
  if (!context) {
    return {};
  }
  auto guard_result = context->push_current();
  if (!guard_result) {
    return {};
  }
  auto guard = std::move(guard_result).value();
  CUevent event = nullptr;
  if (cuEventCreate(&event, CU_EVENT_DISABLE_TIMING) != CUDA_SUCCESS) {
    return {};
  }
  try {
    auto result = std::make_shared<CompletionEvent>();
    result->context = context;
    result->event = event;
    return result;
  } catch (...) {
    // The event was created before allocating its owning state.  Preserve
    // the no-leak guarantee even on an exceptional allocation path.
    (void)cuEventDestroy(event);
    throw;
  }
}

bool validate_stream(
    const std::shared_ptr<const backend::ImportedResources>& resource,
    CUstream stream) noexcept {
  const auto context = resource ? resource->context : nullptr;
  if (!context) {
    return true;
  }
  const int expected_device = context->device_id();
  if (stream == CU_STREAM_LEGACY || stream == CU_STREAM_PER_THREAD) {
    auto guard_result = context->push_current();
    if (!guard_result) {
      return false;
    }
    auto guard = std::move(guard_result).value();
    CUdevice current_device = 0;
    return cuCtxGetDevice(&current_device) == CUDA_SUCCESS &&
           static_cast<int>(current_device) == expected_device;
  }

  CUcontext stream_context = nullptr;
  CUresult result = cuStreamGetCtx(stream, &stream_context);
  if (result != CUDA_SUCCESS) {
    return false;
  }
  auto guard_result =
      ros2_cuda_ipc_core::detail::CudaContextGuard::push(stream_context);
  if (!guard_result) {
    return false;
  }
  auto guard = std::move(guard_result).value();
  CUdevice stream_device = 0;
  result = cuCtxGetDevice(&stream_device);
  return result == CUDA_SUCCESS &&
         static_cast<int>(stream_device) == expected_device;
}

bool enqueue_ready_event(
    const std::shared_ptr<const backend::ImportedResources>& resource,
    CUstream stream) noexcept {
  if (!validate_stream(resource, stream)) {
    return false;
  }
  const CUevent event = resource ? resource->event : nullptr;
  if (event == nullptr) {
    return true;
  }
  const auto context = resource->context;
  if (context) {
    auto guard_result = context->push_current();
    if (!guard_result) {
      return false;
    }
    auto guard = std::move(guard_result).value();
  }
  return cuStreamWaitEvent(stream, event, 0) == CUDA_SUCCESS;
}

}  // namespace

struct ReadHandle::Impl {
  std::shared_ptr<const backend::ImportedResources> resource;
  std::unique_ptr<lease::LeaseHandle> lease;
  std::size_t byte_size = 0;
  int device_id = -1;
  CUstream consumer_stream = nullptr;
  std::shared_ptr<CompletionEvent> completion;

  ~Impl() noexcept { release(); }

  void release() noexcept {
    if (!lease) {
      return;
    }

    if (!completion || completion->event == nullptr || !completion->context) {
      completion.reset();
      resource.reset();
      lease.reset();
      return;
    }

    auto guard_result = completion->context->push_current();
    if (guard_result) {
      auto guard = std::move(guard_result).value();
      const CUresult result = cuEventRecord(completion->event, consumer_stream);
      if (result == CUDA_SUCCESS) {
        DeferredReleaseQueue::instance().enqueue(DeferredItem{
            std::move(completion), std::move(resource), std::move(lease)});
        consumer_stream = nullptr;
        return;
      }
      RCUTILS_LOG_ERROR_NAMED(
          "ros2_cuda_ipc_core.subscriber.read_handle",
          "cuEventRecord failed; retaining publication lease: %s",
          ros2_cuda_ipc_core::detail::CudaDriverError(result)
              .to_string()
              .c_str());
    } else {
      RCUTILS_LOG_ERROR_NAMED(
          "ros2_cuda_ipc_core.subscriber.read_handle",
          "Failed to activate CUDA context while recording completion event; "
          "retaining publication lease");
    }

    DeferredReleaseQueue::instance().retain_failed(DeferredItem{
        std::move(completion), std::move(resource), std::move(lease)});
    consumer_stream = nullptr;
  }
};

ReadHandle::ReadHandle(std::unique_ptr<Impl> impl) noexcept
    : impl_(std::move(impl)) {}

ReadHandle::ReadHandle() noexcept = default;

ReadHandle::~ReadHandle() noexcept = default;

ReadHandle::ReadHandle(ReadHandle&& other) noexcept
    : impl_(std::move(other.impl_)) {}

ReadHandle& ReadHandle::operator=(ReadHandle&& other) noexcept {
  if (this != &other) {
    impl_ = std::move(other.impl_);
  }
  return *this;
}

bool ReadHandle::valid() const noexcept {
  return impl_ && impl_->resource && impl_->resource->dev_ptr != nullptr &&
         impl_->lease != nullptr;
}

void* ReadHandle::device_ptr() const noexcept {
  return impl_ && impl_->resource ? impl_->resource->dev_ptr : nullptr;
}

std::size_t ReadHandle::byte_size() const noexcept {
  return impl_ ? impl_->byte_size : 0;
}

int ReadHandle::device_id() const noexcept {
  return impl_ ? impl_->device_id : -1;
}

namespace detail {

std::optional<ReadHandle> ReadHandleFactory::make_unbound(
    std::shared_ptr<const backend::ImportedResources> resource,
    std::unique_ptr<lease::LeaseHandle> lease, std::size_t byte_size,
    int device_id) {
  if (!resource || !lease || resource->dev_ptr == nullptr) {
    return std::nullopt;
  }

  auto impl = std::make_unique<ReadHandle::Impl>();
  impl->resource = std::move(resource);
  impl->lease = std::move(lease);
  impl->byte_size = byte_size;
  impl->device_id = device_id;
  return ReadHandle(std::move(impl));
}

bool ReadHandleAccess::bind(ReadHandle& handle,
                            CUstream consumer_stream) noexcept {
  if (!handle.impl_ || !handle.impl_->resource || !handle.impl_->lease ||
      handle.impl_->completion) {
    return false;
  }
  if (!validate_stream(handle.impl_->resource, consumer_stream)) {
    return false;
  }
  auto completion = create_completion_event(handle.impl_->resource->context);
  if (handle.impl_->resource->context && !completion) {
    return false;
  }
  if (!enqueue_ready_event(handle.impl_->resource, consumer_stream)) {
    return false;
  }
  handle.impl_->consumer_stream = consumer_stream;
  handle.impl_->completion = std::move(completion);
  return true;
}

void ReadHandleAccess::unbind(ReadHandle& handle) noexcept {
  if (!handle.impl_) {
    return;
  }
  handle.impl_->completion.reset();
  handle.impl_->consumer_stream = nullptr;
}

std::optional<ReadHandle> ReadHandleFactory::make(
    std::shared_ptr<const backend::ImportedResources> resource,
    std::unique_ptr<lease::LeaseHandle> lease, std::size_t byte_size,
    int device_id, CUstream consumer_stream) {
  auto handle =
      make_unbound(std::move(resource), std::move(lease), byte_size, device_id);
  if (!handle || !ReadHandleAccess::bind(*handle, consumer_stream)) {
    return std::nullopt;
  }
  return handle;
}

}  // namespace detail

}  // namespace ros2_cuda_ipc_core::subscriber
