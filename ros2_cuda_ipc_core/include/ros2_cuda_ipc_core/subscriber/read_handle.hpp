// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>

namespace ros2_cuda_ipc_core::backend {
struct ImportedResources;
}

namespace ros2_cuda_ipc_core::lease {
class LeaseHandle;
}

namespace ros2_cuda_ipc_core::subscriber {

class ReadHandle;

namespace detail {
struct ReadHandleAccess;
struct ReadHandleFactory;
}  // namespace detail

/// Owns one subscriber-side GPU read.
///
/// The handle waits for producer publication on the stream supplied to the
/// mapper.  Destroying the handle records consumer completion and keeps the
/// publication lease alive until that completion has been observed.  The
/// supplied consumer stream must remain valid until handle destruction has
/// completed the completion-event record.
class ReadHandle {
 public:
  ReadHandle() noexcept;
  ~ReadHandle() noexcept;

  ReadHandle(ReadHandle&& other) noexcept;
  ReadHandle& operator=(ReadHandle&& other) noexcept;
  ReadHandle(const ReadHandle&) = delete;
  ReadHandle& operator=(const ReadHandle&) = delete;

  explicit operator bool() const noexcept { return valid(); }
  bool valid() const noexcept;

  template <class T = void>
  T* data() const noexcept {
    return static_cast<T*>(device_ptr());
  }

  void* device_ptr() const noexcept;
  std::size_t byte_size() const noexcept;
  int device_id() const noexcept;

 private:
  struct Impl;
  explicit ReadHandle(std::unique_ptr<Impl> impl) noexcept;

  std::unique_ptr<Impl> impl_;

  friend class BufferMapper;
  friend struct detail::ReadHandleAccess;
  friend struct detail::ReadHandleFactory;
};

}  // namespace ros2_cuda_ipc_core::subscriber
