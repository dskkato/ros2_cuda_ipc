// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_metadata.hpp"
#include "ros2_cuda_ipc_core/publisher_instance_id.hpp"

namespace ros2_cuda_ipc_core::publisher {

class BufferMetadataManager {
 public:
  struct Reservation {
    std::shared_ptr<buffer_metadata::BufferMetadata> mapping;
    uint32_t slot_id = 0;
    uint32_t generation = 0;
    std::string shm_name;
    PublisherInstanceId publisher_instance_id{};
  };

  BufferMetadataManager(std::string shm_name_prefix, std::size_t slot_count);

  ~BufferMetadataManager();

  bool initialise();
  void reset() noexcept;
  bool is_initialised() const noexcept;
  std::optional<Reservation> reserve_for_publish();
  bool commit(const Reservation& reservation) noexcept;
  bool cancel(const Reservation& reservation) noexcept;

  std::string shm_name() const;
  PublisherInstanceId publisher_instance_id() const;

 private:
  std::string shm_name_prefix_;
  std::string shm_name_;
  PublisherInstanceId publisher_instance_id_{};
  std::size_t slot_count_;
  mutable std::mutex mutex_;
  std::shared_ptr<buffer_metadata::BufferMetadata> mapping_;
  bool initialised_ = false;
};

}  // namespace ros2_cuda_ipc_core::publisher
