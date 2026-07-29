// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "ros2_cuda_ipc_core/lease/lease_mapping.hpp"

namespace ros2_cuda_ipc_core::subscriber::detail {

/// Internal subscriber cache for shared-memory lease mappings.
class LeaseMappingCache {
 public:
  using AttachFn = std::function<std::shared_ptr<lease::LeaseMapping>(
      const std::string&, const PublisherInstanceId&)>;

  explicit LeaseMappingCache(AttachFn attach_fn = lease::LeaseMapping::attach);
  ~LeaseMappingCache();

  std::shared_ptr<lease::LeaseMapping> get_or_attach(
      const std::string& shm_name,
      const PublisherInstanceId& publisher_instance_id) const;

  void clear() const;
  std::size_t size() const;

 private:
  struct Key {
    std::string shm_name;
    PublisherInstanceId publisher_instance_id{};

    bool operator==(const Key& other) const noexcept {
      return shm_name == other.shm_name &&
             publisher_instance_id == other.publisher_instance_id;
    }
  };

  struct KeyHash {
    std::size_t operator()(const Key& key) const noexcept;
  };

  AttachFn attach_fn_;
  mutable std::mutex mutex_;
  mutable std::unordered_map<Key, std::shared_ptr<lease::LeaseMapping>, KeyHash>
      mappings_;
};

}  // namespace ros2_cuda_ipc_core::subscriber::detail
