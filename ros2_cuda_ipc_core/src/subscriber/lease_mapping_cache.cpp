// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include "ros2_cuda_ipc_core/detail/lease_mapping_cache.hpp"

#include <utility>

namespace ros2_cuda_ipc_core::subscriber::detail {

std::size_t LeaseMappingCache::KeyHash::operator()(
    const Key& key) const noexcept {
  std::size_t hash = std::hash<std::string>{}(key.shm_name);
  for (const uint8_t byte : key.publisher_instance_id) {
    hash ^= static_cast<std::size_t>(byte) +
            static_cast<std::size_t>(0x9e3779b9) + (hash << 6) + (hash >> 2);
  }
  return hash;
}

LeaseMappingCache::LeaseMappingCache(AttachFn attach_fn)
    : attach_fn_(std::move(attach_fn)) {}

LeaseMappingCache::~LeaseMappingCache() { clear(); }

std::shared_ptr<lease::LeaseMapping> LeaseMappingCache::get_or_attach(
    const std::string& shm_name,
    const PublisherInstanceId& publisher_instance_id) const {
  const Key key{shm_name, publisher_instance_id};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = mappings_.find(key);
    if (it != mappings_.end()) {
      return it->second;
    }
  }

  auto candidate = attach_fn_(shm_name, publisher_instance_id);
  if (!candidate) {
    return nullptr;
  }

  std::shared_ptr<lease::LeaseMapping> result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto [it, inserted] = mappings_.emplace(key, candidate);
    (void)inserted;
    result = it->second;
  }
  // If another thread won the race, the duplicate candidate is released after
  // the mutex scope so a custom mapping deleter can safely inspect the cache.
  return result;
}

void LeaseMappingCache::clear() const {
  decltype(mappings_) entries;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    entries.swap(mappings_);
  }
  entries.clear();
}

std::size_t LeaseMappingCache::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mappings_.size();
}

}  // namespace ros2_cuda_ipc_core::subscriber::detail
