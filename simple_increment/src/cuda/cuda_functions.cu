// Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
// Copyright 2025 dskkato
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simple_increment/cuda_functions.hpp"

namespace simple_increment {
namespace {

__global__ void increment_kernel(int size, const uint8_t *source,
                                 uint8_t *destination) {
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= size) {
    return;
  }
  destination[index] = static_cast<uint8_t>(source[index] + 1);
}

inline int block_count(int total_elements, int threads_per_block) {
  return (total_elements + threads_per_block - 1) / threads_per_block;
}

}  // namespace

cudaError_t compute_increment(int size_in_bytes, const uint8_t *source,
                              uint8_t *destination,
                              cudaStream_t stream) noexcept {
  const int threads = 256;
  increment_kernel<<<block_count(size_in_bytes, threads), threads, 0, stream>>>(
      size_in_bytes, source, destination);
  return cudaGetLastError();
}

cudaError_t compute_increment_inplace(int size_in_bytes, uint8_t *image,
                                      cudaStream_t stream) noexcept {
  const int threads = 256;
  increment_kernel<<<block_count(size_in_bytes, threads), threads, 0, stream>>>(
      size_in_bytes, image, image);
  return cudaGetLastError();
}

}  // namespace simple_increment
