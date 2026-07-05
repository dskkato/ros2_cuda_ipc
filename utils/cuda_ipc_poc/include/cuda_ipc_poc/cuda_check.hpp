// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#pragma once

#include <cuda.h>
#include <cuda_runtime.h>

#include <cstdio>
#include <cstdlib>

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
