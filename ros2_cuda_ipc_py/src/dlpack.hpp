// Copyright (c) 2017 by DLPack contributors
// SPDX-License-Identifier: Apache-2.0
//
// This is the small ABI subset needed by the Python DLPack producer in this
// package.  It follows the public DLPack v1.0 header.  Keep this header free
// of Python and framework dependencies so managed-tensor deleters can run
// without the GIL.

#pragma once

#include <cstddef>
#include <cstdint>

extern "C" {

typedef struct {
  uint32_t major;
  uint32_t minor;
} DLPackVersion;

typedef enum : int32_t {
  kDLCPU = 1,
  kDLCUDA = 2,
} DLDeviceType;

typedef struct {
  DLDeviceType device_type;
  int32_t device_id;
} DLDevice;

typedef enum {
  kDLInt = 0U,
  kDLUInt = 1U,
  kDLFloat = 2U,
} DLDataTypeCode;

typedef struct {
  uint8_t code;
  uint8_t bits;
  uint16_t lanes;
} DLDataType;

typedef struct {
  void* data;
  DLDevice device;
  int32_t ndim;
  DLDataType dtype;
  int64_t* shape;
  int64_t* strides;
  uint64_t byte_offset;
} DLTensor;

typedef struct DLManagedTensor {
  DLTensor dl_tensor;
  void* manager_ctx;
  void (*deleter)(struct DLManagedTensor* self);
} DLManagedTensor;

typedef struct DLManagedTensorVersioned {
  DLPackVersion version;
  void* manager_ctx;
  void (*deleter)(struct DLManagedTensorVersioned* self);
  uint64_t flags;
  DLTensor dl_tensor;
} DLManagedTensorVersioned;

}  // extern "C"

constexpr uint64_t DLPACK_FLAG_BITMASK_READ_ONLY = (1ULL << 0U);
