#include <cuda_runtime_api.h>
#include <pybind11/pybind11.h>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace py = pybind11;

// Minimal DLPack structures to avoid external dependency
extern "C" {

enum DLDeviceType {
  kDLCPU = 1,
  kDLGPU = 2,
};

struct DLDevice {
  DLDeviceType device_type;
  int device_id;
};

struct DLDataType {
  uint8_t code;
  uint8_t bits;
  uint16_t lanes;
};

struct DLTensor {
  void* data;
  DLDevice device;
  int ndim;
  DLDataType dtype;
  int64_t* shape;
  int64_t* strides;
  uint64_t byte_offset;
};

struct DLManagedTensor {
  DLTensor dl_tensor;
  void* manager_ctx;
  void (*deleter)(DLManagedTensor* self);
};

}  // extern "C"

namespace {

py::capsule to_dlpack(std::uintptr_t ptr, std::size_t size_bytes) {
  if (ptr == 0) {
    throw std::invalid_argument("Pointer is null.");
  }
  cudaPointerAttributes attr;
  cudaError_t err =
      cudaPointerGetAttributes(&attr, reinterpret_cast<void*>(ptr));
#if CUDART_VERSION >= 10000
  if (err != cudaSuccess || attr.type != cudaMemoryTypeDevice) {
    throw std::invalid_argument("Pointer is not a valid CUDA device pointer.");
  }
  int device_id = attr.device;
#else
  if (err != cudaSuccess || attr.memoryType != cudaMemoryTypeDevice) {
    throw std::invalid_argument("Pointer is not a valid CUDA device pointer.");
  }
  int device_id = attr.device;
#endif
  int current_device = 0;
  err = cudaGetDevice(&current_device);
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string("cudaGetDevice failed: ") +
                             cudaGetErrorString(err));
  }
  if (device_id != current_device) {
    throw std::invalid_argument(
        "Pointer is not accessible from the current CUDA device.");
  }
  auto* managed = new DLManagedTensor();
  int64_t* shape = new int64_t[1];
  shape[0] = static_cast<int64_t>(size_bytes);
  managed->dl_tensor.data = reinterpret_cast<void*>(ptr);
  managed->dl_tensor.device = {kDLGPU, device_id};
  managed->dl_tensor.ndim = 1;
  managed->dl_tensor.dtype = {1, 8, 1};  // kDLUInt
  managed->dl_tensor.shape = shape;
  managed->dl_tensor.strides = nullptr;
  managed->dl_tensor.byte_offset = 0;
  managed->manager_ctx = nullptr;
  managed->deleter = [](DLManagedTensor* self) {
    delete[] self->dl_tensor.shape;
    delete self;
  };
  return py::capsule(managed, "dltensor", [](PyObject* cap) {
    auto* m =
        static_cast<DLManagedTensor*>(PyCapsule_GetPointer(cap, "dltensor"));
    if (m && m->deleter) {
      m->deleter(m);
    }
  });
}

void from_dlpack(std::uintptr_t dst_ptr, py::capsule cap) {
  auto* managed = static_cast<DLManagedTensor*>(
      PyCapsule_GetPointer(cap.ptr(), "dltensor"));
  if (!managed) {
    throw std::runtime_error("Invalid DLTensor capsule");
  }
  DLTensor& t = managed->dl_tensor;
  std::size_t elem_bytes = (t.dtype.bits * t.dtype.lanes + 7) / 8;
  std::size_t n = 1;
  for (int i = 0; i < t.ndim; ++i) {
    n *= static_cast<std::size_t>(t.shape[i]);
  }
  std::size_t bytes = elem_bytes * n;
  cudaError_t err;
  if (t.device.device_type == kDLCPU) {
    err = cudaMemcpy(reinterpret_cast<void*>(dst_ptr), t.data, bytes,
                     cudaMemcpyHostToDevice);
  } else if (t.device.device_type == kDLGPU) {
    int current_device = 0;
    err = cudaGetDevice(&current_device);
    if (err != cudaSuccess) {
      throw std::runtime_error(std::string("cudaGetDevice failed: ") +
                               cudaGetErrorString(err));
    }
    if (t.device.device_id != current_device) {
      err = cudaMemcpyPeer(reinterpret_cast<void*>(dst_ptr), current_device,
                           t.data, t.device.device_id, bytes);
    } else {
      err = cudaMemcpy(reinterpret_cast<void*>(dst_ptr), t.data, bytes,
                       cudaMemcpyDeviceToDevice);
    }
  } else {
    throw std::invalid_argument("Unsupported device type in DLTensor");
  }
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string("cudaMemcpy failed: ") +
                             cudaGetErrorString(err));
  }
  PyCapsule_SetName(cap.ptr(), "used_dltensor");
  // Do not call the deleter here; let the capsule destructor handle cleanup.
}

}  // namespace

void init_dlpack_utils(py::module_& m) {
  m.def("to_dlpack", &to_dlpack, py::arg("ptr"), py::arg("size_bytes"));
  m.def("from_dlpack", &from_dlpack, py::arg("dst_ptr"), py::arg("tensor"));
}
