#include <cuda_runtime_api.h>
#include <pybind11/pybind11.h>

#include <cstdint>
#include <cstring>

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
  auto* managed = new DLManagedTensor();
  int64_t* shape = new int64_t[1];
  shape[0] = static_cast<int64_t>(size_bytes);
  managed->dl_tensor.data = reinterpret_cast<void*>(ptr);
  int device_id = 0;
  (void)cudaGetDevice(&device_id);
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
    auto* m = static_cast<DLManagedTensor*>(PyCapsule_GetPointer(cap, "dltensor"));
    if (m && m->deleter) {
      m->deleter(m);
    }
  });
}

void from_dlpack(std::uintptr_t dst_ptr, py::capsule cap) {
  auto* managed =
      static_cast<DLManagedTensor*>(PyCapsule_GetPointer(cap.ptr(), "dltensor"));
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
  cudaMemcpy(reinterpret_cast<void*>(dst_ptr), t.data, bytes, cudaMemcpyDeviceToDevice);
  if (managed->deleter) {
    managed->deleter(managed);
  }
  PyCapsule_SetName(cap.ptr(), "used_dltensor");
}

}  // namespace

void init_dlpack_utils(py::module_& m) {
  m.def("to_dlpack", &to_dlpack, py::arg("ptr"), py::arg("size_bytes"));
  m.def("from_dlpack", &from_dlpack, py::arg("dst_ptr"), py::arg("tensor"));
}

