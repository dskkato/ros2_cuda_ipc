// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <cuda.h>
#include <pybind11/pybind11.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"
#include "ros2_cuda_ipc_core/detail/cuda_driver_context.hpp"
#include "ros2_cuda_ipc_core/image/image_view_mapper.hpp"
#include "ros2_cuda_ipc_core/lease/lease_handle.hpp"
#include "ros2_cuda_ipc_core/lease/lease_mapping.hpp"
#include "ros2_cuda_ipc_core/subscriber/buffer_view_mapper.hpp"
#include "ros2_cuda_ipc_core/subscriber/ipc_handle_cache.hpp"
#include "ros2_cuda_ipc_core/transport/memory_types.hpp"
#include "tensor_metadata.hpp"

#ifdef ROS2_CUDA_IPC_PY_ENABLE_TEST_SUPPORT
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace py = pybind11;

namespace ros2_cuda_ipc_py {

class MappingError : public std::runtime_error {
 public:
  explicit MappingError(const std::string& message)
      : std::runtime_error(message) {}
};

namespace {

py::handle required(const py::dict& descriptor, const char* key) {
  if (!descriptor.contains(key)) {
    throw py::key_error(std::string("descriptor is missing field '") + key +
                        "'");
  }
  return descriptor[key];
}

py::dict required_dict(const py::dict& descriptor, const char* key) {
  const py::handle value = required(descriptor, key);
  if (!PyDict_Check(value.ptr())) {
    throw py::type_error(std::string("descriptor field '") + key +
                         "' must be a dictionary");
  }
  return py::reinterpret_borrow<py::dict>(value);
}

template <typename T>
T unsigned_integer(py::handle value, const std::string& path) {
  // PyNumber_Index also accepts integer-like scalar objects such as numpy
  // integer values while rejecting floats and strings.
  PyObject* raw_index = PyNumber_Index(value.ptr());
  if (raw_index == nullptr) {
    PyErr_Clear();
    throw py::type_error(path + " must be a non-negative integer");
  }
  py::object index = py::reinterpret_steal<py::object>(raw_index);
  const unsigned long long parsed = PyLong_AsUnsignedLongLong(index.ptr());
  if (PyErr_Occurred() != nullptr) {
    PyErr_Clear();
    throw py::value_error(path + " is outside the supported unsigned range");
  }
  if (parsed > static_cast<unsigned long long>(std::numeric_limits<T>::max())) {
    throw py::value_error(path + " is outside the supported unsigned range");
  }
  return static_cast<T>(parsed);
}

int32_t signed_int32(py::handle value, const std::string& path) {
  PyObject* raw_index = PyNumber_Index(value.ptr());
  if (raw_index == nullptr) {
    PyErr_Clear();
    throw py::type_error(path + " must be an integer");
  }
  py::object index = py::reinterpret_steal<py::object>(raw_index);
  const long long parsed = PyLong_AsLongLong(index.ptr());
  if (PyErr_Occurred() != nullptr ||
      parsed < std::numeric_limits<int32_t>::min() ||
      parsed > std::numeric_limits<int32_t>::max()) {
    PyErr_Clear();
    throw py::value_error(path + " is outside the int32 range");
  }
  return static_cast<int32_t>(parsed);
}

std::string string_value(py::handle value, const std::string& path) {
  if (!PyUnicode_Check(value.ptr())) {
    throw py::type_error(path + " must be a string");
  }
  return py::cast<std::string>(value);
}

template <typename T, std::size_t N>
std::array<T, N> fixed_sequence(py::handle value, const std::string& path) {
  if (!PySequence_Check(value.ptr())) {
    throw py::type_error(path + " must be a sequence");
  }
  const Py_ssize_t length = PySequence_Size(value.ptr());
  if (length == -1) {
    throw py::error_already_set();
  }
  if (length != static_cast<Py_ssize_t>(N)) {
    throw py::value_error(path + " must contain exactly " + std::to_string(N) +
                          " elements");
  }

  std::array<T, N> result{};
  for (std::size_t index = 0; index < N; ++index) {
    py::object item = py::reinterpret_steal<py::object>(
        PySequence_GetItem(value.ptr(), static_cast<Py_ssize_t>(index)));
    if (!item) {
      throw py::error_already_set();
    }
    result[index] =
        unsigned_integer<T>(item, path + "[" + std::to_string(index) + "]");
  }
  return result;
}

ros2_cuda_ipc_msgs::msg::BufferCore buffer_core_from_descriptor(
    const py::dict& descriptor) {
  ros2_cuda_ipc_msgs::msg::BufferCore message;
  message.backend =
      unsigned_integer<uint8_t>(required(descriptor, "backend"), "backend");
  message.mem_handle = fixed_sequence<uint8_t, 64>(
      required(descriptor, "mem_handle"), "mem_handle");
  message.event_handle = fixed_sequence<uint8_t, 64>(
      required(descriptor, "event_handle"), "event_handle");
  message.shm_name = string_value(required(descriptor, "shm_name"), "shm_name");
  message.publisher_instance_id = fixed_sequence<uint8_t, 16>(
      required(descriptor, "publisher_instance_id"), "publisher_instance_id");
  message.device_id = unsigned_integer<uint32_t>(
      required(descriptor, "device_id"), "device_id");
  message.slot_id =
      unsigned_integer<uint32_t>(required(descriptor, "slot_id"), "slot_id");
  message.generation = unsigned_integer<uint32_t>(
      required(descriptor, "generation"), "generation");
  message.byte_size = unsigned_integer<uint64_t>(
      required(descriptor, "byte_size"), "byte_size");
  return message;
}

void fill_header(const py::dict& descriptor, std_msgs::msg::Header& header) {
  if (!descriptor.contains("header")) {
    return;
  }
  const py::dict header_descriptor = required_dict(descriptor, "header");
  if (header_descriptor.contains("frame_id")) {
    header.frame_id =
        string_value(header_descriptor["frame_id"], "header.frame_id");
  }
  if (!header_descriptor.contains("stamp")) {
    return;
  }
  const py::dict stamp = required_dict(header_descriptor, "stamp");
  header.stamp.sec = signed_int32(required(stamp, "sec"), "header.stamp.sec");
  header.stamp.nanosec = unsigned_integer<uint32_t>(required(stamp, "nanosec"),
                                                    "header.stamp.nanosec");
}

ros2_cuda_ipc_msgs::msg::GpuImage gpu_image_from_descriptor(
    const py::dict& descriptor) {
  ros2_cuda_ipc_msgs::msg::GpuImage message;
  fill_header(descriptor, message.header);
  message.dtype =
      unsigned_integer<uint8_t>(required(descriptor, "dtype"), "dtype");
  message.shape =
      fixed_sequence<uint32_t, 3>(required(descriptor, "shape"), "shape");
  message.strides =
      fixed_sequence<uint64_t, 3>(required(descriptor, "strides"), "strides");
  message.core = buffer_core_from_descriptor(required_dict(descriptor, "core"));
  if (descriptor.contains("encoding")) {
    message.encoding = string_value(descriptor["encoding"], "encoding");
  }
  return message;
}

uint32_t dtype_size(uint8_t dtype) {
  try {
    const auto dl_dtype =
        tensor_dl_dtype(static_cast<ros2_cuda_ipc_core::image::DType>(dtype));
    return dl_dtype.bits / 8;
  } catch (const std::invalid_argument&) {
    throw py::value_error("unsupported ros2_cuda_ipc image dtype " +
                          std::to_string(dtype));
  }
}

void validate_image_descriptor(
    const ros2_cuda_ipc_msgs::msg::GpuImage& message) {
  const uint32_t element_size = dtype_size(message.dtype);
  if (message.shape[0] == 0 || message.shape[1] == 0 || message.shape[2] == 0) {
    throw py::value_error("GpuImage shape dimensions must all be non-zero");
  }

  using WideUnsigned = unsigned __int128;
  const WideUnsigned needed =
      (static_cast<WideUnsigned>(message.shape[0] - 1) * message.strides[0]) +
      (static_cast<WideUnsigned>(message.shape[1] - 1) * message.strides[1]) +
      (static_cast<WideUnsigned>(message.shape[2] - 1) * message.strides[2]) +
      element_size;
  if (needed > std::numeric_limits<uint64_t>::max() ||
      needed > message.core.byte_size) {
    throw py::value_error("GpuImage shape/strides exceed BufferCore.byte_size");
  }
}

std::string cuda_error_message(
    const ros2_cuda_ipc_core::detail::CudaDriverError& error) {
  return error.to_string();
}

class PyTensorMetadata {
 public:
  explicit PyTensorMetadata(TensorMetadata metadata)
      : metadata_(std::move(metadata)) {}

  bool valid() const noexcept { return metadata_.owner.valid(); }
  uint64_t device_ptr() const noexcept {
    return static_cast<uint64_t>(reinterpret_cast<uintptr_t>(metadata_.data));
  }
  uint64_t byte_size() const noexcept { return metadata_.allocation_size; }
  int device_id() const noexcept { return metadata_.device_id; }
  int rank() const noexcept { return metadata_.rank; }
  uint64_t byte_offset() const noexcept { return metadata_.byte_offset; }
  py::tuple shape() const {
    return py::make_tuple(metadata_.shape[0], metadata_.shape[1],
                          metadata_.shape[2]);
  }
  py::tuple byte_strides() const {
    return py::make_tuple(metadata_.byte_strides[0], metadata_.byte_strides[1],
                          metadata_.byte_strides[2]);
  }
  py::tuple element_strides() const {
    return py::make_tuple(metadata_.element_strides[0],
                          metadata_.element_strides[1],
                          metadata_.element_strides[2]);
  }
  const std::string& dtype() const noexcept { return metadata_.dtype_name; }

 private:
  TensorMetadata metadata_;
};

struct DLPackManagerContext {
  TensorMetadata metadata;
};

void legacy_dlpack_deleter(DLManagedTensor* managed) noexcept {
  if (managed == nullptr) {
    return;
  }
  auto* context = static_cast<DLPackManagerContext*>(managed->manager_ctx);
  managed->manager_ctx = nullptr;
  try {
    delete context;
  } catch (...) {
    // A C ABI deleter must never allow an exception to escape.  The
    // C++ owner has a noexcept destructor in practice; retain this guard as a
    // final boundary guarantee.
  }
  try {
    delete managed;
  } catch (...) {
  }
}

void versioned_dlpack_deleter(DLManagedTensorVersioned* managed) noexcept {
  if (managed == nullptr) {
    return;
  }
  auto* context = static_cast<DLPackManagerContext*>(managed->manager_ctx);
  managed->manager_ctx = nullptr;
  try {
    delete context;
  } catch (...) {
  }
  try {
    delete managed;
  } catch (...) {
  }
}

constexpr char kDLPackName[] = "dltensor";
constexpr char kVersionedDLPackName[] = "dltensor_versioned";

void dlpack_capsule_destructor(PyObject* capsule) noexcept {
  if (capsule == nullptr) {
    return;
  }

  const char* name = PyCapsule_GetName(capsule);
  if (name == nullptr) {
    PyErr_Clear();
    return;
  }

  if (std::strcmp(name, kDLPackName) == 0) {
    auto* managed = static_cast<DLManagedTensor*>(
        PyCapsule_GetPointer(capsule, kDLPackName));
    if (managed != nullptr && managed->deleter != nullptr) {
      managed->deleter(managed);
    }
  } else if (std::strcmp(name, kVersionedDLPackName) == 0) {
    auto* managed = static_cast<DLManagedTensorVersioned*>(
        PyCapsule_GetPointer(capsule, kVersionedDLPackName));
    if (managed != nullptr && managed->deleter != nullptr) {
      managed->deleter(managed);
    }
  }
  // PyCapsule_GetPointer reports an error for an invalid/moved capsule.  A
  // destructor cannot propagate that error to Python.
  PyErr_Clear();
}

template <typename ManagedTensor>
void populate_dlpack_tensor(ManagedTensor& managed,
                            DLPackManagerContext& context) noexcept {
  auto& tensor = managed.dl_tensor;
  tensor.data = context.metadata.data;
  tensor.device = {kDLCUDA, context.metadata.device_id};
  tensor.ndim = context.metadata.rank;
  tensor.dtype = context.metadata.dl_dtype;
  tensor.shape = context.metadata.shape.data();
  tensor.strides = context.metadata.element_strides.data();
  tensor.byte_offset = context.metadata.byte_offset;
}

py::capsule make_legacy_dlpack_capsule(TensorMetadata metadata) {
  auto context = std::make_unique<DLPackManagerContext>();
  context->metadata = std::move(metadata);
  auto managed = std::make_unique<DLManagedTensor>();
  managed->manager_ctx = context.get();
  managed->deleter = &legacy_dlpack_deleter;
  populate_dlpack_tensor(*managed, *context);
  auto* managed_ptr = managed.release();
  context.release();

  try {
    return py::capsule(managed_ptr, kDLPackName, &dlpack_capsule_destructor);
  } catch (...) {
    legacy_dlpack_deleter(managed_ptr);
    throw;
  }
}

py::capsule make_versioned_dlpack_capsule(TensorMetadata metadata) {
  auto context = std::make_unique<DLPackManagerContext>();
  context->metadata = std::move(metadata);
  auto managed = std::make_unique<DLManagedTensorVersioned>();
  managed->version = {1, 0};
  managed->manager_ctx = context.get();
  managed->deleter = &versioned_dlpack_deleter;
  managed->flags = 0;
  populate_dlpack_tensor(*managed, *context);
  auto* managed_ptr = managed.release();
  context.release();

  try {
    return py::capsule(managed_ptr, kVersionedDLPackName,
                       &dlpack_capsule_destructor);
  } catch (...) {
    versioned_dlpack_deleter(managed_ptr);
    throw;
  }
}

class PyBufferView {
 public:
  explicit PyBufferView(ros2_cuda_ipc_core::subscriber::BufferView view)
      : view_(std::move(view)) {}

  bool valid() const noexcept { return view_.valid(); }
  uint64_t device_ptr() const noexcept {
    return static_cast<uint64_t>(
        reinterpret_cast<uintptr_t>(view_.device_ptr()));
  }
  uint64_t byte_size() const noexcept { return view_.byte_size; }
  int device_id() const noexcept { return view_.device_id; }
  uint32_t slot_id() const noexcept { return view_.slot_id; }
  uint32_t generation() const noexcept { return view_.generation; }

  void wait(std::uintptr_t stream_ptr) const {
    if (!view_.valid()) {
      throw MappingError("cannot wait on an invalid BufferView");
    }
    const auto result = [&]() {
      py::gil_scoped_release release;
      return view_.enqueue_ready_event(reinterpret_cast<CUstream>(stream_ptr));
    }();
    if (!result) {
      throw std::runtime_error("CUDA ready-event wait failed: " +
                               cuda_error_message(result.error()));
    }
  }

  void close() noexcept { view_.reset(); }

 private:
  ros2_cuda_ipc_core::subscriber::BufferView view_;
};

class PyImageView {
 public:
  explicit PyImageView(ros2_cuda_ipc_core::image::ImageView view)
      : view_(std::move(view)) {}

  PyImageView retain() const { return PyImageView(view_); }

  PyTensorMetadata tensor_metadata() const {
    return PyTensorMetadata(make_tensor_metadata(view_));
  }

  bool valid() const noexcept { return view_.valid(); }
  uint64_t device_ptr() const noexcept {
    return static_cast<uint64_t>(
        reinterpret_cast<uintptr_t>(view_.core.device_ptr()));
  }
  uint64_t byte_size() const noexcept { return view_.core.byte_size; }
  int device_id() const noexcept { return view_.core.device_id; }
  uint32_t slot_id() const noexcept { return view_.core.slot_id; }
  uint32_t generation() const noexcept { return view_.core.generation; }
  py::tuple shape() const {
    return py::make_tuple(view_.shape[0], view_.shape[1], view_.shape[2]);
  }
  py::tuple strides() const {
    return py::make_tuple(view_.strides[0], view_.strides[1], view_.strides[2]);
  }
  uint8_t dtype_code() const noexcept {
    return static_cast<uint8_t>(view_.dtype);
  }
  const std::string& encoding() const noexcept { return view_.encoding; }
  const std::string& frame_id() const noexcept { return view_.header.frame_id; }

  void wait(std::uintptr_t stream_ptr) const {
    if (!view_.valid()) {
      throw MappingError("cannot wait on an invalid ImageView");
    }
    const auto result = [&]() {
      py::gil_scoped_release release;
      return view_.enqueue_ready_event(reinterpret_cast<CUstream>(stream_ptr));
    }();
    if (!result) {
      throw std::runtime_error("CUDA ready-event wait failed: " +
                               cuda_error_message(result.error()));
    }
  }

  py::capsule dlpack(std::uintptr_t stream_ptr, bool synchronize,
                     bool versioned) const {
    if (!view_.valid()) {
      throw MappingError("cannot export an invalid ImageView through DLPack");
    }

    // Validate and retain the framework-independent layout before touching a
    // consumer stream. Invalid metadata must not enqueue a synchronization
    // side effect.
    TensorMetadata metadata;
    try {
      metadata = make_tensor_metadata(view_);
    } catch (const std::invalid_argument& error) {
      // The DLPack Python protocol uses BufferError for layouts that cannot
      // be represented safely as a dense strided tensor.
      throw py::buffer_error(error.what());
    }

    if (synchronize) {
      const auto result = [&]() {
        py::gil_scoped_release release;
        return view_.enqueue_ready_event(
            reinterpret_cast<CUstream>(stream_ptr));
      }();
      if (!result) {
        throw std::runtime_error("CUDA ready-event wait failed: " +
                                 cuda_error_message(result.error()));
      }
    }

    if (versioned) {
      return make_versioned_dlpack_capsule(std::move(metadata));
    }
    return make_legacy_dlpack_capsule(std::move(metadata));
  }

  void close() noexcept { view_.core.reset(); }

 private:
  ros2_cuda_ipc_core::image::ImageView view_;
};

class PyBufferViewMapper {
 public:
  PyBufferView map(const py::dict& descriptor) const {
    const auto message = buffer_core_from_descriptor(descriptor);
    ros2_cuda_ipc_core::subscriber::BufferView view;
    {
      py::gil_scoped_release release;
      view = mapper_.map(message);
    }
    if (!view.valid()) {
      throw MappingError(
          "BufferCore was rejected by the C++ mapper (lease, generation, "
          "backend, or CUDA import failure)");
    }
    return PyBufferView(std::move(view));
  }

 private:
  ros2_cuda_ipc_core::subscriber::BufferViewMapper mapper_;
};

class PyImageMapper {
 public:
  PyImageView map(const py::dict& descriptor) const {
    const auto message = gpu_image_from_descriptor(descriptor);
    validate_image_descriptor(message);

    ros2_cuda_ipc_core::image::ImageView view;
    {
      py::gil_scoped_release release;
      view = mapper_.map(message);
    }
    if (!view.valid()) {
      throw MappingError(
          "GpuImage was rejected by the C++ mapper (lease, generation, "
          "backend, or CUDA import failure)");
    }
    // Keep the C++ implementation's complete bounds check as the final
    // authority after the native core view has been attached.
    if (!view.sanity_check()) {
      throw py::value_error(
          "GpuImage shape/strides exceed BufferCore.byte_size");
    }
    return PyImageView(std::move(view));
  }

 private:
  ros2_cuda_ipc_core::image::ImageViewMapper mapper_;
};

#ifdef ROS2_CUDA_IPC_PY_ENABLE_TEST_SUPPORT

ros2_cuda_ipc_core::PublisherInstanceId test_instance_id(
    const std::string& seed) {
  ros2_cuda_ipc_core::PublisherInstanceId id{};
  uint32_t state = 2166136261u;
  for (const uint8_t byte : seed) {
    state = (state ^ byte) * 16777619u;
  }
  for (auto& byte : id) {
    state = state * 1664525u + 1013904223u;
    byte = static_cast<uint8_t>(state >> 24);
  }
  if (ros2_cuda_ipc_core::is_nil(id)) {
    id.back() = 1;
  }
  return id;
}

class TestLeaseProbe {
 public:
  TestLeaseProbe(
      std::shared_ptr<ros2_cuda_ipc_core::lease::LeaseMapping> mapping,
      uint32_t slot_id, std::string shm_name)
      : mapping_(std::move(mapping)),
        slot_id_(slot_id),
        shm_name_(std::move(shm_name)) {}

  ~TestLeaseProbe() {
    if (!shm_name_.empty()) {
      (void)::shm_unlink(shm_name_.c_str());
    }
  }

  uint32_t refcount() const {
    const auto value = ros2_cuda_ipc_core::lease::LeaseHandle::current_refcount(
        mapping_, slot_id_);
    return value.value_or(0);
  }

  const std::string& shm_name() const noexcept { return shm_name_; }

 private:
  std::shared_ptr<ros2_cuda_ipc_core::lease::LeaseMapping> mapping_;
  uint32_t slot_id_;
  std::string shm_name_;
};

template <typename Array>
py::list bytes_to_list(const Array& bytes) {
  py::list result;
  for (const uint8_t byte : bytes) {
    result.append(byte);
  }
  return result;
}

py::tuple make_test_image() {
  static std::atomic<uint64_t> counter{0};
  std::ostringstream name;
  name << "/ros2_cuda_ipc_py_test_" << static_cast<long long>(::getpid()) << "_"
       << counter.fetch_add(1);
  const std::string shm_name = name.str();
  const auto instance_id = test_instance_id(shm_name);
  auto mapping =
      ros2_cuda_ipc_core::lease::LeaseMapping::create(shm_name, instance_id, 1);
  if (!mapping) {
    throw std::runtime_error("test LeaseMapping::create failed");
  }
  const auto reservation =
      ros2_cuda_ipc_core::lease::LeaseHandle::reserve_for_publish(mapping, 1);
  if (!reservation) {
    throw std::runtime_error("test LeaseHandle::reserve_for_publish failed");
  }

  ros2_cuda_ipc_msgs::msg::GpuImage message;
  message.header.frame_id = "test_frame";
  message.dtype = static_cast<uint8_t>(ros2_cuda_ipc_core::image::DType::U8);
  message.shape = {2, 3, 4};
  message.strides = {12, 4, 1};
  message.encoding = "rgba8";
  message.core.backend = ros2_cuda_ipc_msgs::msg::BufferCore::CUDA_IPC;
  message.core.mem_handle.fill(0);
  message.core.event_handle.fill(0);
  message.core.mem_handle[0] = 17;
  message.core.event_handle[0] = 18;
  message.core.shm_name = shm_name;
  message.core.publisher_instance_id = instance_id;
  message.core.device_id = 0;
  message.core.slot_id = reservation->slot_id;
  message.core.generation = reservation->generation;
  message.core.byte_size = 24;

  ros2_cuda_ipc_core::subscriber::IpcHandleKey key{};
  key.publisher_instance_id = instance_id;
  key.backend = message.core.backend;
  key.device_id = message.core.device_id;
  key.mem = message.core.mem_handle;
  key.event = message.core.event_handle;
  ros2_cuda_ipc_core::backend::ImportedResources imported;
  imported.dev_ptr = reinterpret_cast<void*>(static_cast<uintptr_t>(0x100000));
  // A null event makes this deterministic, while the normal native wait path
  // still receives and forwards the requested stream pointer.
  (void)ros2_cuda_ipc_core::subscriber::IpcHandleCache::instance()
      .insert_or_discard_duplicate(key, std::move(imported));

  ros2_cuda_ipc_core::image::ImageViewMapper mapper;
  auto view = mapper.map(message);
  if (!view.valid() || !view.sanity_check()) {
    throw std::runtime_error("test ImageViewMapper fixture failed");
  }

  py::dict core;
  core["backend"] = message.core.backend;
  core["mem_handle"] = bytes_to_list(message.core.mem_handle);
  core["event_handle"] = bytes_to_list(message.core.event_handle);
  core["shm_name"] = message.core.shm_name;
  core["publisher_instance_id"] =
      bytes_to_list(message.core.publisher_instance_id);
  core["device_id"] = message.core.device_id;
  core["slot_id"] = message.core.slot_id;
  core["generation"] = message.core.generation;
  core["byte_size"] = message.core.byte_size;
  py::dict stamp;
  stamp["sec"] = 0;
  stamp["nanosec"] = 0;
  py::dict header;
  header["stamp"] = stamp;
  header["frame_id"] = message.header.frame_id;
  py::dict descriptor;
  descriptor["header"] = header;
  descriptor["dtype"] = message.dtype;
  descriptor["shape"] = py::make_tuple(2, 3, 4);
  descriptor["strides"] = py::make_tuple(12, 4, 1);
  descriptor["core"] = core;
  descriptor["encoding"] = message.encoding;

  auto probe =
      std::make_shared<TestLeaseProbe>(mapping, reservation->slot_id, shm_name);
  return py::make_tuple(PyImageView(std::move(view)), probe, descriptor);
}

#endif  // ROS2_CUDA_IPC_PY_ENABLE_TEST_SUPPORT

}  // namespace

}  // namespace ros2_cuda_ipc_py

PYBIND11_MODULE(_native, module) {
  using namespace ros2_cuda_ipc_py;

  module.doc() = "Native ros2_cuda_ipc subscriber mapper bindings";
  auto& mapping_error = py::register_local_exception<MappingError>(
      module, "MappingError", PyExc_RuntimeError);
  (void)mapping_error;

  py::class_<PyBufferView>(module, "BufferView")
      .def_property_readonly("valid", &PyBufferView::valid)
      .def_property_readonly("device_ptr", &PyBufferView::device_ptr)
      .def_property_readonly("byte_size", &PyBufferView::byte_size)
      .def_property_readonly("device_id", &PyBufferView::device_id)
      .def_property_readonly("slot_id", &PyBufferView::slot_id)
      .def_property_readonly("generation", &PyBufferView::generation)
      .def("wait", &PyBufferView::wait, py::arg("stream_ptr"))
      .def("close", &PyBufferView::close);

  py::class_<PyTensorMetadata>(module, "_TensorMetadata")
      .def_property_readonly("valid", &PyTensorMetadata::valid)
      .def_property_readonly("device_ptr", &PyTensorMetadata::device_ptr)
      .def_property_readonly("byte_size", &PyTensorMetadata::byte_size)
      .def_property_readonly("device_id", &PyTensorMetadata::device_id)
      .def_property_readonly("rank", &PyTensorMetadata::rank)
      .def_property_readonly("byte_offset", &PyTensorMetadata::byte_offset)
      .def_property_readonly("shape", &PyTensorMetadata::shape)
      .def_property_readonly("byte_strides", &PyTensorMetadata::byte_strides)
      .def_property_readonly("element_strides",
                             &PyTensorMetadata::element_strides)
      .def_property_readonly("dtype", &PyTensorMetadata::dtype);

  py::class_<PyImageView>(module, "ImageView")
      .def("_retain", &PyImageView::retain)
      .def("_tensor_metadata", &PyImageView::tensor_metadata)
      .def_property_readonly("valid", &PyImageView::valid)
      .def_property_readonly("device_ptr", &PyImageView::device_ptr)
      .def_property_readonly("byte_size", &PyImageView::byte_size)
      .def_property_readonly("device_id", &PyImageView::device_id)
      .def_property_readonly("slot_id", &PyImageView::slot_id)
      .def_property_readonly("generation", &PyImageView::generation)
      .def_property_readonly("shape", &PyImageView::shape)
      .def_property_readonly("strides", &PyImageView::strides)
      .def_property_readonly("dtype_code", &PyImageView::dtype_code)
      .def_property_readonly("encoding", &PyImageView::encoding)
      .def_property_readonly("frame_id", &PyImageView::frame_id)
      .def("wait", &PyImageView::wait, py::arg("stream_ptr"))
      .def("_dlpack", &PyImageView::dlpack, py::arg("stream_ptr"),
           py::arg("synchronize"), py::arg("versioned"))
      .def("close", &PyImageView::close);

  py::class_<PyBufferViewMapper>(module, "BufferViewMapper")
      .def(py::init<>())
      .def("map", &PyBufferViewMapper::map, py::arg("descriptor"));

  py::class_<PyImageMapper>(module, "ImageMapper")
      .def(py::init<>())
      .def("map", &PyImageMapper::map, py::arg("descriptor"));

#ifdef ROS2_CUDA_IPC_PY_ENABLE_TEST_SUPPORT
  py::class_<TestLeaseProbe, std::shared_ptr<TestLeaseProbe>>(module,
                                                              "_TestLeaseProbe")
      .def("refcount", &TestLeaseProbe::refcount)
      .def_property_readonly("shm_name", &TestLeaseProbe::shm_name);
  module.def("_make_test_image", &make_test_image);
#endif
}
