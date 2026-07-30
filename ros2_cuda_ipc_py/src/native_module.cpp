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
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "ros2_cuda_ipc_core/detail/image_view_dlpack.hpp"
#include "ros2_cuda_ipc_core/image/image_view_mapper.hpp"
#include "ros2_cuda_ipc_core/subscriber/buffer_mapper.hpp"
#include "ros2_cuda_ipc_py/dlpack/image_tensor_descriptor.hpp"

#ifdef ROS2_CUDA_IPC_PY_ENABLE_TEST_SUPPORT
#include <sys/mman.h>
#include <unistd.h>

#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"
#include "ros2_cuda_ipc_core/lease/lease_handle.hpp"
#include "ros2_cuda_ipc_core/lease/lease_mapping.hpp"
#include "ros2_cuda_ipc_core/subscriber/ipc_handle_cache.hpp"
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
    const auto dl_dtype = dlpack::tensor_dl_dtype(
        static_cast<ros2_cuda_ipc_core::image::DType>(dtype));
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

// Lifetime object for a DLPack export. The descriptor is the value projected
// into DLPack, while owner retains the mapped resource until the consumer
// releases the managed tensor.
struct DlpackExportContext {
  dlpack::ImageTensorDescriptor tensor;
  std::unique_ptr<ros2_cuda_ipc_core::image::ImageView> owner;
};

void legacy_dlpack_deleter(DLManagedTensor* managed) noexcept {
  if (managed == nullptr) {
    return;
  }
  auto* context = static_cast<DlpackExportContext*>(managed->manager_ctx);
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
  auto* context = static_cast<DlpackExportContext*>(managed->manager_ctx);
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
                            DlpackExportContext& context) noexcept {
  auto& tensor = managed.dl_tensor;
  tensor.data = context.tensor.data;
  tensor.device = {kDLCUDA, context.tensor.device_id};
  tensor.ndim = context.tensor.rank;
  tensor.dtype = context.tensor.dl_dtype;
  tensor.shape = context.tensor.shape.data();
  tensor.strides = context.tensor.element_strides.data();
  tensor.byte_offset = context.tensor.byte_offset;
}

py::capsule prepare_legacy_dlpack_capsule(
    const dlpack::ImageTensorDescriptor& tensor,
    std::unique_ptr<ros2_cuda_ipc_core::image::ImageView>& owner,
    DlpackExportContext*& prepared_context) {
  auto context = std::make_unique<DlpackExportContext>();
  context->tensor = tensor;
  context->owner = std::move(owner);
  try {
    auto managed = std::make_unique<DLManagedTensor>();
    managed->manager_ctx = context.get();
    managed->deleter = &legacy_dlpack_deleter;
    populate_dlpack_tensor(*managed, *context);
    auto* managed_ptr = managed.release();
    try {
      py::capsule capsule(managed_ptr, kDLPackName, &dlpack_capsule_destructor);
      prepared_context = context.release();
      return capsule;
    } catch (...) {
      owner = std::move(context->owner);
      managed_ptr->manager_ctx = nullptr;
      delete managed_ptr;
      throw;
    }
  } catch (...) {
    if (!owner) {
      owner = std::move(context->owner);
    }
    throw;
  }
}

py::capsule prepare_versioned_dlpack_capsule(
    const dlpack::ImageTensorDescriptor& tensor,
    std::unique_ptr<ros2_cuda_ipc_core::image::ImageView>& owner,
    DlpackExportContext*& prepared_context) {
  auto context = std::make_unique<DlpackExportContext>();
  context->tensor = tensor;
  context->owner = std::move(owner);
  try {
    auto managed = std::make_unique<DLManagedTensorVersioned>();
    managed->version = {1, 0};
    managed->manager_ctx = context.get();
    managed->deleter = &versioned_dlpack_deleter;
    managed->flags = 0;
    populate_dlpack_tensor(*managed, *context);
    auto* managed_ptr = managed.release();
    try {
      py::capsule capsule(managed_ptr, kVersionedDLPackName,
                          &dlpack_capsule_destructor);
      prepared_context = context.release();
      return capsule;
    } catch (...) {
      owner = std::move(context->owner);
      managed_ptr->manager_ctx = nullptr;
      delete managed_ptr;
      throw;
    }
  } catch (...) {
    if (!owner) {
      owner = std::move(context->owner);
    }
    throw;
  }
}

class PyReadHandle {
 public:
  explicit PyReadHandle(ros2_cuda_ipc_core::subscriber::ReadHandle view)
      : view_(std::move(view)) {}

  bool valid() const noexcept { return view_.valid(); }
  uint64_t device_ptr() const noexcept {
    return static_cast<uint64_t>(
        reinterpret_cast<uintptr_t>(view_.device_ptr()));
  }
  uint64_t byte_size() const noexcept { return view_.byte_size(); }
  int device_id() const noexcept { return view_.device_id(); }

  void close() noexcept { view_ = {}; }

 private:
  ros2_cuda_ipc_core::subscriber::ReadHandle view_;
};

class PyImageView {
 public:
  explicit PyImageView(ros2_cuda_ipc_core::image::ImageView view)
      : view_(std::make_unique<ros2_cuda_ipc_core::image::ImageView>(
            std::move(view))),
        valid_(view_->valid()),
        byte_size_(
            ros2_cuda_ipc_core::image::detail::DLPackImageView::byte_size(
                *view_)),
        device_id_(
            ros2_cuda_ipc_core::image::detail::DLPackImageView::device_id(
                *view_)),
        shape_(view_->shape),
        strides_(view_->strides),
        dtype_(view_->dtype),
        encoding_(view_->encoding),
        frame_id_(view_->header.frame_id) {}

  bool valid() const noexcept { return valid_; }
  uint64_t byte_size() const noexcept { return byte_size_; }
  int device_id() const noexcept { return device_id_; }
  py::tuple dlpack_device() const {
    if (!valid_) {
      throw MappingError("cannot export an invalid ImageView through DLPack");
    }
    return py::make_tuple(static_cast<int32_t>(kDLCUDA), device_id_);
  }

  std::string dtype() const {
    switch (dtype_) {
      case ros2_cuda_ipc_core::image::DType::U8:
        return "uint8";
      case ros2_cuda_ipc_core::image::DType::U16:
        return "uint16";
      case ros2_cuda_ipc_core::image::DType::F16:
        return "float16";
      case ros2_cuda_ipc_core::image::DType::F32:
        return "float32";
      case ros2_cuda_ipc_core::image::DType::F64:
        return "float64";
      case ros2_cuda_ipc_core::image::DType::S16:
        return "int16";
      case ros2_cuda_ipc_core::image::DType::S32:
        return "int32";
      case ros2_cuda_ipc_core::image::DType::U32:
        return "uint32";
    }
    throw std::logic_error("unsupported ros2_cuda_ipc image dtype");
  }

  py::tuple shape() const {
    return py::make_tuple(shape_[0], shape_[1], shape_[2]);
  }
  py::tuple strides() const {
    return py::make_tuple(strides_[0], strides_[1], strides_[2]);
  }
  const std::string& encoding() const noexcept { return encoding_; }
  const std::string& frame_id() const noexcept { return frame_id_; }

  py::capsule dlpack(std::uintptr_t stream_ptr, bool synchronize,
                     bool versioned) {
    if (!valid_) {
      throw MappingError("cannot export an invalid ImageView through DLPack");
    }
    if (consumed_) {
      throw MappingError("ImageView has already been consumed by DLPack");
    }
    if (!synchronize || stream_ptr == 0) {
      throw py::value_error(
          "DLPack export requires a consumer CUDA stream; stream=-1 is not "
          "supported");
    }

    // Validate the framework-independent layout before touching the consumer
    // stream. Invalid metadata must not enqueue a synchronization side effect.
    dlpack::ImageTensorDescriptor tensor;
    try {
      tensor = dlpack::project_to_tensor(*view_);
    } catch (const std::invalid_argument& error) {
      // The DLPack Python protocol uses BufferError for layouts that cannot
      // be represented safely as a dense strided tensor.
      throw py::buffer_error(error.what());
    }

    auto pending = std::move(view_);
    DlpackExportContext* prepared_context = nullptr;
    py::capsule capsule;
    try {
      if (versioned) {
        capsule =
            prepare_versioned_dlpack_capsule(tensor, pending, prepared_context);
      } else {
        capsule =
            prepare_legacy_dlpack_capsule(tensor, pending, prepared_context);
      }
    } catch (...) {
      view_ = std::move(pending);
      throw;
    }

    // The capsule and its managed state are already allocated.  Binding is
    // the last fallible step before publication; on failure the capsule
    // destructor drops an empty context after its publication owner is moved
    // back to this mapped object.
    try {
      bool bound = false;
      if (prepared_context != nullptr && prepared_context->owner) {
        py::gil_scoped_release release;
        bound = ros2_cuda_ipc_core::image::detail::DLPackImageView::bind(
            *prepared_context->owner, reinterpret_cast<CUstream>(stream_ptr));
      }
      if (!bound) {
        if (prepared_context != nullptr) {
          view_ = std::move(prepared_context->owner);
        }
        throw std::runtime_error("CUDA ready-event wait failed");
      }
    } catch (...) {
      if (prepared_context != nullptr && prepared_context->owner) {
        view_ = std::move(prepared_context->owner);
      }
      throw;
    }

    consumed_ = true;
    return capsule;
  }

  void close() {
    if (consumed_) {
      throw MappingError(
          "cannot close an ImageView after DLPack ownership was transferred");
    }
    view_.reset();
    valid_ = false;
  }

 private:
  std::unique_ptr<ros2_cuda_ipc_core::image::ImageView> view_;
  bool valid_ = false;
  bool consumed_ = false;
  uint64_t byte_size_ = 0;
  int device_id_ = -1;
  std::array<uint32_t, 3> shape_{};
  std::array<uint64_t, 3> strides_{};
  ros2_cuda_ipc_core::image::DType dtype_ =
      ros2_cuda_ipc_core::image::DType::U8;
  std::string encoding_;
  std::string frame_id_;
};

class PyBufferMapper {
 public:
  PyReadHandle map(const py::dict& descriptor,
                   std::uintptr_t stream_ptr) const {
    const auto message = buffer_core_from_descriptor(descriptor);
    std::optional<ros2_cuda_ipc_core::subscriber::ReadHandle> view;
    {
      py::gil_scoped_release release;
      view = mapper_.map(message, reinterpret_cast<CUstream>(stream_ptr));
    }
    if (!view) {
      throw MappingError(
          "BufferCore was rejected by the C++ mapper (lease, generation, "
          "backend, or CUDA import failure)");
    }
    return PyReadHandle(std::move(*view));
  }

 private:
  ros2_cuda_ipc_core::subscriber::BufferMapper mapper_;
};

class PyImageMapper {
 public:
  PyImageView map(const py::dict& descriptor) const {
    const auto message = gpu_image_from_descriptor(descriptor);
    validate_image_descriptor(message);

    ros2_cuda_ipc_core::image::ImageView view;
    {
      py::gil_scoped_release release;
      view = mapper_.map_for_dlpack(message);
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
      ros2_cuda_ipc_core::lease::LeaseHandle::reserve_for_publish(mapping);
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
  auto view = mapper.map_for_dlpack(message);
  if (!view.valid() || !view.sanity_check()) {
    throw std::runtime_error("test ImageViewMapper fixture failed");
  }
  if (!ros2_cuda_ipc_core::lease::LeaseHandle::commit_publish(
          mapping, reservation->slot_id, reservation->generation)) {
    (void)ros2_cuda_ipc_core::lease::LeaseHandle::cancel_publish(
        mapping, reservation->slot_id, reservation->generation);
    (void)::shm_unlink(shm_name.c_str());
    throw std::runtime_error("test LeaseHandle::commit_publish failed");
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

  py::class_<PyReadHandle>(module, "ReadHandle")
      .def_property_readonly("valid", &PyReadHandle::valid)
      .def_property_readonly("device_ptr", &PyReadHandle::device_ptr)
      .def_property_readonly("byte_size", &PyReadHandle::byte_size)
      .def_property_readonly("device_id", &PyReadHandle::device_id)
      .def("close", &PyReadHandle::close);

  py::class_<PyImageView>(module, "ImageView")
      .def_property_readonly("valid", &PyImageView::valid)
      .def_property_readonly("byte_size", &PyImageView::byte_size)
      .def_property_readonly("device_id", &PyImageView::device_id)
      .def_property_readonly("shape", &PyImageView::shape)
      .def_property_readonly("strides", &PyImageView::strides)
      .def_property_readonly("dtype", &PyImageView::dtype)
      .def_property_readonly("encoding", &PyImageView::encoding)
      .def_property_readonly("frame_id", &PyImageView::frame_id)
      .def("_dlpack_device", &PyImageView::dlpack_device)
      .def("_dlpack", &PyImageView::dlpack, py::arg("stream_ptr"),
           py::arg("synchronize"), py::arg("versioned"))
      .def("close", &PyImageView::close);

  py::class_<PyBufferMapper>(module, "BufferMapper")
      .def(py::init<>())
      .def("map", &PyBufferMapper::map, py::arg("descriptor"),
           py::arg("stream_ptr"));

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
