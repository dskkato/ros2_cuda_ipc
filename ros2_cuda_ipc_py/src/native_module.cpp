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

#include "ros2_cuda_ipc_core/detail/read_handle_factory.hpp"
#include "ros2_cuda_ipc_core/subscriber/buffer_mapper.hpp"
#include "ros2_cuda_ipc_image/image_read_handle.hpp"
#include "ros2_cuda_ipc_py/dlpack/image_tensor_descriptor.hpp"

#ifdef ROS2_CUDA_IPC_PY_ENABLE_TEST_SUPPORT
#include <sys/mman.h>
#include <unistd.h>

#include "ros2_cuda_ipc_core/backend/memory_importer.hpp"
#include "ros2_cuda_ipc_core/buffer_metadata/buffer_metadata.hpp"
#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"
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
  message.vmm_socket_path =
      string_value(required(descriptor, "vmm_socket_path"), "vmm_socket_path");
  message.event_handle = fixed_sequence<uint8_t, 64>(
      required(descriptor, "event_handle"), "event_handle");
  message.publisher_pid = unsigned_integer<uint32_t>(
      required(descriptor, "publisher_pid"), "publisher_pid");
  message.device_id = unsigned_integer<uint32_t>(
      required(descriptor, "device_id"), "device_id");
  message.block_id =
      unsigned_integer<uint32_t>(required(descriptor, "block_id"), "block_id");
  message.uid = unsigned_integer<uint64_t>(required(descriptor, "uid"), "uid");
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
        dlpack::tensor_dl_dtype(static_cast<ros2_cuda_ipc_image::DType>(dtype));
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
  std::unique_ptr<ros2_cuda_ipc_core::subscriber::ReadHandle> owner;
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

  ros2_cuda_ipc_core::subscriber::ReadHandle release() noexcept {
    return std::move(view_);
  }

 private:
  ros2_cuda_ipc_core::subscriber::ReadHandle view_;
};

class PyImageReadHandle {
 public:
  explicit PyImageReadHandle(ros2_cuda_ipc_image::ImageReadHandle view)
      : view_(std::make_unique<ros2_cuda_ipc_image::ImageReadHandle>(
            std::move(view))),
        valid_(view_->valid()),
        byte_size_(view_->read.byte_size()),
        device_id_(view_->read.device_id()),
        shape_(view_->shape),
        strides_(view_->strides),
        dtype_(view_->dtype),
        encoding_(view_->encoding),
        frame_id_(view_->header.frame_id) {}

  static PyImageReadHandle from_message(const py::dict& descriptor,
                                        PyReadHandle& read) {
    const auto message = gpu_image_from_descriptor(descriptor);
    validate_image_descriptor(message);
    auto typed = ros2_cuda_ipc_image::ImageReadHandle::from_message(
        message, read.release());
    if (!typed) {
      throw MappingError(
          "GpuImage metadata was rejected after BufferMapper mapping");
    }
    return PyImageReadHandle(std::move(*typed));
  }

  bool valid() const noexcept { return valid_; }
  uint64_t byte_size() const noexcept { return byte_size_; }
  int device_id() const noexcept { return device_id_; }
  static std::string dtype_name(ros2_cuda_ipc_image::DType dtype) {
    switch (dtype) {
      case ros2_cuda_ipc_image::DType::U8:
        return "uint8";
      case ros2_cuda_ipc_image::DType::U16:
        return "uint16";
      case ros2_cuda_ipc_image::DType::F16:
        return "float16";
      case ros2_cuda_ipc_image::DType::F32:
        return "float32";
      case ros2_cuda_ipc_image::DType::F64:
        return "float64";
      case ros2_cuda_ipc_image::DType::S16:
        return "int16";
      case ros2_cuda_ipc_image::DType::S32:
        return "int32";
      case ros2_cuda_ipc_image::DType::U32:
        return "uint32";
    }
    throw std::logic_error("unsupported ros2_cuda_ipc image dtype");
  }
  std::string dtype() const { return dtype_name(dtype_); }

  py::tuple shape() const {
    return py::make_tuple(shape_[0], shape_[1], shape_[2]);
  }
  py::tuple strides() const {
    return py::make_tuple(strides_[0], strides_[1], strides_[2]);
  }
  const std::string& encoding() const noexcept { return encoding_; }
  const std::string& frame_id() const noexcept { return frame_id_; }

  void close() {
    view_.reset();
    valid_ = false;
  }

 private:
  std::unique_ptr<ros2_cuda_ipc_image::ImageReadHandle> view_;
  bool valid_ = false;
  uint64_t byte_size_ = 0;
  int device_id_ = -1;
  std::array<uint32_t, 3> shape_{};
  std::array<uint64_t, 3> strides_{};
  ros2_cuda_ipc_image::DType dtype_ = ros2_cuda_ipc_image::DType::U8;
  std::string encoding_;
  std::string frame_id_;
};

// Late-binding DLPack producer.  Unlike PyImageReadHandle this object never
// contains a ReadHandle before __dlpack__; publication is the sole unbound
// state and keeps the lease alive while metadata is inspected.
class PyDLPackImage {
 public:
  PyDLPackImage(
      const ros2_cuda_ipc_msgs::msg::GpuImage& message,
      std::unique_ptr<ros2_cuda_ipc_core::subscriber::detail::MappedPublication>
          publication)
      : publication_(std::move(publication)),
        shape_(message.shape),
        strides_(message.strides),
        dtype_(static_cast<ros2_cuda_ipc_image::DType>(message.dtype)),
        encoding_(message.encoding),
        frame_id_(message.header.frame_id) {
    if (!publication_ || !publication_->valid()) {
      throw MappingError("GpuImage publication is invalid");
    }
    byte_size_ = publication_->byte_size();
    device_id_ = publication_->device_id();
  }

  bool valid() const noexcept { return state_ == State::Available; }
  uint64_t byte_size() const noexcept { return byte_size_; }
  int device_id() const noexcept { return device_id_; }
  py::tuple shape() const {
    return py::make_tuple(shape_[0], shape_[1], shape_[2]);
  }
  py::tuple strides() const {
    return py::make_tuple(strides_[0], strides_[1], strides_[2]);
  }
  const std::string& encoding() const noexcept { return encoding_; }
  const std::string& frame_id() const noexcept { return frame_id_; }
  std::string dtype() const { return PyImageReadHandle::dtype_name(dtype_); }

  py::tuple dlpack_device() const {
    ensure_available("query DLPack device");
    return py::make_tuple(static_cast<int32_t>(kDLCUDA), device_id_);
  }

  py::capsule dlpack(std::uintptr_t stream_ptr, bool synchronize,
                     bool versioned) {
    ensure_available("export through DLPack");
    if (!synchronize || stream_ptr == 0) {
      throw py::value_error(
          "DLPack export requires a consumer CUDA stream; stream=-1 is not "
          "supported");
    }
    dlpack::ImageTensorDescriptor tensor;
    try {
      tensor = dlpack::project_to_tensor(dtype_, shape_, strides_,
                                         publication_->device_ptr(), byte_size_,
                                         device_id_);
    } catch (const std::invalid_argument& error) {
      throw py::buffer_error(error.what());
    }

    // Allocate all capsule objects first. make_bound retains publication on
    // every failure before its ownership commit.
    auto capsule_context = std::make_unique<DlpackExportContext>();
    capsule_context->tensor = tensor;
    std::unique_ptr<DLManagedTensor> legacy;
    std::unique_ptr<DLManagedTensorVersioned> versioned_managed;
    if (versioned) {
      versioned_managed = std::make_unique<DLManagedTensorVersioned>();
      versioned_managed->version = {1, 0};
      versioned_managed->flags = 0;
    } else {
      legacy = std::make_unique<DLManagedTensor>();
    }

    auto read =
        ros2_cuda_ipc_core::subscriber::detail::ReadHandleFactory::make_bound(
            *publication_, reinterpret_cast<CUstream>(stream_ptr));
    if (!read)
      throw MappingError(
          "failed to bind GpuImage to the DLPack consumer stream");
    capsule_context->owner =
        std::make_unique<ros2_cuda_ipc_core::subscriber::ReadHandle>(
            std::move(*read));

    // make_bound committed the publication. If PyCapsule creation fails, the
    // producer cannot be retried and must remain in the safe Closed state.
    publication_.reset();
    state_ = State::Closed;

    // No fallible allocation follows the ownership commit. PyCapsule_New is
    // the only remaining fallible operation; its failure destroys read safely.
    if (versioned) {
      versioned_managed->manager_ctx = capsule_context.get();
      versioned_managed->deleter = &versioned_dlpack_deleter;
      populate_dlpack_tensor(*versioned_managed, *capsule_context);
      auto* raw = versioned_managed.release();
      py::capsule result;
      try {
        result =
            py::capsule(raw, kVersionedDLPackName, &dlpack_capsule_destructor);
      } catch (...) {
        raw->deleter(raw);
        throw;
      }
      (void)capsule_context.release();
      state_ = State::Consumed;
      return result;
    }
    legacy->manager_ctx = capsule_context.get();
    legacy->deleter = &legacy_dlpack_deleter;
    populate_dlpack_tensor(*legacy, *capsule_context);
    auto* raw = legacy.release();
    py::capsule result;
    try {
      result = py::capsule(raw, kDLPackName, &dlpack_capsule_destructor);
    } catch (...) {
      raw->deleter(raw);
      throw;
    }
    (void)capsule_context.release();
    state_ = State::Consumed;
    return result;
  }

  void close() {
    ensure_available("close");
    publication_.reset();
    state_ = State::Closed;
  }

 private:
  enum class State { Available, Consumed, Closed };
  void ensure_available(const char* action) const {
    if (state_ == State::Consumed)
      throw MappingError(std::string("cannot ") + action +
                         " after DLPack ownership was transferred");
    if (state_ == State::Closed)
      throw MappingError(std::string("cannot ") + action + " after close");
  }
  std::unique_ptr<ros2_cuda_ipc_core::subscriber::detail::MappedPublication>
      publication_;
  State state_ = State::Available;
  uint64_t byte_size_ = 0;
  int device_id_ = -1;
  std::array<uint32_t, 3> shape_{};
  std::array<uint64_t, 3> strides_{};
  ros2_cuda_ipc_image::DType dtype_;
  std::string encoding_, frame_id_;
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
          "BufferCore was rejected by the C++ mapper (buffer reference, "
          "uid, "
          "VMM-FD import failure)");
    }
    return PyReadHandle(std::move(*view));
  }

 private:
  ros2_cuda_ipc_core::subscriber::BufferMapper mapper_;
};

class PyImageMapper {
 public:
  PyDLPackImage map(const py::dict& descriptor) const {
    const auto message = gpu_image_from_descriptor(descriptor);
    validate_image_descriptor(message);
    std::unique_ptr<ros2_cuda_ipc_core::subscriber::detail::MappedPublication>
        publication;
    {
      py::gil_scoped_release release;
      publication = ros2_cuda_ipc_core::subscriber::detail::acquire_publication(
          mapper_, message.core);
    }
    if (!publication)
      throw MappingError(
          "GpuImage was rejected while acquiring its publication");
    return PyDLPackImage(message, std::move(publication));
  }

 private:
  ros2_cuda_ipc_core::subscriber::BufferMapper mapper_;
};

#ifdef ROS2_CUDA_IPC_PY_ENABLE_TEST_SUPPORT

class TestBufferRefProbe {
 public:
  TestBufferRefProbe(
      std::shared_ptr<ros2_cuda_ipc_core::buffer_metadata::BufferMetadata>
          mapping,
      std::string shm_name)
      : mapping_(std::move(mapping)), shm_name_(std::move(shm_name)) {}

  ~TestBufferRefProbe() {
    if (!shm_name_.empty()) {
      (void)::shm_unlink(shm_name_.c_str());
    }
  }

  uint32_t refcount() const {
    const auto value =
        ros2_cuda_ipc_core::buffer_metadata::BufferRef::current_refcount(
            mapping_);
    return value.value_or(0);
  }

  const std::string& shm_name() const noexcept { return shm_name_; }

 private:
  std::shared_ptr<ros2_cuda_ipc_core::buffer_metadata::BufferMetadata> mapping_;
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
  const uint32_t publisher_pid = static_cast<uint32_t>(::getpid());
  const uint32_t block_id = static_cast<uint32_t>(counter.fetch_add(1));
  const std::string shm_name =
      ros2_cuda_ipc_core::buffer_metadata::shm_name_for_block(publisher_pid,
                                                              block_id);
  auto mapping =
      ros2_cuda_ipc_core::buffer_metadata::BufferMetadata::create(shm_name);
  if (!mapping) {
    throw std::runtime_error("test BufferMetadata::create failed");
  }
  const auto reservation =
      ros2_cuda_ipc_core::buffer_metadata::BufferRef::reserve_for_publish(
          mapping);
  if (!reservation) {
    throw std::runtime_error("test BufferRef::reserve_for_publish failed");
  }

  ros2_cuda_ipc_msgs::msg::GpuImage message;
  message.header.frame_id = "test_frame";
  message.dtype = static_cast<uint8_t>(ros2_cuda_ipc_image::DType::U8);
  message.shape = {2, 3, 4};
  message.strides = {12, 4, 1};
  message.encoding = "rgba8";
  message.core.event_handle.fill(0);
  message.core.event_handle[0] = 18;
  message.core.vmm_socket_path =
      "/tmp/ros2_cuda_ipc_test_socket_12345678901234567890.sock";
  message.core.publisher_pid = publisher_pid;
  message.core.device_id = 0;
  message.core.block_id = block_id;
  message.core.uid = reservation->uid;
  message.core.byte_size = 24;

  ros2_cuda_ipc_core::subscriber::IpcHandleKey key{};
  key.publisher_pid = publisher_pid;
  key.block_id = block_id;
  key.device_id = message.core.device_id;
  key.vmm_socket_path = message.core.vmm_socket_path;
  key.event = message.core.event_handle;
  ros2_cuda_ipc_core::backend::ImportedResources imported;
  imported.dev_ptr = reinterpret_cast<void*>(static_cast<uintptr_t>(0x100000));
  // A null event makes this deterministic, while the normal native wait path
  // still receives and forwards the requested stream pointer.
  (void)ros2_cuda_ipc_core::subscriber::IpcHandleCache::instance()
      .insert_or_discard_duplicate(key, std::move(imported));

  ros2_cuda_ipc_core::subscriber::BufferMapper mapper;
  auto read = mapper.map(message.core, CU_STREAM_LEGACY);
  if (!read) {
    throw std::runtime_error("test BufferMapper fixture failed");
  }
  auto view = ros2_cuda_ipc_image::ImageReadHandle::from_message(
      message, std::move(*read));
  if (!view) {
    throw std::runtime_error("test ImageReadHandle fixture failed");
  }
  if (!ros2_cuda_ipc_core::buffer_metadata::BufferRef::commit_publish(
          mapping, reservation->uid)) {
    (void)ros2_cuda_ipc_core::buffer_metadata::BufferRef::cancel_publish(
        mapping, reservation->uid);
    (void)::shm_unlink(shm_name.c_str());
    throw std::runtime_error("test BufferRef::commit_publish failed");
  }

  py::dict core;
  core["vmm_socket_path"] = message.core.vmm_socket_path;
  core["event_handle"] = bytes_to_list(message.core.event_handle);
  core["publisher_pid"] = message.core.publisher_pid;
  core["device_id"] = message.core.device_id;
  core["block_id"] = message.core.block_id;
  core["uid"] = message.core.uid;
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

  auto probe = std::make_shared<TestBufferRefProbe>(mapping, shm_name);
  return py::make_tuple(PyImageReadHandle(std::move(*view)), probe, descriptor);
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

  py::class_<PyImageReadHandle>(module, "ImageReadHandle")
      .def_property_readonly("valid", &PyImageReadHandle::valid)
      .def_property_readonly("byte_size", &PyImageReadHandle::byte_size)
      .def_property_readonly("device_id", &PyImageReadHandle::device_id)
      .def_property_readonly("shape", &PyImageReadHandle::shape)
      .def_property_readonly("strides", &PyImageReadHandle::strides)
      .def_property_readonly("dtype", &PyImageReadHandle::dtype)
      .def_property_readonly("encoding", &PyImageReadHandle::encoding)
      .def_property_readonly("frame_id", &PyImageReadHandle::frame_id)
      .def_static("from_message", &PyImageReadHandle::from_message,
                  py::arg("message"), py::arg("read"))
      .def("close", &PyImageReadHandle::close);

  py::class_<PyBufferMapper>(module, "BufferMapper")
      .def(py::init<>())
      .def("map", &PyBufferMapper::map, py::arg("descriptor"),
           py::arg("stream_ptr"));

  py::class_<PyDLPackImage>(module, "DLPackImage")
      .def_property_readonly("valid", &PyDLPackImage::valid)
      .def_property_readonly("byte_size", &PyDLPackImage::byte_size)
      .def_property_readonly("device_id", &PyDLPackImage::device_id)
      .def_property_readonly("shape", &PyDLPackImage::shape)
      .def_property_readonly("strides", &PyDLPackImage::strides)
      .def_property_readonly("dtype", &PyDLPackImage::dtype)
      .def_property_readonly("encoding", &PyDLPackImage::encoding)
      .def_property_readonly("frame_id", &PyDLPackImage::frame_id)
      .def("_dlpack_device", &PyDLPackImage::dlpack_device)
      .def("_dlpack", &PyDLPackImage::dlpack, py::arg("stream_ptr"),
           py::arg("synchronize"), py::arg("versioned"))
      .def("close", &PyDLPackImage::close);

  py::class_<PyImageMapper>(module, "ImageMapper")
      .def(py::init<>())
      .def("map", &PyImageMapper::map, py::arg("descriptor"));

#ifdef ROS2_CUDA_IPC_PY_ENABLE_TEST_SUPPORT
  py::class_<TestBufferRefProbe, std::shared_ptr<TestBufferRefProbe>>(
      module, "_TestBufferRefProbe")
      .def("refcount", &TestBufferRefProbe::refcount)
      .def_property_readonly("shm_name", &TestBufferRefProbe::shm_name);
  module.def("_make_test_image", &make_test_image);
#endif
}
