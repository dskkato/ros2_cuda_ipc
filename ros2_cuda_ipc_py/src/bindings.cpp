#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "ros2_cuda_ipc_core/gpu_buffer_pool.hpp"
#include "ros2_cuda_ipc_core/gpu_buffer_mapper.hpp"

namespace py = pybind11;
namespace rcc = ros2_cuda_ipc_core;

void init_bindings(py::module_ &m) {
  py::class_<rcc::GpuBufferPool>(m, "GpuBufferPool")
      .def(py::init<std::size_t>())
      .def(py::init<std::size_t, std::size_t, bool>())
      .def("borrow", &rcc::GpuBufferPool::borrow)
      .def("release", &rcc::GpuBufferPool::release)
      .def("capacity", &rcc::GpuBufferPool::capacity);

  py::class_<rcc::GpuBufferMapper>(m, "GpuBufferMapper")
      .def(py::init<>())
      .def("open_memory", &rcc::GpuBufferMapper::open_memory,
           py::return_value_policy::reference)
      .def("open_event", &rcc::GpuBufferMapper::open_event,
           py::return_value_policy::reference)
      .def("get_memory", &rcc::GpuBufferMapper::get_memory,
           py::return_value_policy::reference)
      .def("get_event", &rcc::GpuBufferMapper::get_event,
           py::return_value_policy::reference)
      .def("wait_ready", &rcc::GpuBufferMapper::wait_ready)
      .def("close_slot", &rcc::GpuBufferMapper::close_slot)
      .def("reset", &rcc::GpuBufferMapper::reset);
}
