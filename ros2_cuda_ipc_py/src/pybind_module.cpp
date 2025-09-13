#include <pybind11/pybind11.h>

namespace py = pybind11;

int add(int a, int b) {
  return a + b;
}

PYBIND11_MODULE(_cuda_ipc, m) {
  m.def("add", &add, "Add two integers");
}
