cmake_minimum_required(VERSION 3.5)
project(ros2_cuda_ipc_py)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 REQUIRED)
find_package(ros2_cuda_ipc_core REQUIRED)

ament_python_install_package(${PROJECT_NAME})

pybind11_add_module(_cuda_ipc src/pybind_module.cpp src/bindings.cpp src/dlpack_utils.cpp)
target_link_libraries(_cuda_ipc PRIVATE ros2_cuda_ipc_core::ros2_cuda_ipc_core)
ament_target_dependencies(_cuda_ipc ros2_cuda_ipc_core)

install(TARGETS _cuda_ipc EXPORT export_${PROJECT_NAME}
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ros2_cuda_ipc_core)

ament_package()
