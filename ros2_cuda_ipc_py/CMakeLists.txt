cmake_minimum_required(VERSION 3.16)
project(ros2_cuda_ipc_py)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED CONFIG)
find_package(ros2_cuda_ipc_core REQUIRED)
find_package(ros2_cuda_ipc_msgs REQUIRED)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)

pybind11_add_module(_native MODULE src/native_module.cpp)
target_link_libraries(_native PRIVATE ros2_cuda_ipc_core::ros2_cuda_ipc_core)
target_compile_definitions(_native PRIVATE ROS2_CUDA_IPC_PY_VERSION="0.2.0")

if(BUILD_TESTING)
  target_compile_definitions(_native PRIVATE ROS2_CUDA_IPC_PY_ENABLE_TEST_SUPPORT)
  target_include_directories(_native PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
endif()

ament_python_install_package(ros2_cuda_ipc_py)
install(TARGETS _native
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/ros2_cuda_ipc_py
  RUNTIME DESTINATION ${PYTHON_INSTALL_DIR}/ros2_cuda_ipc_py
)

install(PROGRAMS
  examples/gpu_image_subscriber.py
  examples/gpu_image_torch_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_python_api test/test_python_api.py)
endif()

ament_package()
