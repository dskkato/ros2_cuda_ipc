cmake_minimum_required(VERSION 3.16)
project(gpu_image_transport)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_cuda_ipc_core REQUIRED)
find_package(ros2_cuda_ipc_msgs REQUIRED)
find_package(CUDAToolkit REQUIRED)

add_executable(gpu_image_transport
  src/gpu_image_transport.cpp
)

target_include_directories(gpu_image_transport
  PRIVATE
    ${CUDAToolkit_INCLUDE_DIRS}
)

target_link_libraries(gpu_image_transport
  CUDA::cudart
)

ament_target_dependencies(gpu_image_transport
  rclcpp
  sensor_msgs
  ros2_cuda_ipc_core
  ros2_cuda_ipc_msgs
)

install(TARGETS
  gpu_image_transport
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
