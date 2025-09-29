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
find_package(OpenCV REQUIRED COMPONENTS imgcodecs)

add_library(gpu_image_transport_base
  src/gpu_image_transport_base.cpp
)

target_include_directories(gpu_image_transport_base
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CUDAToolkit_INCLUDE_DIRS}
)

target_link_libraries(gpu_image_transport_base
  CUDA::cudart
)

ament_target_dependencies(gpu_image_transport_base
  rclcpp
  ros2_cuda_ipc_core
  ros2_cuda_ipc_msgs
)

add_executable(gpu_image_transport
  src/gpu_image_transport.cpp
)

target_link_libraries(gpu_image_transport
  gpu_image_transport_base
)

ament_target_dependencies(gpu_image_transport
  rclcpp
  sensor_msgs
  ros2_cuda_ipc_core
  ros2_cuda_ipc_msgs
)

add_executable(gpu_image_transport_compressed
  src/gpu_image_transport_compressed.cpp
)

target_include_directories(gpu_image_transport_compressed
  PRIVATE
    ${CUDAToolkit_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(gpu_image_transport_compressed
  gpu_image_transport_base
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(gpu_image_transport_compressed
  rclcpp
  sensor_msgs
  ros2_cuda_ipc_core
  ros2_cuda_ipc_msgs
)

install(TARGETS
  gpu_image_transport_base
  gpu_image_transport
  gpu_image_transport_compressed
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
