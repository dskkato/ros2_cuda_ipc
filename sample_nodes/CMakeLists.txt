cmake_minimum_required(VERSION 3.16)
project(sample_nodes)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_cuda_ipc_core REQUIRED)
find_package(ros2_cuda_ipc_msgs REQUIRED)
find_package(CUDAToolkit REQUIRED)

add_library(sample_nodes_helpers
  src/gpu_image_publisher_helper.cpp
  src/gpu_pointcloud_publisher_helper.cpp
)

target_include_directories(sample_nodes_helpers
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(sample_nodes_helpers
  rclcpp
  sensor_msgs
  ros2_cuda_ipc_core
  ros2_cuda_ipc_msgs
)

target_link_libraries(sample_nodes_helpers
  CUDA::cudart
)

add_executable(gpu_image_publisher src/gpu_image_publisher.cpp)
target_link_libraries(gpu_image_publisher sample_nodes_helpers)

add_executable(gpu_image_subscriber src/gpu_image_subscriber.cpp)
target_link_libraries(gpu_image_subscriber sample_nodes_helpers)

add_executable(gpu_pointcloud_publisher src/gpu_pointcloud_publisher.cpp)
target_link_libraries(gpu_pointcloud_publisher sample_nodes_helpers)

add_executable(gpu_pointcloud_subscriber src/gpu_pointcloud_subscriber.cpp)
target_link_libraries(gpu_pointcloud_subscriber sample_nodes_helpers)

ament_target_dependencies(gpu_image_publisher
  rclcpp
  sensor_msgs
  ros2_cuda_ipc_core
  ros2_cuda_ipc_msgs
)

ament_target_dependencies(gpu_image_subscriber
  rclcpp
  sensor_msgs
  ros2_cuda_ipc_core
  ros2_cuda_ipc_msgs
)

ament_target_dependencies(gpu_pointcloud_publisher
  rclcpp
  sensor_msgs
  ros2_cuda_ipc_core
  ros2_cuda_ipc_msgs
)

ament_target_dependencies(gpu_pointcloud_subscriber
  rclcpp
  sensor_msgs
  ros2_cuda_ipc_core
  ros2_cuda_ipc_msgs
)

install(TARGETS
  sample_nodes_helpers
  gpu_image_publisher
  gpu_image_subscriber
  gpu_pointcloud_publisher
  gpu_pointcloud_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(rclcpp sensor_msgs ros2_cuda_ipc_core ros2_cuda_ipc_msgs)

ament_package()
