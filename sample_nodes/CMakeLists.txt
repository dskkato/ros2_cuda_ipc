cmake_minimum_required(VERSION 3.5)
project(sample_nodes)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_cuda_ipc_msgs REQUIRED)
find_package(ros2_cuda_ipc_core REQUIRED)

add_executable(gpu_buffer_publisher src/gpu_buffer_publisher.cpp)
ament_target_dependencies(gpu_buffer_publisher rclcpp ros2_cuda_ipc_msgs ros2_cuda_ipc_core)

add_executable(gpu_buffer_subscriber src/gpu_buffer_subscriber.cpp)
ament_target_dependencies(gpu_buffer_subscriber rclcpp ros2_cuda_ipc_msgs ros2_cuda_ipc_core)

install(TARGETS gpu_buffer_publisher gpu_buffer_subscriber DESTINATION lib/${PROJECT_NAME})

ament_package()
