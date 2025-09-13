cmake_minimum_required(VERSION 3.5)
project(sample_nodes)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(gpu_buffer_publisher src/gpu_buffer_publisher.cpp)
ament_auto_add_executable(gpu_buffer_subscriber src/gpu_buffer_subscriber.cpp)

ament_auto_package()
