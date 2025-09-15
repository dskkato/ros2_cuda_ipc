cmake_minimum_required(VERSION 3.5)
project(sample_nodes)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Setup CUDA library for GPU utilities
set(SAMPLE_CUDA_LIB sample_cuda_utils)
enable_language(CUDA)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
add_library(${SAMPLE_CUDA_LIB} STATIC src/sample_cuda_utils.cu)
target_include_directories(${SAMPLE_CUDA_LIB} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(${SAMPLE_CUDA_LIB} ros2_cuda_ipc_core)

# Publisher node
ament_auto_add_executable(gpu_buffer_publisher src/gpu_buffer_publisher.cpp)
target_link_libraries(gpu_buffer_publisher ${SAMPLE_CUDA_LIB})

# Subscriber node
ament_auto_add_executable(gpu_buffer_subscriber src/gpu_buffer_subscriber.cpp)

ament_auto_package()

