cmake_minimum_required(VERSION 3.5)
project(sample_nodes)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(gpu_buffer_publisher src/gpu_buffer_publisher.cpp)
ament_auto_add_executable(gpu_buffer_subscriber src/gpu_buffer_subscriber.cpp)

# Helper library for publisher utilities (public for reuse, not only tests)
add_library(sample_nodes_helpers STATIC src/gpu_buffer_publisher_helper.cpp)
target_include_directories(sample_nodes_helpers PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(sample_nodes_helpers ros2_cuda_ipc_core ros2_cuda_ipc_msgs)

# Optional CUDA support for demo GPU work in samples
set(SAMPLE_CUDA_LIB sample_cuda_utils)
enable_language(CUDA)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
add_library(${SAMPLE_CUDA_LIB} STATIC src/sample_cuda_utils.cu)
find_package(CUDAToolkit QUIET)
if(CUDAToolkit_FOUND AND TARGET CUDA::cudart)
  target_link_libraries(${SAMPLE_CUDA_LIB} CUDA::cudart)
endif()

if(TARGET gpu_buffer_publisher)
  target_link_libraries(gpu_buffer_publisher ${SAMPLE_CUDA_LIB} sample_nodes_helpers)
  target_include_directories(gpu_buffer_publisher PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_auto_package()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gpu_buffer_publisher_helper test/test_gpu_buffer_publisher_helper.cpp)
  if(TARGET test_gpu_buffer_publisher_helper)
    target_include_directories(test_gpu_buffer_publisher_helper PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(test_gpu_buffer_publisher_helper sample_nodes_helpers)
    ament_target_dependencies(test_gpu_buffer_publisher_helper ros2_cuda_ipc_core ros2_cuda_ipc_msgs)
  endif()
endif()
