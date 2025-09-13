cmake_minimum_required(VERSION 3.5)
project(ros2_cuda_ipc_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# Always build with CUDA support (simplified core)
enable_language(CUDA)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
find_package(CUDAToolkit REQUIRED)

set(SOURCES
  src/gpu_buffer_pool.cpp
  src/gpu_buffer_mapper.cpp
  src/cuda_support.cu
)

add_library(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(${PROJECT_NAME} PUBLIC ROS2_CUDA_IPC_WITH_CUDA=1)
target_link_libraries(${PROJECT_NAME} CUDA::cudart)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gpu_buffer_pool test/test_gpu_buffer_pool.cpp)
  if(TARGET test_gpu_buffer_pool)
    target_link_libraries(test_gpu_buffer_pool ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_cuda_support test/test_cuda_support.cpp)
  if(TARGET test_cuda_support)
    target_link_libraries(test_cuda_support ${PROJECT_NAME})
    set_tests_properties(test_cuda_support PROPERTIES LABELS "cuda")
  endif()

  ament_add_gtest(test_gpu_buffer_mapper test/test_gpu_buffer_mapper.cpp)
  if(TARGET test_gpu_buffer_mapper)
    target_link_libraries(test_gpu_buffer_mapper ${PROJECT_NAME})
    set_tests_properties(test_gpu_buffer_mapper PROPERTIES LABELS "cuda")
  endif()
  # Note: process‑separated IPC tests removed for portability
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(CUDAToolkit)
ament_package()
