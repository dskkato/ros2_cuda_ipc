cmake_minimum_required(VERSION 3.16)
project(ros2_cuda_ipc_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(ros2_cuda_ipc_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/buffer_view.cpp
  src/image_view.cpp
  src/pointcloud2_view.cpp
  src/lease_handle.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rcpputils
  ros2_cuda_ipc_msgs
  sensor_msgs
  std_msgs
)

target_link_libraries(${PROJECT_NAME} CUDA::cudart)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lease_handle test/test_lease_handle.cpp)
  target_link_libraries(test_lease_handle ${PROJECT_NAME})
  ament_add_gtest(test_type_adapters test/test_type_adapters.cpp)
  target_link_libraries(test_type_adapters ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rcpputils ros2_cuda_ipc_msgs rosidl_default_runtime sensor_msgs std_msgs)
ament_package()
