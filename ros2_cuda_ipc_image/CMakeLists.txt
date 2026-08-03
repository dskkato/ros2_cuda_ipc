cmake_minimum_required(VERSION 3.16)
project(ros2_cuda_ipc_image)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(ros2_cuda_ipc_core REQUIRED)
find_package(ros2_cuda_ipc_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/image_reader.cpp
  src/image_read_handle.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ros2_cuda_ipc_core::ros2_cuda_ipc_core
    ${ros2_cuda_ipc_msgs_TARGETS}
    ${std_msgs_TARGETS}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_image_read_handle test/test_image_read_handle.cpp)
  target_include_directories(test_image_read_handle PRIVATE test)
  target_link_libraries(test_image_read_handle ${PROJECT_NAME})

  ament_add_gtest(test_image_message_utils test/test_message_utils.cpp)
  target_link_libraries(test_image_message_utils ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(ros2_cuda_ipc_core ros2_cuda_ipc_msgs std_msgs)
ament_package()
