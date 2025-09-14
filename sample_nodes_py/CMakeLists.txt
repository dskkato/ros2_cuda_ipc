cmake_minimum_required(VERSION 3.5)
project(sample_nodes_py)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_pool_flow test/test_sample_nodes_py.py)
endif()

ament_package()
