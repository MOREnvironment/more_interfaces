cmake_minimum_required(VERSION 3.8)
project(vehicle_api_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# The list of files is written by tools/generate_interfaces.py (with the files themselves).
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/interface_files.cmake)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${INTERFACE_FILES}
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(consistency ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_consistency.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(defaults ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_defaults.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
