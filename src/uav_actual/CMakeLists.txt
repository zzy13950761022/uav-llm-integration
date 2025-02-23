cmake_minimum_required(VERSION 3.8)
project(uav_actual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(sick_scan_xd REQUIRED)

# Instead of find_package(AriaCoda), manually locate the library.
find_library(ARIA_LIB Aria PATHS /usr/local/lib)
if(NOT ARIA_LIB)
  message(FATAL_ERROR "Could not find AriaCoda library (libAria.so) in /usr/local/lib")
endif()

# Include header directory for ARIA (AriaCoda headers)
include_directories(/usr/local/include/Aria)

# Add the pioneer driver node (ariaNode implementation)
add_executable(ariaNode src/ariaNode.cpp)
ament_target_dependencies(ariaNode rclcpp geometry_msgs)
target_link_libraries(ariaNode ${ARIA_LIB})

install(TARGETS ariaNode DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch meshes models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()