cmake_minimum_required(VERSION 3.8)
project(hardware_drivers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Locate AriaCoda library
find_library(ARIA_LIB Aria PATHS /usr/local/lib)
if(NOT ARIA_LIB)
  message(FATAL_ERROR "Could not find AriaCoda library (libAria.so) in /usr/local/lib")
endif()

include_directories(/usr/local/include/Aria)

# Add ariaNode executable
add_executable(aria_node src/ariaNode.cpp)
ament_target_dependencies(aria_node rclcpp geometry_msgs)
target_link_libraries(aria_node ${ARIA_LIB})

install(TARGETS aria_node DESTINATION lib/${PROJECT_NAME})

ament_package()