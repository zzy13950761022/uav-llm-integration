cmake_minimum_required(VERSION 3.8)
project(uav_actual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Locate AriaCoda manually
find_library(ARIA_LIB Aria PATHS /usr/local/lib)
if(NOT ARIA_LIB)
  message(FATAL_ERROR "Could not find AriaCoda library (libAria.so) in /usr/local/lib")
endif()

include_directories(/usr/local/include/Aria)

# Add Pioneer driver node
add_executable(driver_node src/driver_node.cpp)
ament_target_dependencies(driver_node rclcpp geometry_msgs)
target_link_libraries(driver_node ${ARIA_LIB})

install(TARGETS driver_node DESTINATION lib/${PROJECT_NAME})

ament_package()