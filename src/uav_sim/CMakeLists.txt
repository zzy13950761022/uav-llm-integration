cmake_minimum_required(VERSION 3.8)
project(uav_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# install launch files
install(
  DIRECTORY config launch meshes models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()