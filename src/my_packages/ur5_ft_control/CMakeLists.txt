cmake_minimum_required(VERSION 3.8)
project(ur5_ft_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install directories
install(DIRECTORY
  launch
  config
  description
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()