cmake_minimum_required(VERSION 3.8)
project(scr_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scr_msgs REQUIRED)
find_package(scr REQUIRED)

add_executable(core src/core.cpp)
ament_target_dependencies(core rclcpp scr_msgs scr)

install(TARGETS
  core
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
