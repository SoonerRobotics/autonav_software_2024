cmake_minimum_required(VERSION 3.8)
project(autonav_display)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scr_msgs REQUIRED)
find_package(scr REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(display src/display.cpp)
ament_target_dependencies(display rclcpp scr_msgs scr sensor_msgs)

install(TARGETS
  display
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
