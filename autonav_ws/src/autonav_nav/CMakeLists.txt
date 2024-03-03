cmake_minimum_required(VERSION 3.8)
project(autonav_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Inlcude Cpp "include" directory
include_directories(include)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autonav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(scr REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
 
# Install Python programs
install(PROGRAMS
  src/path_resolver.py
  src/astar.py
  DESTINATION lib/${PROJECT_NAME}
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# C++
add_executable(astar src/astar_node.cpp src/astar.cpp)
ament_target_dependencies(astar rclcpp sensor_msgs autonav_msgs nav_msgs cv_bridge image_transport scr)

install(TARGETS
  astar
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()
ament_package()