cmake_minimum_required(VERSION 3.8)
project(autonav_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

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

ament_auto_find_build_dependencies()
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(astar src/astar.cpp)
ament_target_dependencies(core rclcpp sensor_msgs autonav_msgs nav_msgs cv_bridge image_transport scr)

install(TARGETS
	astar
	DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
	src/path_resolver.py
	src/astar.py
	src/pure_pursuit.py
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
ament_auto_package()