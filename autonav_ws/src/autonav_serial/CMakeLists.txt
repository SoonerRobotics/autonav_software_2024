cmake_minimum_required(VERSION 3.8)
project(autonav_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(scr REQUIRED)
find_package(scr_msgs REQUIRED)
find_package(autonav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
	src/serial_node.py
	src/camera.py
	src/safety_lights.py
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()