cmake_minimum_required(VERSION 3.8)
project(autonav_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(autonav_msgs REQUIRED)
find_package(autonav_core REQUIRED)

# Install Python executables
install(PROGRAMS
  src/example.py
  DESTINATION lib/${PROJECT_NAME}
)

# C++
add_executable(example_exec src/example.cpp)
ament_target_dependencies(example_exec rclcpp autonav_msgs autonav_core)

install(TARGETS
  example_exec
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()
ament_package()