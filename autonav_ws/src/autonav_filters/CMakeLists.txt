cmake_minimum_required(VERSION 3.8)
project(autonav_filters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autonav_msgs REQUIRED)
find_package(scr REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# msgs and srvs 
#rosidl_generate_interfaces(${PROJECT_NAME}
  # add message types here
  #"path_to_message/message.msg"
#)

# C++

# Inlcude Cpp "include" directory
include_directories(include)

# add Cpp libraries


# Create Cpp executables
#add_executable(executable_name /path_to_executable/executable.cpp)
#ament_target_dependencies(executable_name rclcpp other_dependencies)
add_executable(filters_node src/filters_node.cpp src/filters.cpp)
ament_target_dependencies(filters_node rclcpp autonav_msgs scr)

add_executable(motor_feedback_example_node src/motor_feedback_example_node.cpp src/motor_feedback_example.cpp)
ament_target_dependencies(motor_feedback_example_node rclcpp autonav_msgs std_msgs)

# Install Cpp executables
install(TARGETS
  # install executables by name
  #executable_name
  filters_node
  motor_feedback_example_node
  DESTINATION lib/${PROJECT_NAME}
)
 
# Python

 # Use only if not using rosidl_generate_interfaces
 # Install Python modules
 #ament_python_install_package(${PROJECT_NAME})
 
 # Install Python programs
 install(PROGRAMS
   # add programs in format:
   #/path_to_program/program.py
   DESTINATION lib/${PROJECT_NAME}
  )


if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Add gtest executables
  ament_add_gtest(${PROJECT_NAME}_motor_feedback_test tests/motor_feedback_test.cpp src/motor_feedback_example.cpp)
  target_include_directories(${PROJECT_NAME}_motor_feedback_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_motor_feedback_test
    rclcpp
    autonav_msgs
  )
  ament_add_gtest(${PROJECT_NAME}_particle_filter_test tests/particle_filter_test.cpp)
  target_include_directories(${PROJECT_NAME}_motor_feedback_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_particle_filter_test
    rclcpp
    autonav_msgs
  )

  install(TARGETS
    ${PROJECT_NAME}_motor_feedback_test
    ${PROJECT_NAME}_particle_filter_test
    # Install the generated test executable file to the path after DESTINATION
    DESTINATION lib/${PROJECT_NAME}
  )

endif()

ament_package()