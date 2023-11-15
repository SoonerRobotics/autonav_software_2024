cmake_minimum_required(VERSION 3.8)
project(autonav_core_2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
ament_auto_find_build_dependencies()


# C++

include_directories(include)

ament_auto_add_library(${PROJECT_NAME} SHARED src/core.cpp)

# python
ament_python_install_package(${PROJECT_NAME})
ament_auto_package()

# testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_logger_test tests/logger_test.cpp)
  target_include_directories(${PROJECT_NAME}_logger_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_add_gtest(${PROJECT_NAME}_performance_timer_test tests/performance_timer_test.cpp src/performance_timer.cpp)
  target_include_directories(${PROJECT_NAME}_logger_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  install(TARGETS 
  ${PROJECT_NAME}_logger_test
  ${PROJECT_NAME}_performance_timer_test
  
  # Install the generated test executable file to the path after DESTINATION
  DESTINATION lib/${PROJECT_NAME}
  )
  
endif()