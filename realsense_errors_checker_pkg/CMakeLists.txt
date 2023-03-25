cmake_minimum_required(VERSION 3.5)
project(realsense_errors_checker_pkg)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realsense2 REQUIRED)

add_executable(realsense_errors_checker_executable src/realsense_errors_checker.cpp)
add_executable(realsense_errors_checker_executable_tester src/realsense_errors_checker_tester.cpp)

ament_target_dependencies(realsense_errors_checker_executable rclcpp std_msgs realsense2)
target_link_libraries(realsense_errors_checker_executable  realsense2)


ament_target_dependencies(realsense_errors_checker_executable_tester rclcpp std_msgs realsense2)
target_link_libraries(realsense_errors_checker_executable_tester  realsense2 gtest)

install(TARGETS
  realsense_errors_checker_executable
  realsense_errors_checker_executable_tester
  DESTINATION lib/${PROJECT_NAME})

ament_package()
