cmake_minimum_required(VERSION 3.8)
project(realsense_health_checker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



add_executable(realsense_health_checker_node 
src/realsense_health_checker_node.cpp
src/realsense_health_checker.cpp
)


include_directories(include params)
ament_target_dependencies(realsense_health_checker_node rclcpp std_msgs diagnostic_msgs)


install(TARGETS 
  realsense_health_checker_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

ament_package()
