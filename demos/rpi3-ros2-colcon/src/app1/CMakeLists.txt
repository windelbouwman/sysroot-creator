cmake_minimum_required(VERSION 3.5)
project(app1)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(app1 src/main.cpp)
ament_target_dependencies(app1 rclcpp)
install(TARGETS app1)

ament_package()
