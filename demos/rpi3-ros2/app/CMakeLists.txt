
cmake_minimum_required(VERSION 3.14)
project(
    "example_app"
    DESCRIPTION "An example for cross compilation using ROS2."
)

find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
set(msg_files
  "msg/ThunderStruck.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

add_executable(app main.cpp)
ament_target_dependencies(app rclcpp)


rosidl_target_interfaces(
  app
  example_app
  "rosidl_typesupport_cpp"
)

install(TARGETS app)

ament_package()
