cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PaintWaypoint.msg"
  "srv/GeneratePath.srv"
  DEPENDENCIES nav_msgs
)

ament_package()
