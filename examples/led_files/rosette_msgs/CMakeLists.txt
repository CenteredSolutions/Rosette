cmake_minimum_required(VERSION 3.8)
project(rosette_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetLed.srv"
  "srv/PutFile.srv"
  "srv/GetFile.srv"
  "srv/ListFiles.srv"
  "srv/EraseFile.srv"
)

ament_package()
