cmake_minimum_required(VERSION 3.5)
project(ricaip_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/AssignedTask.srv"
        "action/TaskGoTo.action"
        "action/TaskGoHome.action"
        "action/TaskTransport.action"
        DEPENDENCIES geometry_msgs
        )

ament_export_dependencies(rosidl_default_runtime)
ament_package()