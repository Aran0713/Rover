cmake_minimum_required(VERSION 3.5)
project(go_to_service)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Generate interfaces from .action and .srv
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DriveDistance.action"
  "srv/GoToDistance.srv"
)

ament_export_dependencies(rosidl_default_runtime)

# Install the Python executables (your scripts)
install(PROGRAMS
  go_to_service/drive_distance_server.py
  go_to_service/go_to_service.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
