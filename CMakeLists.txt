cmake_minimum_required(VERSION 3.8)
project(snaak_weight_read)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ReadWeight.srv"
  DEPENDENCIES std_msgs
)

# Ensure the service can be used from C++
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/snaak_weight_service_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
