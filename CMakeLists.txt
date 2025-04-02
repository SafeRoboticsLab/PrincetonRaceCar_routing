cmake_minimum_required(VERSION 3.5)
project(racecar_routing)

# ----------------------------------------------------------------------------
# Required ROS 2 packages
# ----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(lanelet2 REQUIRED)

# Uncomment this block if you are using custom messages
# find_package(rosidl_default_generators REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/YourMessage.msg"
#   DEPENDENCIES std_msgs
# )

# ----------------------------------------------------------------------------
# Declare Python files as executable scripts
# ----------------------------------------------------------------------------
install(PROGRAMS
  script/routing_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------------------------------------------------------
# Install Python modules as a package (e.g., script/routing/*.py)
# This allows importing things like `from routing.lanelet_wrapper import ...`
# ----------------------------------------------------------------------------
install(DIRECTORY script/routing
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# ----------------------------------------------------------------------------
# Install any config files (e.g., map files)
# ----------------------------------------------------------------------------
install(DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)

# ----------------------------------------------------------------------------
# Standard ROS 2 package declaration
# ----------------------------------------------------------------------------
ament_package()
