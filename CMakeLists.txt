# ----------------------------------------------------------------------------
# Set the minimum CMake version and declare the project name.
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.5)
project(racecar_routing)

# ----------------------------------------------------------------------------
# Use ament_cmake as the build system for ROS2 instead of catkin (ROS1).
# ----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)

# ----------------------------------------------------------------------------
# For Python nodes in ROS2, use rclpy (instead of rospy in ROS1).
# ----------------------------------------------------------------------------
find_package(rclpy REQUIRED)

# ----------------------------------------------------------------------------
# std_msgs and tf2 are available in ROS2. They are needed for standard messages
# and transformation handling respectively.
# ----------------------------------------------------------------------------
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

# ----------------------------------------------------------------------------
# dynamic_reconfigure is a ROS1 package for runtime parameter tuning.
# In ROS2, parameter handling is built into rclpy/rclcpp so we omit it.
# ----------------------------------------------------------------------------
# (dynamic_reconfigure not needed in ROS2)

# ----------------------------------------------------------------------------
# If your package defines custom messages, services, or actions, use the ROS2 
# message generation system. The rosidl_default_generators package replaces 
# message_generation/message_runtime from ROS1.
#
# Uncomment and update the following lines if you have any custom message files.
# For example, if you have a message file in msg/YourMessage.msg:
#
 find_package(rosidl_default_generators REQUIRED)
#
 rosidl_generate_interfaces(${PROJECT_NAME}
   "msg/YourMessage.msg"
   DEPENDENCIES std_msgs  # Add additional dependencies if needed
# )
#
# ----------------------------------------------------------------------------
# (For now, we assume no custom messages are being generated, so this section remains commented.)

# ----------------------------------------------------------------------------
# lanelet2 is assumed to have a ROS2-compatible version. Include it as a dependency.
# ----------------------------------------------------------------------------
find_package(lanelet2 REQUIRED)

# ----------------------------------------------------------------------------
# Declare the package with ament_package() which replaces catkin_package() in ROS2.
# ----------------------------------------------------------------------------
ament_package()

# ----------------------------------------------------------------------------
# Installation instructions:
#
# 1. Install the Python node executable.
#    In ROS2, executables are conventionally installed in the lib/<project> directory.
#
# 2. Install the configuration files (the 'cfg' directory) so that they are available
#    in the share/<project>/cfg directory.
# ----------------------------------------------------------------------------
install(PROGRAMS
  script/routing_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)
