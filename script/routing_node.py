#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from routing import Routing

def main(args=None):
    # === ROS 2: Initialize the rclpy library (equivalent to rospy.init_node in ROS 1) ===
    rclpy.init(args=args)

    # === ROS 2: Create a temporary node to declare and fetch parameters (no global get_param like in rospy) ===
    temp_node = Node('routing_node_param_loader')
    temp_node.declare_parameter('map_file')  # ROS 2: parameters must be explicitly declared
    map_file = temp_node.get_parameter('map_file').get_parameter_value().string_value
    temp_node.destroy_node()  # Clean up temporary node after retrieving the parameter

    # === ROS 2: Instantiate the main Routing node, passing the map file ===
    routing_node = Routing(map_file)

    # === ROS 2: Use node.get_logger().info() instead of rospy.loginfo ===
    routing_node.get_logger().info("Started Routing Node")

    # === ROS 2: Spin the node (equivalent to rospy.spin in ROS 1) ===
    rclpy.spin(routing_node)

    # === ROS 2: Shutdown procedure ===
    routing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
