import lanelet2
from lanelet2.projection import LocalCartesianProjector

# ROS2 imports for time and messages:
import rclpy
from rclpy.clock import Clock
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

'''
Utility functions for lanelet2 visualization (ROS2 version)
'''

def LineString3d_to_marker(line_string, id):
    # Helper functions to set marker appearance
    def yellow_solid(marker):
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.02
        marker.scale.y = 0.0
        marker.scale.z = 0.0
        marker.color.r = 237.0/255.0
        marker.color.g = 212.0/255.0
        marker.color.b = 0.0/255.0
        marker.color.a = 1.0

    def white_dashed(marker):
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.01
        marker.scale.y = 0.0
        marker.scale.z = 0.0
        marker.color.r = 255.0/255.0
        marker.color.g = 255.0/255.0
        marker.color.b = 255.0/255.0
        marker.color.a = 0.5

    def white_solid(marker):
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.01
        marker.scale.y = 0.0
        marker.scale.z = 0.0
        marker.color.r = 255.0/255.0
        marker.color.g = 255.0/255.0
        marker.color.b = 255.0/255.0
        marker.color.a = 1.0

    def virtual(marker):
        marker.type = Marker.POINTS
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.0
        marker.color.r = 192.0/255.0
        marker.color.g = 192.0/255.0
        marker.color.b = 192.0/255.0
        marker.color.a = 0.4

    # Main function
    marker = Marker()
    marker.header.frame_id = "map"
    # Use ROS2 clock to get current time
    marker.header.stamp = Clock().now().to_msg()
    marker.ns = "lanelet"
    marker.id = id
    # Use Marker.ADD constant for clarity (in ROS2, Marker.ADD is available)
    marker.action = Marker.ADD
    # Lifetime of zero means "forever" in ROS2 as well:
    marker.lifetime = Duration(sec=0, nanosec=0)

    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    num_pt = len(line_string)

    if line_string.attributes["type"] == "virtual":
        virtual(marker)
    elif line_string.attributes.get("subtype") == "solid_solid":
        yellow_solid(marker)
    elif line_string.attributes.get("subtype") == "dashed":
        white_dashed(marker)
    elif line_string.attributes.get("subtype") == "solid":
        white_solid(marker)
    else:
        virtual(marker)

    for i in range(num_pt):
        point = line_string[i]
        marker.points.append(Point(x=point.x, y=point.y, z=0.0))
    if num_pt % 2 == 1:
        # if the number of points is odd, add the last point once more
        point = line_string[-1]
        marker.points.append(Point(x=point.x, y=point.y, z=0.0))
    return marker

def map_to_markerarray(lanelet_map):
    i = 0
    marker_array = MarkerArray()
    linestring_layer = lanelet_map.lineStringLayer
    for line_string in linestring_layer:
        marker = LineString3d_to_marker(line_string, i)
        i += 1
        marker_array.markers.append(marker)
    return marker_array

'''
Utility functions for ROS parameter handling (ROS2 version)
Note: In ROS2 parameters are accessed via a node instance.
'''

def get_ros_param(node, param_name, default):
    """
    Read a parameter from the ROS2 parameter server via the node.
    If the parameter does not exist, return the default value.
    """
    if node.has_parameter(param_name):
        return node.get_parameter(param_name).value
    else:
        node.get_logger().warn(f"Parameter '{param_name}' not found, using default: {default}")

        return default
