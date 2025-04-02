import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from message_filters import Subscriber, ApproximateTimeSynchronizer

from .lanelet_wrapper import LaneletWrapper
from .util import map_to_markerarray, get_ros_param

class Routing(Node):
    def __init__(self, map_file):
        super().__init__('lanelet_routing_node')

        # Initialize wrapper with node for parameter access
        self.lanelet_wrapper = LaneletWrapper(map_file, self)

        # Declare parameters and read them
        self.declare_parameter('odom_topic', '/slam_pose')
        self.read_parameters()

        # Publishers
        self.map_pub = self.create_publisher(MarkerArray, 'Routing/Map', 10)
        self.path_pub = self.create_publisher(Path, 'Routing/Path', 10)

        # Subscribers using message_filters
        self.pose_sub = Subscriber(self, Odometry, self.odom_topic)
        self.goal_sub = Subscriber(self, PoseStamped, '/move_base_simple/goal')

        self.replan_callback = ApproximateTimeSynchronizer(
            [self.pose_sub, self.goal_sub], queue_size=10, slop=0.1)
        self.replan_callback.registerCallback(self.replan)

        # Publish map once a subscriber is available
        self.publish_map_once_connected()

    def read_parameters(self):
        self.odom_topic = get_ros_param(self, 'odom_topic', '/slam_pose')

    def publish_map_once_connected(self):
        self.get_logger().info("Waiting for RViz map subscriber...")
        timer = self.create_timer(0.5, self._try_publish_map)
        self.map_published = False

        def done():
            timer.cancel()
            self.get_logger().info("Map published to RViz")

        def try_once():
            self.publish_map()
            if self.map_pub.get_subscription_count() > 0:
                done()

        self._try_publish_map = try_once

    def publish_map(self):
        marker_array = map_to_markerarray(self.lanelet_wrapper.lanelet_map)
        self.map_pub.publish(marker_array)

    def replan(self, pose_msg, goal_msg):
        # Extract start and goal positions
        pose_x = pose_msg.pose.pose.position.x
        pose_y = pose_msg.pose.pose.position.y
        goal_x = goal_msg.pose.position.x
        goal_y = goal_msg.pose.position.y

        # Compute path
        path = self.lanelet_wrapper.get_shortest_path(pose_x, pose_y, goal_x, goal_y)
        if len(path) == 0:
            self.get_logger().warn('No path found')
            return

        # Construct and publish Path message
        path_msg = Path()
        path_msg.header = pose_msg.header

        for waypoint in path:
            pose = PoseStamped()
            pose.header = pose_msg.header
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.orientation.x = waypoint[2]  # left width
            pose.pose.orientation.y = waypoint[3]  # right width
            pose.pose.orientation.z = waypoint[4]  # speed limit
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
