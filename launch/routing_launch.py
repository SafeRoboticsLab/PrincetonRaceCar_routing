from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare configurable launch arguments
    return LaunchDescription([
        DeclareLaunchArgument(
            'output',
            default_value='screen',
            description='Display output to screen or log file'
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/sim_pose',
            description='ROS topic for the pose'
        ),
        DeclareLaunchArgument(
            'lane_change_cost',
            default_value='1.0',
            description='Cost of lane change'
        ),
        DeclareLaunchArgument(
            'map_file',
            default_value=os.path.join(
                get_package_share_directory('racecar_routing'),
                'cfg',
                'track.osm'
            ),
            description='Path to the lanelet .osm map file'
        ),

        # Routing Node
        Node(
            package='racecar_routing',
            executable='routing_node',
            name='routing_node',
            output=LaunchConfiguration('output'),
            parameters=[
                {'odom_topic': LaunchConfiguration('odom_topic')},
                {'lane_change_cost': LaunchConfiguration('lane_change_cost')},
                {'map_file': LaunchConfiguration('map_file')}
            ]
        )
    ])
