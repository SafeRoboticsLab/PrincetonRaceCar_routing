import lanelet2
from lanelet2.core import BasicPoint2d, BasicPoint3d
from lanelet2.projection import LocalCartesianProjector
from lanelet2.geometry import to2D, toArcCoordinates, fromArcCoordinates, ArcCoordinates

from .util import get_ros_param
import numpy as np
import rclpy

'''
This class is a wrapper for lanelet2 objects.
It uses the lanelet2 Python API to provide a more convenient interface using generic data types.
Now fully migrated to ROS 2 and compatible with ROS 2 node-based parameter handling.
'''
class LaneletWrapper:
    def __init__(self, osm_file, node) -> None:
        # Store the node handle for logging and parameter access
        self.node = node

        # Read ROS2 parameters from the node
        self.read_parameters()

        # Create a local cartesian projector from (0, 0, 0)
        self.projector = LocalCartesianProjector(lanelet2.io.Origin(0, 0, 0))

        # Load lanelet map
        self.lanelet_map = self.load_lanelet_map(osm_file)
        self.lanelet_layer = self.lanelet_map.laneletLayer

        # Set up traffic rules and routing graph
        self.traffic_rules = lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany,
            lanelet2.traffic_rules.Participants.Vehicle
        )

        routing_cost = [
            lanelet2.routing.RoutingCostDistance(self.lane_change_cost),
            lanelet2.routing.RoutingCostTravelTime(self.lane_change_cost)
        ]

        self.routing_graph = lanelet2.routing.RoutingGraph(
            self.lanelet_map, self.traffic_rules, routing_cost)

        # Pre-cache all centerlines to avoid lazy evaluation delays
        self.warmup()

    def read_parameters(self):
        '''
        Read parameters from the ROS 2 parameter server using the provided node.
        '''
        self.lane_change_cost = get_ros_param(self.node, 'lane_change_cost', 1.0)

    def find_lanelet_by_xy(self, pt):
        '''
        Given a 2D point, find the nearest lanelet in the map.
        Returns:
            dis: distance to the nearest lanelet (0 if inside)
            lanelet: nearest lanelet object
        '''
        return lanelet2.geometry.findNearest(self.lanelet_layer, pt, 1)[0]

    def get_shortest_path(self, x_start, y_start, x_end, y_end, allow_lane_change=True):
        '''
        Computes the shortest path from start (x_start, y_start) to end (x_end, y_end)
        using the Lanelet2 routing graph.
        Returns:
            path_centerline: list of waypoints representing the path centerline
        '''
        path_centerline = []
        start_point = BasicPoint2d(float(x_start), float(y_start))
        end_point = BasicPoint2d(float(x_end), float(y_end))

        # Find nearest lanelets to start and end
        dis_to_start, start_lanelet = self.find_lanelet_by_xy(start_point)
        dis_to_end, end_lanelet = self.find_lanelet_by_xy(end_point)

        relation = self.routing_graph.routingRelation(start_lanelet, end_lanelet, True)

        # Handle special case when start and end are in the same or neighboring lanelet
        if start_lanelet.id == end_lanelet.id or \
            relation in [lanelet2.routing.RelationType.Left, lanelet2.routing.RelationType.Right]:

            cl = to2D(start_lanelet.centerline)
            cl_length = lanelet2.geometry.length(cl)
            dis_start = self.get_dis_from_point(cl, start_point.x, start_point.y) / cl_length
            dis_end = self.get_dis_from_point(cl, end_point.x, end_point.y) / cl_length
            if dis_end < dis_start:
                path_centerline.extend(self.get_path_centerline([start_lanelet], start_point, None, False))
                start_lanelet = self.routing_graph.following(start_lanelet, False)[0]
                start_point = None
            elif dis_end == dis_start:
                return []  # Same point, no path needed

        # Warn if starting/ending point is outside any lanelet
        if dis_to_start > 0:
            self.node.get_logger().warn(f"Start point [{x_start}, {y_start}] is not inside a lanelet")

        if dis_to_end > 0:
            self.node.get_logger().warn(f"End point [{x_end}, {y_end}] is not inside a lanelet")

        # Get the shortest path using the routing graph
        path = self.routing_graph.shortestPath(start_lanelet, end_lanelet, 0, allow_lane_change)
        path_centerline.extend(self.get_path_centerline(path, start_point, end_point, allow_lane_change))
        return path_centerline

    def get_path_centerline(self, path, start_point=None, end_point=None, allow_lane_change=True):
        '''
        Converts a list of lanelets into a full centerline path.
        If start_point or end_point are provided, trims the path accordingly.
        '''
        path_centerline = []
        num_lanelet = len(path)

        prev_start = 0
        if start_point is not None:
            cl = to2D(path[0].centerline)
            cl_length = lanelet2.geometry.length(cl)
            prev_start = self.get_dis_from_point(cl, start_point.x, start_point.y) / cl_length
            prev_start = min(0.98, prev_start)

        last_lanelet = path[-1]
        if end_point is not None:
            cl = to2D(last_lanelet.centerline)
            cl_length = lanelet2.geometry.length(cl)
            dis_end = self.get_dis_from_point(cl, end_point.x, end_point.y) / cl_length
        else:
            dis_end = 1

        # Build centerline for each lanelet segment
        for i in range(num_lanelet - 1):
            cur_lanelet = path[i]
            next_lanelet = path[i + 1]
            relation = self.routing_graph.routingRelation(cur_lanelet, next_lanelet, True)

            if relation in [lanelet2.routing.RelationType.Left, lanelet2.routing.RelationType.Right]:
                if i == num_lanelet - 2:
                    cur_end = (dis_end - prev_start) * 0.3 + prev_start
                    next_start = (dis_end - prev_start) * 0.7 + prev_start
                else:
                    cur_end = (0.98 - prev_start) * 0.3 + prev_start
                    next_start = (0.98 - prev_start) * 0.7 + prev_start
                path_centerline.extend(self.get_centerline_section(cur_lanelet, prev_start, cur_end, False, allow_lane_change))
                prev_start = next_start
            else:
                path_centerline.extend(self.get_centerline_section(cur_lanelet, prev_start, 0.99, False, allow_lane_change))
                prev_start = 0

        path_centerline.extend(self.get_centerline_section(last_lanelet, prev_start, dis_end, True))
        return path_centerline

    def get_centerline_section(self, lanelet, norm_dis_start, norm_dis_end, include_end_point=False, allow_lane_change=True):
        '''
        Extract a segment of a lanelet's centerline, normalized from 0.0 to 1.0.
        Also includes lane width and speed limit information per point.
        '''
        centerline_section = []
        centerline = to2D(lanelet.centerline)
        length = lanelet2.geometry.length(centerline)
        speed_limit = self.get_lanelet_speed_limit(lanelet)

        assert norm_dis_start * norm_dis_end >= 0, "Start and end distances must have same sign"
        if abs(norm_dis_start) > abs(norm_dis_end):
            self.node.get_logger().warn(f"Start distance {norm_dis_start} must be less than end distance {norm_dis_end}")
            return centerline_section

        if norm_dis_start < 0:
            centerline = centerline.invert()

        dis_start = max(abs(norm_dis_start) * length, 0)
        dis_end = min(abs(norm_dis_end) * length, length)
        num_points = len(centerline) - int(include_end_point)

        # Add interpolated start point
        arc_coord = ArcCoordinates()
        arc_coord.dis = 0.0
        arc_coord.length = dis_start
        pt_start = fromArcCoordinates(centerline, arc_coord)
        width_left, width_right = self.get_lane_width(pt_start, lanelet, allow_lane_change)
        centerline_section.append([pt_start.x, pt_start.y, width_left, width_right, speed_limit])

        for i in range(num_points):
            pt = centerline[i]
            pt_len = self.get_dis_from_point(centerline, pt.x, pt.y)
            if pt_len <= dis_start:
                continue
            elif pt_len >= dis_end:
                break
            width_left, width_right = self.get_lane_width(pt, lanelet, allow_lane_change)
            centerline_section.append([pt.x, pt.y, width_left, width_right, speed_limit])

        if include_end_point:
            arc_coord.length = dis_end
            pt_end = fromArcCoordinates(centerline, arc_coord)
            width_left, width_right = self.get_lane_width(pt_end, lanelet, allow_lane_change)
            centerline_section.append([pt_end.x, pt_end.y, width_left, width_right, speed_limit])
        return centerline_section

    def get_lane_width(self, point, lanelet, allow_lane_change=True):
        '''
        Computes width from the center point to the left and right bounds of the lanelet.
        If lane changing is allowed, uses bounds of neighboring lanelets if available.
        '''
        left_bound = lanelet.leftBound
        right_bound = lanelet.rightBound

        if allow_lane_change:
            left_lanelet = self.routing_graph.left(lanelet)
            right_lanelet = self.routing_graph.right(lanelet)
            if left_lanelet is not None:
                left_bound = left_lanelet.leftBound
            if right_lanelet is not None:
                right_bound = right_lanelet.rightBound

        left_bound = to2D(left_bound)
        right_bound = to2D(right_bound)
        basic_point = BasicPoint2d(point.x, point.y)

        width_left = lanelet2.geometry.distance(left_bound, basic_point)
        width_right = lanelet2.geometry.distance(right_bound, basic_point)
        return width_left, width_right

    @staticmethod
    def get_dis_from_point(line_string, x, y):
        '''
        Compute the length along a line string up to the projection of a 2D point.
        '''
        pt = BasicPoint2d(x, y)
        return toArcCoordinates(line_string, pt).length

    @staticmethod
    def get_lanelet_length(lanelet):
        '''Return the total 2D centerline length of a lanelet.'''
        return lanelet2.geometry.length2d(lanelet)

    @staticmethod
    def get_linestring_length(line_string):
        '''Return the total length of a 3D linestring.'''
        return lanelet2.geometry.length(line_string)

    @staticmethod
    def get_lanelet_speed_limit(lanelet):
        '''Return the speed limit for the given lanelet, defaulting to 2.0 if not present.'''
        return float(lanelet.attributes.get('speed', 2.0))

    @staticmethod
    def get_point_at_dis(line_string, dist):
        '''Get a point at a given arc distance along a line string (interpolated).'''
        return lanelet2.geometry.interpolatedPointAtDistance(line_string, dist)

    @staticmethod
    def get_point_at_normalized_dis(line_string, dist_norm):
        '''Same as above, but with normalized distance from 0 to 1.'''
        dis = dist_norm * lanelet2.geometry.length(line_string)
        return lanelet2.geometry.interpolatedPointAtDistance(line_string, dis)

    @staticmethod
    def project_xy_to_linestring(x, y, line_string):
        '''Project a 2D point onto a 3D linestring and return the closest projected x, y.'''
        pt = BasicPoint3d(float(x), float(y), 0.0)
        pt_project = lanelet2.geometry.project(line_string, pt)
        return pt_project.x, pt_project.y

    def load_lanelet_map(self, osm_file):
        '''Load a Lanelet2 map from .osm using the local projector.'''
        return lanelet2.io.load(osm_file, self.projector)

    def save_lanelet_map(self, lanelet_map, osm_file):
        '''Save a Lanelet2 map to .osm using the local projector.'''
        lanelet2.io.write(osm_file, lanelet_map, self.projector)

    def warmup(self):
        '''
        This function is used to warm up the Lanelet2 library by forcing all lanelet centerlines to be cached.
        Avoids lazy-loading delays during routing.
        '''
        for lanelet in self.lanelet_layer:
            _ = lanelet.centerline
