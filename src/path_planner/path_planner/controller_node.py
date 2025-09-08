#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from .graph import Graph
from .graph_search import GraphSearch
from .map import Map
from .path_smoother import PathSmoother


class GraphSearchController(Node):
    def __init__(self):
        super().__init__('graph_search_controller_node')
        transient_qos = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   depth=10)
        
        # Parameters: visuals
        self.declare_parameter('visualisation_rate', rclpy.Parameter.Type.INTEGER)
        self.visualisation_sleep_ = 0.5/self.get_parameter('visualisation_rate').value
        
        # Parameters: problem variations
        self.declare_parameter('use_energy_costs', rclpy.Parameter.Type.BOOL)
        self.declare_parameter('show_connectivity', rclpy.Parameter.Type.BOOL)

        # Parameters: map
        self.declare_parameter('filename', rclpy.Parameter.Type.STRING)
        self.declare_parameter('resolution', rclpy.Parameter.Type.DOUBLE)

        # Parameters: graph
        self.declare_parameter('use_prm', rclpy.Parameter.Type.BOOL)
        self.declare_parameter('grid_step_size', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('prm_num_nodes', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('prm_max_edge_length', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('do_distance_transform', rclpy.Parameter.Type.BOOL)
        self.declare_parameter('use_distance_transform_graph', rclpy.Parameter.Type.BOOL)

        # Parameters: A* search
        self.declare_parameter('heuristic_weight', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('use_naive_planner', rclpy.Parameter.Type.BOOL)

        # Parameters: smoothing
        self.declare_parameter('alpha', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('beta', rclpy.Parameter.Type.DOUBLE)

        # Parameters: initial start & goal position
        self.declare_parameter('startx', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('starty', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('goalx', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('goaly', rclpy.Parameter.Type.INTEGER)

        # Create a map from .mat file
        self.map_ = Map(self.get_logger(),
                        self.get_parameter('filename').value,
                        self.get_parameter('resolution').value,
                        self.get_parameter('do_distance_transform').value)
        self.map_viz_pub_ = self.create_publisher(PointCloud2, 'terrain_viz', transient_qos)
        self.distance_transform_viz_pub_ = self.create_publisher(PointCloud2, 'distance_transform_viz', transient_qos)
        
        # Create a graph from the map
        self.graph_ = Graph(self.get_logger(), self.map_,
                            self.get_parameter('grid_step_size').value,
                            self.get_parameter('prm_num_nodes').value,
                            self.get_parameter('use_energy_costs').value,
                            self.get_parameter('use_prm').value,
                            self.get_parameter('show_connectivity').value,
                            self.get_parameter('prm_max_edge_length').value,
                            self.get_parameter('use_distance_transform_graph').value)
        self.graph_marker_pub_ = self.create_publisher(Marker, 'markers/grid', transient_qos)

        # The map and grid and distance transform never change, so we can just publish once and persist this message
        self.visualise_map()
        self.visualise_graph()
        if self.get_parameter('do_distance_transform').value:
            self.visualise_distance_transform()

        if not self.get_parameter('show_connectivity').value:

            # Initialise graph search object
            self.graph_search_ = GraphSearch(self.get_logger(), self.graph_,
                                             self.get_parameter('use_naive_planner').value,
                                             self.get_parameter('heuristic_weight').value,
                                             self.visualise_search)
            self.search_marker_pub_ = self.create_publisher(MarkerArray, 'markers/search', 3)
            self.path_pub_ = self.create_publisher(Path, 'path_planner/plan', transient_qos)
            self.start_end_marker_pub_ = self.create_publisher(Marker, 'markers/start_end', 3)

            # Initialise path smoother object
            self.path_smoother_ = PathSmoother(self, self.graph_,
                                               self.get_parameter('alpha').value,
                                               self.get_parameter('beta').value)
            self.path_smooth_pub_ = self.create_publisher(Path, 'path_planner/plan_smooth', transient_qos)

            # Plan initial path from start to goal
            self.start_xy_ = [self.get_parameter('startx').value, self.get_parameter('starty').value]
            self.plan_path([self.get_parameter('goalx').value, self.get_parameter('goaly').value])

            # Wait for a new goal from RViz
            self.rviz_goal_sub_ = self.create_subscription(PointStamped, 'clicked_point', self.rviz_goal_callback, 1)

    def plan_path(self, goal_xy):
        """Plan a path from start tself.get_logger().info(f'Planning from {xy[i]} to {best_dist}...')o goal and visualise it"""

        self.get_logger().info(f'Planning from {self.start_xy_} to {goal_xy}...')
        start_idx = self.graph_.get_closest_node(self.start_xy_)
        goal_idx = self.graph_.get_closest_node(goal_xy)

        # Visualise start and end
        marker = self.graph_.generate_start_end_markers(start_idx, goal_idx)
        self.start_end_marker_pub_.publish(marker)

        # Do graph search
        self.graph_search_.plan_path(start_idx, goal_idx)
        
        # Smooth resulting path
        self.path_smoother_.smooth_path(self.graph_search_.path_)

        # Visualise resulting paths
        self.visualise_path()

        # Plan from the end of this path next time
        self.start_xy_ = goal_xy
    
    def rviz_goal_callback(self, msg):
        """Go to the goal provided by the clicked_point in RViz"""

        self.get_logger().warn('New goal from RViz')
        goal_xy = self.map_.world_to_pixel(msg.point.x, msg.point.y)

        # Clear visualised path and search markers
        empty_path = Path()
        empty_path.header.frame_id = 'map'
        self.path_pub_.publish(empty_path)
        self.path_smooth_pub_.publish(empty_path)

        # Start planning
        self.plan_path(goal_xy)
    
    def visualise_map(self):
        """Publishes the map to be visualised in RViz"""

        self.map_viz_pub_.publish(self.map_.generate_pcl2_msg(self.get_clock().now().to_msg()))
        self.get_logger().warn('Published terrain cloud')

    def visualise_distance_transform(self):
        """Publishes the distance transform map to be visualised in RViz"""

        self.distance_transform_viz_pub_.publish(self.map_.generate_pcl2_msg_distance_transform(self.get_clock().now().to_msg()))
        self.get_logger().warn('Published distance transform cloud')
    
    def visualise_graph(self):
        """Publishes the graph to be visualised in RViz"""

        marker_nodes, marker_edges = self.graph_.generate_marker_msgs()

        self.graph_marker_pub_.publish(marker_nodes)
        self.graph_marker_pub_.publish(marker_edges)
        self.get_logger().warn('Published graph visualisation')

    def visualise_path(self):
        """Publishes the path to be visualised in RViz"""

        self.path_pub_.publish(self.map_.generate_path_msg(self.graph_search_.path_))
        self.path_smooth_pub_.publish(self.map_.generate_path_msg(self.path_smoother_.path_smooth_,
                                                                  z_offset=0.025))
        self.get_logger().warn('Published paths')

    def visualise_search(self, visited_set, unvisited_set):
        """Publishes the progress of the search to be visualised in RViz"""

        marker_visited, marker_unvisited = self.graph_.generate_search_markers(
            visited_set, unvisited_set)

        marker_array = MarkerArray()
        marker_array.markers = [marker_visited, marker_unvisited]

        self.search_marker_pub_.publish(marker_array)
        time.sleep(self.visualisation_sleep_)
        # self.get_logger().warn('Published search markers')

def main():
    # Initialise
    rclpy.init()

    # Do the graph search
    graph_search_controller = GraphSearchController()

    while rclpy.ok():
        rclpy.spin(graph_search_controller)

if __name__ == '__main__':
    main()