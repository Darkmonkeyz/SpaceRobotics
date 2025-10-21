#!/usr/bin/env python3

import math
import random
from enum import Enum

import cv2  # OpenCV2
import torch
from ultralytics import YOLO
import rclpy
from cv_bridge import CvBridge
import numpy as np
from scipy.ndimage import binary_dilation #for frontiers
from sklearn.cluster import DBSCAN #for clustering frontiers
from geometry_msgs.msg import Pose, Pose2D, PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R 
from action_msgs.msg import GoalStatus
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

import heapq


from collections import deque


from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA


def wrap_angle(angle):
    """Function to wrap an angle between 0 and 2*Pi"""
    while angle < 0.0:
        angle = angle + 2 * math.pi

    while angle > 2 * math.pi:
        angle = angle - 2 * math.pi

    return angle

def pose2d_to_pose(pose_2d):
    """Convert a Pose2D to a full 3D Pose"""
    pose = Pose()

    pose.position.x = pose_2d.x
    pose.position.y = pose_2d.y

    pose.orientation.w = math.cos(pose_2d.theta / 2.0)
    pose.orientation.z = math.sin(pose_2d.theta / 2.0)

    return pose


class PlannerType(Enum):
    ERROR = 0
    MOVE_FORWARDS = 1
    RETURN_HOME = 2
    GO_TO_FIRST_ARTIFACT = 3
    RANDOM_WALK = 4
    RANDOM_GOAL = 5
    # Add more!
    SELECT_AND_GO_TO_FRONTIER = 6
    CLOSERANGE_INSPECTION = 7
    NOSCOPE360 = 8
    HEADSNAP = 9
    TRAVELSALESMAN = 10
    

class ArtifactType(Enum):
    MUSHROOM = 0
    ICECASTLE = 1
    URANIUM = 2
    HOWARD = 3
    SNOWBALL = 4

ARTIFACT_HEIGHTS = { #height in meters from blender and mars_cave.sdf
    ArtifactType.MUSHROOM: 2.03,
    ArtifactType.ICECASTLE: 1.51,
    ArtifactType.URANIUM: 2.67, #changing height from 2.84 since its buried
    ArtifactType.HOWARD: 3.00, #chaning height from 2.86 to slightly taller since the alien floats
    ArtifactType.SNOWBALL: 1.6,
}

    
    
class MockFuture:  #useful only for my quick goal cancel logic
        def __init__(self, success=True):
            self._success = success
        def result(self):
            class Result:
                result = type('r', (), {})()
                result.result = NavigateToPose.Result()
            return Result()

class CaveExplorer(Node):
    def __init__(self):
        super().__init__('cave_explorer_node')

        # Variables/Flags for mapping
        self.xlim_ = [0.0, 0.0]
        self.ylim_ = [0.0, 0.0]

        self.obstacle_map_ = None

        self.updated_cells_ = set()

        self.distance_transform_map_ = None

        self.map_resolution_ = None
        self.map_origin_ = []

        self.bridge = CvBridge()
        

        # Variables/Flags for perception
        self.artifact_found_ = False

        self.frontierClusters_ =[]

        # Variables/Flags for planning
        self.planner_type_ = PlannerType.ERROR
        self.reached_first_artifact_ = False
        self.returned_home_ = False
        #Variable for behavior
        self.closeRangeInspection_ = False
        self.artifactInView_ = False
        self.viewTicker = 0
        #

        # Marker for artifact locations
        # See https://wiki.ros.org/rviz/DisplayTypes/Marker
        self.marker_artifacts_ = Marker()
        self.marker_artifacts_.header.frame_id = "map"
        self.marker_artifacts_.ns = "artifacts"
        self.marker_artifacts_.id = 0
        self.marker_artifacts_.type = Marker.SPHERE_LIST
        self.marker_artifacts_.action = Marker.ADD
        self.marker_artifacts_.pose.position.x = 0.0
        self.marker_artifacts_.pose.position.y = 0.0
        self.marker_artifacts_.pose.position.z = 0.0
        self.marker_artifacts_.pose.orientation.x = 0.0
        self.marker_artifacts_.pose.orientation.y = 0.0
        self.marker_artifacts_.pose.orientation.z = 0.0
        self.marker_artifacts_.pose.orientation.w = 1.0
        self.marker_artifacts_.scale.x = 1.5
        self.marker_artifacts_.scale.y = 1.5
        self.marker_artifacts_.scale.z = 1.5
        self.marker_artifacts_.color.a = 1.0
        self.marker_artifacts_.color.r = 0.0
        self.marker_artifacts_.color.g = 1.0
        self.marker_artifacts_.color.b = 0.2
        self.marker_pub_ = self.create_publisher(MarkerArray, 'marker_array_artifacts', 10)

        #very original code
        self.rough_marker_artifacts_ = Marker()
        self.rough_marker_artifacts_.header.frame_id = "map"
        self.rough_marker_artifacts_.ns = "artifacts"
        self.rough_marker_artifacts_.id = 0
        self.rough_marker_artifacts_.type = Marker.SPHERE_LIST
        self.rough_marker_artifacts_.action = Marker.ADD
        self.rough_marker_artifacts_.pose.position.x = 0.0
        self.rough_marker_artifacts_.pose.position.y = 0.0
        self.rough_marker_artifacts_.pose.position.z = 0.0
        self.rough_marker_artifacts_.pose.orientation.x = 0.0
        self.rough_marker_artifacts_.pose.orientation.y = 0.0
        self.rough_marker_artifacts_.pose.orientation.z = 0.0
        self.rough_marker_artifacts_.pose.orientation.w = 1.0
        self.rough_marker_artifacts_.scale.x = 1.5
        self.rough_marker_artifacts_.scale.y = 1.5
        self.rough_marker_artifacts_.scale.z = 1.5
        self.rough_marker_artifacts_.color.a = 0.5
        self.rough_marker_artifacts_.color.r = 1.0
        self.rough_marker_artifacts_.color.g = 0.5
        self.rough_marker_artifacts_.color.b = 0.2
        self.rough_marker_pub_ = self.create_publisher(MarkerArray, 'rough_marker_array_artifacts', 10)

        # Remember the artifact locations

        #travlin sales guy
        self.travelsalesmanplan_ = None
        self.travelling_sales_points_ = []

        self.widest = None
        self.narrowest = None
        # Array of type geometry_msgs.Point
        self.artifacts_ = []

        self.rough_artifacts_ = []

        self.artifactSightingOfInterest_ = None
        # Initialise CvBridge
        self.cv_bridge_ = CvBridge()

        # Prepare transformation to get robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action client for nav2
        self.nav2_action_client_ = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().warn('Waiting for navigate_to_pose action...')
        self.nav2_action_client_.wait_for_server()
        self.get_logger().warn('navigate_to_pose connected')
        self.ready_for_next_goal_ = True
        self.declare_parameter('print_feedback', rclpy.Parameter.Type.BOOL)

        # Publisher for the goal pose visualisation
        self.goal_pose_vis_ = self.create_publisher(PoseStamped, 'goal_pose', 1)

        # Subscribe to the map topic to get current bounds
        self.map_sub_ = self.create_subscription(OccupancyGrid, 'map',  self.map_callback, 1)

        self.frontier_pub = self.create_publisher(MarkerArray, "frontier_markers", 10) #frontierPublisher

        self.dt_pub = self.create_publisher(OccupancyGrid, 'distance_transform', 10)

        self.dt_image_pub = self.create_publisher(Image, 'distance_transform_image', 1)

        self.debug_marker_pub_ = self.create_publisher(MarkerArray, 'frontier_candidate_points', 1)


        self.depth_subscriber = self.create_subscription(Image, '/camera/depth/image', self.depth_callback, 10)

        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.latest_depth_image = None
        self.latest_point_cloud = None

        # Prepare image processing
        self.image_detections_pub_ = self.create_publisher(Image, 'detections_image', 1)
        self.declare_parameter('computer_vision_model_filename', rclpy.Parameter.Type.STRING)
        self.computer_vision_model_ = cv2.CascadeClassifier(self.get_parameter('computer_vision_model_filename').value)
        self.declare_parameter('computercooler_vision_model_filename', rclpy.Parameter.Type.STRING)
        self.computer_vision_model_ = YOLO(self.get_parameter('computercooler_vision_model_filename').value)

        self.image_sub_ = self.create_subscription(Image, 'camera/image', self.image_callback, 1)

        # Timer for main loop
        self.main_loop_timer_ = self.create_timer(0.2, self.main_loop)


    def depth_callback(self, msg):
        self.latest_depth_image = msg

    def lidar_callback(self, msg):
        self.latest_point_cloud = msg
    
    def get_pose_2d(self):
        """Get the 2d pose of the robot"""

        # Lookup the latest transform
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().error(f'Could not transform: {ex}')
            return

        # Return a Pose2D message
        pose = Pose2D()
        pose.x = t.transform.translation.x
        pose.y = t.transform.translation.y

        qw = t.transform.rotation.w
        qz = t.transform.rotation.z

        if qz >= 0.:
            pose.theta = wrap_angle(2. * math.acos(qw))
        else: 
            pose.theta = wrap_angle(-2. * math.acos(qw))

        #self.get_logger().warn(f'Pose: {pose}')

        return pose
    
    def get_pose_camera_2d(self):  #not useful maybe?
        """Get the 2d pose of the robot camera"""

        # Lookup the latest transform
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'camera_rgb_optical_frame',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().error(f'Could not transform: {ex}')
            return

        # Return a Pose2D message
        pose = Pose2D()
        pose.x = t.transform.translation.x
        pose.y = t.transform.translation.y

        qw = t.transform.rotation.w
        qz = t.transform.rotation.z

        if qz >= 0.:
            pose.theta = wrap_angle(2. * math.acos(qw))
        else: 
            pose.theta = wrap_angle(-2. * math.acos(qw))

        self.get_logger().warn(f'Camera Pose: {pose}')

        return pose
    
    

    def map_callback(self, map_msg: OccupancyGrid):
        """New map received, so update x and y limits"""

        # Extract data from message
        self.map_origin_ = [
            map_msg.info.origin.position.x,
            map_msg.info.origin.position.y,
        ]
        self.map_resolution_ = map_msg.info.resolution
        map_origin = [map_msg.info.origin.position.x, 
                      map_msg.info.origin.position.y]
        map_resolution = map_msg.info.resolution
        map_height = map_msg.info.height
        map_width = map_msg.info.width
        res = map_msg.info.resolution
        
        
        # Set current limits
        self.xlim_ = [map_origin[0], map_origin[0]+map_width*map_resolution]
        self.ylim_ = [map_origin[1], map_origin[1]+map_height*map_resolution]


        """begininnign of hell // fool errand just use open cv next time moron :(

        new_map = np.array(map_msg.data, dtype=np.int8).reshape((map_height, map_width))

        if self.obstacle_map_ is None or self.obstacle_map_.size == 0:
            # First callback, just store the map
            self.obstacle_map_ = new_map.copy()
            self.distance_transform_map_ = np.full_like(new_map, np.nan, dtype=float)
            oldmap = self.obstacle_map_.copy()
            self.distance_transform_map_[self.obstacle_map_ == 1] = 0
            self.last_origin_x = map_msg.info.origin.position.x
            self.last_origin_y = map_msg.info.origin.position.y
            self.uber_nuke_distance_update()
            return
        else:
            oldmap = self.obstacle_map_.copy()
            old_h, old_w = oldmap.shape

            dx = map_msg.info.origin.position.x - self.last_origin_x
            dy = map_msg.info.origin.position.y - self.last_origin_y

            pad_left   = int(max(0, -dx / res))  
            pad_right  = int(max(0, map_width - old_w - pad_left))
            pad_top    = int(max(0, -dy / res))  
            pad_bottom = int(max(0, map_height - old_h - pad_top))

            if pad_top > 0 or pad_bottom > 0 or pad_left > 0 or pad_right > 0:
                oldmap = np.pad(
                    oldmap,
                    ((pad_top, pad_bottom), (pad_left, pad_right)),
                    mode='constant',
                    constant_values=-1
                )
                if self.distance_transform_map_ is None:
                    self.distance_transform_map_ = np.full_like(oldmap, np.nan, dtype=float)
                else:
                    self.distance_transform_map_ = np.pad(
                        self.distance_transform_map_,
                        ((pad_top, pad_bottom), (pad_left, pad_right)),
                        mode='constant',
                        constant_values=np.nan
                    )

            new_h, new_w = new_map.shape
            if oldmap.shape[0] > new_h:
                oldmap = oldmap[:new_h, :]
                self.distance_transform_map_ = self.distance_transform_map_[:new_h, :]
            if oldmap.shape[1] > new_w:
                oldmap = oldmap[:, :new_w]
                self.distance_transform_map_ = self.distance_transform_map_[:, :new_w]
                
        

        changed_indices = np.argwhere(oldmap != self.obstacle_map_) # checking for changes between map messages
        self.updated_cells_ = set(map(tuple, changed_indices))"""
        
        self.obstacle_map_ = np.array(map_msg.data).reshape((map_height,  map_width)) #added to actually get map info


        
        self.uber_nuke_distance_update()
        #self.incremental_distance_update()
        self.display_distance_map(map_resolution, map_width, map_height, map_origin)

        

        
        # self.get_logger().warn('Map received:')
        # self.get_logger().warn(f'  xlim = [{self.xlim_[0]:.2f}, {self.xlim_[1]:.2f}]')
        # self.get_logger().warn(f'  ylim = [{self.ylim_[0]:.2f}, {self.ylim_[1]:.2f}]')

        frontiers = self.detect_frontiers(self.obstacle_map_, res, map_origin)
        #self.get_logger().warn(f"Detected {len(frontiers)} frontier points") #debugging step
        frontierClusters = self.cluster_frontiers(frontiers)
        self.publish_frontier_markers(frontierClusters)
        self.frontierClusters_ = frontierClusters 

    

    

    #distance transform 
    def uber_nuke_distance_update(self): #way better to monkey brain it since at least it works
        binary_map = np.zeros_like(self.obstacle_map_, dtype=np.uint8)
        binary_map[self.obstacle_map_ == 100] = 255 # obstacle
        binary_map[self.obstacle_map_ == -1] = 255  # unknown
        dt_cv = cv2.distanceTransform(255 - binary_map, cv2.DIST_L2, 3) #who whould have guessed premade distance transform way better than me :)
        self.distance_transform_map_ = dt_cv

    ''' #some garbage incremental distance transform thing :(
    def incremental_distance_update(self):
        """Update self.distance_transform_map_ using only the updated cells."""
        if not self.updated_cells_:
            return

        rows, cols = self.obstacle_map_.shape
        dt = self.distance_transform_map_
        queue = deque()

        for (i, j) in self.updated_cells_:
            if self.obstacle_map_[i, j] == 1:
                dt[i, j] = 0
                queue.append((i, j))
            else:
                if np.isnan(dt[i, j]):
                    dt[i, j] = np.inf
                

        
        directions = [(-1,0,1),(1,0,1),(0,-1,1),(0,1,1), (-1,-1,1.414),(-1,1,1.414),(1,-1,1.414),(1,1,1.414)]
        while queue:
            i, j = queue.popleft()
            for di, dj, dv in directions:
                ni, nj = i + di, j + dj
                if 0 <= ni < rows and 0 <= nj < cols:
                    new_dist = dt[i, j] + dv
                    if np.isnan(dt[ni, nj]) or new_dist < dt[ni, nj]:
                        dt[ni, nj] = new_dist
                        queue.append((ni, nj))

        self.updated_cells_.clear()
    '''
        
    def get_obstacle_value_at(self, x_m: float, y_m: float):
        """
        Get the value of the obstacle map at world coordinates (x_m, y_m) in meters.
        Returns:
            int: obstacle value (e.g., 0 = free, 100 = obstacle, -1 = unknown)
            or None if out of bounds
        """
        if self.obstacle_map_ is None:
            self.get_logger().warn("Obstacle map not initialized yet.")
            return None

        res = self.map_resolution_
        origin_x, origin_y = self.map_origin_

        # Convert world coordinates to grid indices
        j = int((x_m - origin_x) / res)  # column
        i = int((y_m - origin_y) / res)  # row

        if not (0 <= i < self.obstacle_map_.shape[0] and 0 <= j < self.obstacle_map_.shape[1]):
            return None  # out of bounds

        return int(self.obstacle_map_[i, j])


    def get_distance_value_at(self, x_m: float, y_m: float):
        """
        Get the distance transform value (in meters) at world coordinates (x_m, y_m).
        Returns:
            float: distance to nearest obstacle in meters
            or None if out of bounds or undefined
        """
        if self.distance_transform_map_ is None:
            self.get_logger().warn("Distance transform not initialized yet.")
            return None

        res = self.map_resolution_
        origin_x, origin_y = self.map_origin_

        j = int((x_m - origin_x) / res)
        i = int((y_m - origin_y) / res)

        if not (0 <= i < self.distance_transform_map_.shape[0] and
                0 <= j < self.distance_transform_map_.shape[1]):
            return None

        dt_value = float(self.distance_transform_map_[i, j])
        if np.isnan(dt_value):
            return None

        # Each pixel represents 'res' meters; distanceTransform outputs pixel units
        return dt_value * res


    
    #show distance transform
    def display_distance_map(self, res, width, height, origin):
        max_dist = np.max(self.distance_transform_map_)
        min_dist = np.min(self.distance_transform_map_)
   
        dt_normalized = (self.distance_transform_map_ - min_dist) / (max_dist - min_dist) * 100.0
        dt_normalized[np.isnan(self.distance_transform_map_)] = -1
        dt_normalized = dt_normalized.astype(np.int8)

        grid_msg = OccupancyGrid()

        grid_msg.header.frame_id = "map" 

        grid_msg.info.resolution = res
        grid_msg.info.width = width
        grid_msg.info.height = height
        grid_msg.info.origin.position.x = origin[0]
        grid_msg.info.origin.position.y = origin[1]
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = dt_normalized.flatten().tolist()
        
        self.dt_pub.publish(grid_msg)
        # dt_map = self.distance_transform_map_.copy()
        # dt_map[np.isnan(dt_map)] = 0
        # dt_image = (dt_map / np.nanmax(dt_map) * 255).astype(np.uint8)
        # dt_image = (self.distance_transform_map_ / np.nanmax(self.distance_transform_map_) * 255).astype(np.uint8)
        # img_msg = self.bridge.cv2_to_imgmsg(dt_image, encoding="mono8")
        # self.dt_image_pub.publish(img_msg)
        dt = self.distance_transform_map_.copy()
        dt[dt <= 0] = np.nan 

        try:
            max_idx = np.unravel_index(np.nanargmax(dt), dt.shape)
            min_idx = np.unravel_index(np.nanargmin(dt), dt.shape)
        except ValueError:
            return

        def index_to_world(ix, iy):
            wx = origin[0] + ix * res
            wy = origin[1] + iy * res
            return wx, wy

        self.widest = index_to_world(max_idx[1], max_idx[0])
        self.narrowest = index_to_world(min_idx[1], min_idx[0])

        
        lap = (
            np.roll(dt, 1, axis=0)
            + np.roll(dt, -1, axis=0)
            + np.roll(dt, 1, axis=1)
            + np.roll(dt, -1, axis=1)
            - 4 * dt
        )
        saddle_mask = np.logical_and(lap > -0.05, lap < 0.05)
        saddle_indices = np.argwhere(saddle_mask)
        saddle_points = [index_to_world(x, y) for y, x in saddle_indices[::100]] 

        
        if not hasattr(self, "dt_marker_pub"):
            self.dt_marker_pub = self.create_publisher(MarkerArray, "distance_map_extrema", 10)

        marker_array = MarkerArray()
        marker_id = 0

        def make_marker(x, y, color, ns):
            nonlocal marker_id
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = marker_id
            marker_id += 1
            m.ns = ns
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.1
            m.scale.x = 0.25
            m.scale.y = 0.25
            m.scale.z = 0.25
            m.color.r, m.color.g, m.color.b, m.color.a = color
            return m

        
        marker_array.markers.append(make_marker(*self.narrowest, (1.0, 0.0, 0.0, 1.0), "narrowest"))
        marker_array.markers.append(make_marker(*self.widest, (0.0, 1.0, 0.0, 1.0), "widest"))

        for (x, y) in saddle_points:
            marker_array.markers.append(make_marker(x, y, (0.0, 0.0, 1.0, 0.3), "saddle"))

        self.dt_marker_pub.publish(marker_array)


    def detect_frontiers(self, data: np.ndarray, res: float, origin: list):
        free_mask = data == 0
        unknown_mask = data == -1

        unknown_dilated = binary_dilation(unknown_mask, structure=np.ones((3, 3)))
        frontier_mask = free_mask & unknown_dilated

        ys, xs = np.where(frontier_mask)
        
        return np.array([ [origin[0] + x * res, origin[1] + y * res] for x, y in zip(xs, ys) ])
    
    

    def cluster_frontiers(self, frontier_points):
        """# if len(frontier_points) == 0:
        #     return []

        # clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(frontier_points)
        # labels = clustering.labels_

        # clusters = []
        # for label in set(labels):
        #     if label == -1:
        #         continue
        #     cluster_pts = frontier_points[labels == label]
        #     centroid = np.mean(cluster_pts, axis=0)
        #     clusters.append({
        #         "id": label,
        #         "points": cluster_pts,
        #         "centroid": centroid
        #     })
        # #self.get_logger().warn(f"Detected {len(clusters)} clusters")
        # return clusters"""
        neighbour_radius = 1
        cutOff = 8
        if len(frontier_points) == 0:
            return []

        unvisited = set(range(len(frontier_points)))
        clusters = []

        while unvisited:
            idx = unvisited.pop()
            cluster_indices = {idx}
            stack = [idx]

            while stack:
                current_idx = stack.pop()
                current_point = frontier_points[current_idx]

                neighbors = []
                for i in unvisited:
                    if np.linalg.norm(current_point - frontier_points[i]) <= neighbour_radius:
                        neighbors.append(i)

                for n in neighbors:
                    stack.append(n)
                    cluster_indices.add(n)
                    unvisited.remove(n)

            cluster_pts = frontier_points[list(cluster_indices)]
            if len(cluster_pts) > cutOff:
                clusters.append({
                    "points": cluster_pts,
                    "centroid": np.mean(cluster_pts, axis=0),
                    "id": len(clusters)
                })

        return clusters
    
    def select_frontier_goal(self, clusters, robot_pose):
        if not clusters:
            return None
        dists = [np.linalg.norm(cluster["centroid"] - np.array([robot_pose.x, robot_pose.y])) for cluster in clusters]
        self.get_logger().warn(f"Selected cluster {clusters[np.argmin(dists)]['id']}")
        return clusters[np.argmin(dists)]
    
    def planner_choose_and_go_to_frontier(self):
        roboPose = self.get_pose_2d()
        clusters = self.frontierClusters_

        selectedCluster = self.select_frontier_goal(clusters, roboPose)
        if selectedCluster != None:
            goal_pose2d = Pose2D(
            x = selectedCluster["centroid"][0],
            y = selectedCluster["centroid"][1],
            theta = math.pi/2
        )
        self.planner_go_to_pose2d(goal_pose2d)

    def planner_choose_and_go_to_frontier_but_good(self, search_radius=2.0, num_samples=100, res = 0.3): #too lazy to actually grab map.info thing
        roboPose = self.get_pose_2d()
        clusters = self.frontierClusters_

        selectedCluster = self.select_frontier_goal(clusters, roboPose)
        if selectedCluster is None:
            return
        candidates = []
        cx, cy = selectedCluster["centroid"]

        marker_array = MarkerArray() #for visualisation of point cloud since its not behaving
        marker_id = 0

        for _ in range(num_samples): #gaussian splatter like from A1 yipee!!
            dx = np.random.normal(0, search_radius)
            dy = np.random.normal(0, search_radius)
            x = cx + dx
            y = cy + dy
            

            reason = None
            passed = True

            obs_val = self.get_obstacle_value_at(x, y)
            if obs_val is None:
                passed = False
                reason = 'out_of_bounds'
            elif obs_val >= 100 or obs_val == -1:
                passed = False
                reason = 'invalid_terrain'

            elif not self._bresenham_clear_world((x, y), (cx, cy)):
                passed = False
                reason = 'blocked_los'

            if passed:
                dt_val = self.get_distance_value_at(x, y)
                if dt_val is None:
                    passed = False
                    reason = 'no_dt'
                else:
                    candidates.append((x, y, dt_val))

            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = marker_id
            marker_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = 0.1
            m.scale.y = 0.1
            m.scale.z = 0.1
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.0

            if passed:
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
                m.color.a = 1.0
            else:
                m.color.r = 1.0
                m.color.g = 0.0
                m.color.b = 0.0
                m.color.a = 1.0

            marker_array.markers.append(m)
            if not passed:
                t = Marker()
                t.header.frame_id = "map"
                t.header.stamp = self.get_clock().now().to_msg()
                t.id = marker_id
                marker_id += 1
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.scale.z = 0.25 
                t.pose.position.x = x
                t.pose.position.y = y
                t.pose.position.z = 0.15  
                t.color.r = 1.0
                t.color.g = 1.0
                t.color.b = 1.0
                t.color.a = 1.0
                t.text = reason
                marker_array.markers.append(t)

        
        self.debug_marker_pub_.publish(marker_array)

        
        if not candidates:
            goal_x, goal_y = cx, cy
            self.get_logger().info('Fallback to centroid')
        else:
            goal_x, goal_y, _ = max(candidates, key=lambda c: c[2])
            self.get_logger().info('ChoosingActualheatmap good')

        goal_pose2d = Pose2D(
            x=goal_x,
            y=goal_y,
            theta=math.pi/2
        )

        self.planner_go_to_pose2d(goal_pose2d)
    
    def _bresenham_clear_world(self, start_m, end_m):
        """
        Bresenham's line algorithm in world coordinates (meters).
        start_m, end_m: (x, y) in meters
        Returns True if all cells along the line are free (value < 100)
        """
        res = self.map_resolution_
        origin_x = self.map_origin_[0]
        origin_y = self.map_origin_[1]

        # Convert meters to cell indices
        y0 = int((start_m[1] - origin_y) / res)
        x0 = int((start_m[0] - origin_x) / res)
        y1 = int((end_m[1] - origin_y) / res)
        x1 = int((end_m[0] - origin_x) / res)

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        err = dx - dy

        while True:
            # Bounds check
            if not (0 <= y0 < self.obstacle_map_.shape[0] and 0 <= x0 < self.obstacle_map_.shape[1]):
                return False

            # Obstacle check
            if self.obstacle_map_[y0, x0] >= 100:
                return False

            # Goal reached
            if x0 == x1 and y0 == y1:
                break

            # Bresenham step
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return True
    



    
    def _bresenham_clear_world_lenient(self, start_m, end_m):
        """
        Bresenham's line algorithm in world coordinates (meters).
        start_m, end_m: (x, y) in meters
        Returns True if all cells along the line **excluding** the last quarter are free.
        """
        res = self.map_resolution_
        origin_x = self.map_origin_[0]
        origin_y = self.map_origin_[1]

        # Convert meters to cell indices
        y0 = int((start_m[1] - origin_y) / res)
        x0 = int((start_m[0] - origin_x) / res)
        y1 = int((end_m[1] - origin_y) / res)
        x1 = int((end_m[0] - origin_x) / res)

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        err = dx - dy

        # Calculate total steps
        total_steps = max(dx, dy)
        # Determine steps for last quarter
        skip_steps = int(total_steps * 0.75)

        steps_taken = 0

        while True:
            # Bounds check
            if not (0 <= y0 < self.obstacle_map_.shape[0] and 0 <= x0 < self.obstacle_map_.shape[1]):
                return False

            # Obstacle check
            if self.obstacle_map_[y0, x0] >= 100:
                return False

            # Stop before last quarter
            if steps_taken >= total_steps - skip_steps:
                break

            # Bresenham step
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

            steps_taken += 1

        return True
    
    def save_obstacle_map_debug(self, filename="obstacle_map_debug.png"): #im stupid so i got gpt to save me an image :)
        """
        Saves the obstacle map as an image for debugging.
        Shows how axes align with your (x, y) and (i, j) logic.
        """
        if self.obstacle_map_ is None:
            self.get_logger().warn("No obstacle map available to save.")
            return

        # Normalize to 0-255 for display
        normalized = np.zeros_like(self.obstacle_map_, dtype=np.uint8)
        normalized[self.obstacle_map_ == -1] = 128  # Unknown = gray
        normalized[self.obstacle_map_ == 0] = 255   # Free = white
        normalized[self.obstacle_map_ >= 100] = 0   # Obstacle = black

        # Draw coordinate axes
        img = cv2.cvtColor(normalized, cv2.COLOR_GRAY2BGR)

        h, w = img.shape[:2]
        cv2.arrowedLine(img, (0, h//2), (w-1, h//2), (0, 0, 255), 1, tipLength=0.02)  # X-axis (cols)
        cv2.arrowedLine(img, (w//2, h-1), (w//2, 0), (0, 255, 0), 1, tipLength=0.02)  # Y-axis (rows)

        # Label axes
        cv2.putText(img, "X (columns, j)", (10, h//2 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(img, "Y (rows, i)", (w//2 + 10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Save flipped and unflipped versions so you can compare
        cv2.imwrite(filename, img)
        cv2.imwrite(filename.replace(".png", "_flipped.png"), cv2.flip(img, 0))
        self.get_logger().info(f"Saved obstacle map debug image to {filename} and flipped version.")

    def publish_frontier_markers(self, clusters):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for cluster in clusters:
            cluster_id = cluster["id"]
            color = ColorRGBA(
                r=random.random(),
                g=random.random(),
                b=random.random(),
                a=0.8
            )

            
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = now
            m.ns = "frontiers"
            m.id = int(cluster_id)
            m.type = Marker.POINTS
            m.action = Marker.ADD
            m.scale.x = 0.5  
            m.scale.y = 0.5
            m.color = color

            m.points = [
                Point(x=p[0], y=p[1], z=0.0)
                for p in cluster["points"]
            ]

            marker_array.markers.append(m)

            
            c = Marker()
            c.header.frame_id = "map"
            c.header.stamp = now
            c.ns = "centroids"
            c.id = 1000 + int(cluster_id)
            c.type = Marker.SPHERE
            c.action = Marker.ADD
            c.pose.position.x = cluster["centroid"][0]
            c.pose.position.y = cluster["centroid"][1]
            c.pose.position.z = 0.0
            c.scale.x = 1.0
            c.scale.y = 1.0
            c.scale.z = 1.0
            c.color = color
            marker_array.markers.append(c)

            
            t = Marker()
            t.header.frame_id = "map"
            t.header.stamp = now
            t.ns = "labels"
            t.id = 2000 + int(cluster_id)
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = cluster["centroid"][0]
            t.pose.position.y = cluster["centroid"][1]
            t.pose.position.z = 1.0
            t.scale.z = 1.0
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.text = f"Frontier {cluster_id}"
            marker_array.markers.append(t)

        self.frontier_pub.publish(marker_array)

    
    def image_callback(self, image_msg):
        """
        Recieve an RGB image.
        Use this method to detect artifacts of interest.
        
        A simple method has been provided to begin with for detecting stop signs (which is not what we're actually looking for) 
        adapted from: https://www.geeksforgeeks.org/detect-an-object-with-opencv-python/
        """
        """ old code method
        # # Copy the image message to a cv image
        # # see http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
        # image = self.cv_bridge_.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')

        # # Create a grayscale version (some simple models use this)
        # # image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # # Retrieve the pre-trained model
        # stop_sign_model = self.computer_vision_model_

        # # Detect artifacts in the image
        # # The minSize is used to avoid very small detections that are probably noise
        # detections = stop_sign_model.detectMultiScale(image, minSize=(20,20))

        # # You can set "artifact_found_" to true to signal to "main_loop" that you have found a artifact
        # # You may want to communicate more information
        # # Since the "image_callback" and "main_loop" methods can run at the same time you should protect any shared variables
        # # with a mutex
        # # "artifact_found_" doesn't need a mutex because it's an atomic
        # num_detections = len(detections)

        # if num_detections > 0:
        #     self.artifact_found_ = True
        # else:
        #     self.artifact_found_ = False

        # # Draw a bounding box rectangle on the image for each detection
        # for(x, y, width, height) in detections:
        #     cv2.rectangle(image, (x, y), (x + height, y + width), (0, 255, 0), 5)

        # # Publish the image with the detection bounding boxes
        # image_detection_message = self.cv_bridge_.cv2_to_imgmsg(image, encoding="rgb8")
        # self.image_detections_pub_.publish(image_detection_message)

        # if self.artifact_found_:
        #     self.get_logger().info('Artifact found!')
        #     self.localise_artifact()
        """

        image = self.cv_bridge_.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        results = self.computer_vision_model_.predict(source=image, imgsz=640, conf=0.5, device='cpu', verbose=False, save=False)
        detections = []
        for box in results[0].boxes:  # x1, y1, x2, y2
            bbox = box.xyxy[0]
            confidence = float(box.conf)
            class_id = int(box.cls)
            label = ArtifactType(class_id)
            x1, y1, x2, y2 = map(int, bbox)
            detections.append({"bbox": (x1, y1, x2 - x1, y2 - y1),"class": label, "confidence": confidence})

        
        self.artifact_found_ = len(detections) > 0
        
        
        
        for det in detections:
            x, y, w, h = det["bbox"]
            label = det["class"]
            conf = det["confidence"]
            roughDistance = self.roughDistanceOfArtifact(label, h)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(image, f"{label} with confidence {conf:.2f}% and at {roughDistance:.2f}", (x-30, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            self.rough_localise_artifact(det)

        
        
        
        image_detection_message = self.cv_bridge_.cv2_to_imgmsg(image, encoding="bgr8")
        self.image_detections_pub_.publish(image_detection_message)

        if self.artifact_found_:
            for artifact in self.rough_artifacts_:
                self.artifactInView_ = True
                if self.artifactSightingOfInterest_ == None:
                    self.artifactSightingOfInterest_ = artifact

                if self.planner_type_ == PlannerType.SELECT_AND_GO_TO_FRONTIER and self.should_check_detection(artifact):
                    self.closeRangeInspection_ = True
                    self.cancel_current_goal()
                    self.artifactSightingOfInterest_ = artifact

                    continue
                elif self.planner_type_ == PlannerType.CLOSERANGE_INSPECTION and self.should_check_detection(artifact) and artifact["label"] == self.artifactSightingOfInterest_["label"]:
                    self.artifactSightingOfInterest_ = artifact
        else:
            self.artifactInView_ = False
        
            
                    

        self.rough_publish_artifact_markers()


    def planner_do_close_range_inspection(self):
        selfPose = self.get_pose_2d()

        num_samples = 1000
        artifactToCheck = self.artifactSightingOfInterest_
        if artifactToCheck is None:
            return
        candidates = []
        cx = artifactToCheck["point"].x 
        cy = artifactToCheck["point"].y

        for _ in range(num_samples): #gaussian splatter like from A1 yipee!! except now its just circle
            theta = np.random.uniform(0, 2 * np.pi)
            # Sample a radius uniformly within the circle
            r = 3
            # Convert polar to cartesian coordinates
            dx = r * np.cos(theta)
            dy = r * np.sin(theta)
            x = cx + dx
            y = cy + dy
            

            reason = None
            passed = True

            obs_val = self.get_obstacle_value_at(x, y)
            if obs_val is None:
                passed = False
                reason = 'out_of_bounds'
            elif obs_val >= 100 or obs_val == -1:
                passed = False
                reason = 'invalid_terrain'

            elif not self._bresenham_clear_world_lenient((x, y), (cx, cy)):
                passed = False
                reason = 'blocked_los'

            if passed:
                dt_val = self.get_distance_value_at(x, y)
                if dt_val is None:
                    passed = False
                    reason = 'no_dt'
                else:
                    candidates.append((x, y, dt_val))
        
        if not candidates:
            goal_x, goal_y = cx, cy
            self.planner_go_to_pose2d(self.justlookingAtPoint(goal_x, goal_y, artifactToCheck["point"].x, artifactToCheck["point"].y))
            self.get_logger().info('Fallback to centroid')
        else:
            goal_x, goal_y, _ = max(candidates, key=lambda c: c[2])
            self.planner_go_to_pose2d(self.justlookingAtPoint(goal_x, goal_y, artifactToCheck["point"].x, artifactToCheck["point"].y))
            self.get_logger().info('ChoosingActualheatmap good')

        delta_x = selfPose.x - goal_x
        delta_y = selfPose.y - goal_y
        

        if ((delta_x**2 +delta_y**2)**0.5 < 1 and self.artifactInView_ == True):
            self.planner_type_ = PlannerType.SELECT_AND_GO_TO_FRONTIER
            self.addArtifact(self.artifactSightingOfInterest_)
            self.closeRangeInspection_ = False
            
        if (self.artifactInView_ == False):
            self.viewTicker = self.viewTicker + 1
        else:
            self.viewTicker = 0

        if self.viewTicker > 15:
            self.planner_type_ = PlannerType.SELECT_AND_GO_TO_FRONTIER
            self.closeRangeInspection_ = False
            self.viewTicker = 0



        

        
    def addArtifact(self, artifact):
        x = artifact["point"].x
        y = artifact["point"].y
        if not self.location_too_close_to_logged_artifacts(x, y, 3, artifact["label"]):
            self.artifacts_.append(artifact)
            self.travelling_sales_points_.append((self.get_pose_2d().x, self.get_pose_2d().y))

        

        self.publish_artifact_markers()
            

        

    def halfwayPoint(self, startPointX, startPointY, endPointX, endPointY):
        mid_x = (startPointX + endPointX) / 2
        mid_y = (startPointY + endPointY) / 2

        delta_x = endPointX - mid_x
        delta_y = endPointY - mid_y
        heading = math.atan2(delta_y, delta_x)

        return Pose2D(x=mid_x, y=mid_y, theta=heading)

    def justlookingAtPoint(self, startPointX, startPointY, endPointX, endPointY):
        delta_x = endPointX - startPointX
        delta_y = endPointY - startPointY
        heading = math.atan2(delta_y, delta_x)

        return Pose2D(x=startPointX, y=startPointY, theta=heading)
    
    def pointAtDistance(self, startPoint, endPoint, distance=3.0):
        dx = endPoint.x - startPoint.x
        dy = endPoint.y - startPoint.y
        total_distance = math.hypot(dx, dy)
    
        if total_distance == 0:
            return Pose2D(x=startPoint.x, y=startPoint.y, theta=0.0)
        ratio = distance / total_distance
        new_x = startPoint.x + dx * ratio
        new_y = startPoint.y + dy * ratio

        heading = math.atan2(dy, dx)
        
        return Pose2D(x=new_x, y=new_y, theta=heading)

                
    def should_check_detection(self, artifact):
        x = artifact["point"].x
        y = artifact["point"].y
        label = artifact["label"]
        if (self.location_too_close_to_logged_artifacts(x, y, 18, label)):
            return False

        return True
    
    def location_too_close_to_logged_artifacts(self, locationX, locationY, radius, type):
        for artifact in self.artifacts_:
            dx = locationX - artifact["point"].x
            dy = locationY - artifact["point"].y
            distance = (dx**2 + dy**2)**0.5
            if distance < radius and type == artifact["label"]:
                return True
        return False

    def roughDistanceOfArtifact(self, label, pixelHeight):
        image_width = 720
        image_height = 480
        h_fov = 2.0944  # stolen straight from the <gazebo reference="camera_link"> if i actually had time i would do the camera calibration with the ros thing.
        fx = image_width / (2 * math.tan(h_fov / 2))
        v_fov = 2 * math.atan((image_height / image_width) * math.tan(h_fov / 2))
        fy = image_height / (2 * math.tan(v_fov / 2))
        distance = (fy * ARTIFACT_HEIGHTS[label]) / pixelHeight

        return distance
    
    def detection_to_point(self, distance, x, y, w, h):
        image_width = 720
        image_height = 480
        h_fov = 2.0944
        u = x + w / 2
        v = y + h / 2
        fx = image_width / (2 * math.tan(h_fov / 2))
        v_fov = 2 * math.atan((image_height / image_width) * math.tan(h_fov / 2))
        fy = image_height / (2 * math.tan(v_fov / 2))
        cx = image_width / 2
        cy = image_height / 2
        x_cam = (u - cx) / fx
        y_cam = (v - cy) / fy
        z_cam = 1.0
        norm = math.sqrt(x_cam**2 + y_cam**2 + z_cam**2)
        x_cam, y_cam, z_cam = x_cam / norm, y_cam / norm, z_cam / norm
        p_cam = np.array([x_cam * distance, y_cam * distance, z_cam * distance])
        return p_cam


    def rough_localise_artifact(self, det=None): #just going to split this in half. This one uses the height of the pixel to figure out rough distance and location. Need to use depth sens for other one
        """
        INCOMPLETE:
        Compute the location of the artifact
        Save it to a list, publish rviz marker
        This version just uses the robot location rather than the artifact location
        You can find other examples of using RViz markers in the previous assignments template code
        """

        # Current location of the robot
        try:
            camera_pose = self.tf_buffer.lookup_transform('map', 'camera_rgb_optical_frame', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().error(f'Could not transform camera: {ex}')
            return

        if camera_pose == None:
            self.get_logger().warn(f'localise_artifact: robot_pose is None.')
            return
        
        

        label = det["class"]
        x, y, w, h = det["bbox"]

        
        dist = self.roughDistanceOfArtifact(label, h)

        pointCamera = self.detection_to_point(dist, x, y, w, h)


        trans = np.array([
            camera_pose.transform.translation.x,
            camera_pose.transform.translation.y,
            camera_pose.transform.translation.z
        ])

        
        quat = camera_pose.transform.rotation
        rot = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_matrix()
            
        p_world = rot @ pointCamera + trans
        
        # Compute the location of the artifact
        # This is currently INCOMPLETE
        point = Point()
        point.x = p_world[0]
        point.y = p_world[1]
        point.z = 1.0

        # Save it
        self.rough_artifacts_.append({"point": point, "label": label})
    
        
    

    def localise_artifact(self, det=None):
        """
        INCOMPLETE:
        Compute the location of the artifact
        Save it to a list, publish rviz marker
        This version just uses the robot location rather than the artifact location
        You can find other examples of using RViz markers in the previous assignments template code
        """

        # Current location of the robot
        try:
            camera_pose = self.tf_buffer.lookup_transform('map', 'camera_rgb_optical_frame', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().error(f'Could not transform camera: {ex}')
            return

        if camera_pose == None:
            self.get_logger().warn(f'localise_artifact: robot_pose is None.')
            return
        
        label = det["class"]
        x, y, w, h = det["bbox"]

        
        dist = self.roughDistanceOfArtifact(label, h)

        pointCamera = self.detection_to_point(dist, x, y, w, h)


        trans = np.array([
            camera_pose.transform.translation.x,
            camera_pose.transform.translation.y,
            camera_pose.transform.translation.z
        ])

        
        quat = camera_pose.transform.rotation
        rot = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_matrix()
            
        p_world = rot @ pointCamera + trans
        
        # Compute the location of the artifact
        # This is currently INCOMPLETE
        point = Point()
        point.x = p_world[0]
        point.y = p_world[1]
        point.z = 1.0

        # Save it
        self.artifacts_.append({"point": point, "label": label})

        # Publish the markers
        self.publish_artifact_markers()

    def publish_artifact_markers(self):
        """ Publish the artifact location markers"""

        # Update the locations
        self.marker_artifacts_.points = [artifact["point"] for artifact in self.artifacts_]

        # Create and publish the MarkerArray
        marker_array = MarkerArray()
        marker_array.markers = [self.marker_artifacts_]
        self.marker_pub_.publish(marker_array)

    def rough_publish_artifact_markers(self):
        """ Publish the artifact location markers"""

        # Update the locations
        self.rough_marker_artifacts_.points = [artifact["point"] for artifact in self.rough_artifacts_]

        # Create and publish the MarkerArray
        marker_array = MarkerArray()
        marker_array.markers = [self.rough_marker_artifacts_]
        self.rough_marker_pub_.publish(marker_array)
        self.rough_artifacts_ = []


    def planner_go_to_pose2d(self, pose2d):
        """Go to a provided 2d pose"""

        # Send a goal to navigate_to_pose with self.nav2_action_client_
        action_goal = NavigateToPose.Goal()
        action_goal.pose.header.stamp = self.get_clock().now().to_msg()
        action_goal.pose.header.frame_id = 'map'
        action_goal.pose.pose = pose2d_to_pose(pose2d)

        # Publish visualisation
        self.goal_pose_vis_.publish(action_goal.pose)

        # Decide whether to show feedback or not
        if self.get_parameter('print_feedback').value:
            feedback_method = self.feedback_callback
        else:
            feedback_method = None

        # Send goal to action server
        self.get_logger().warn(f'Sending goal [{pose2d.x:.2f}, {pose2d.y:.2f}]...')
        self.send_goal_future_ = self.nav2_action_client_.send_goal_async(
            action_goal,
            feedback_callback=feedback_method)
        self.send_goal_future_.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """The requested goal pose has been sent to the action server"""

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        # Goal accepted: get result when it's completed
        self.get_logger().warn(f'Goal accepted')
        self.get_result_future_ = goal_handle.get_result_async()
        self.get_result_future_.add_done_callback(self.goal_reached_callback)

    def feedback_callback(self, feedback_msg):
        """Monitor the feedback from the action server"""

        feedback = feedback_msg.feedback

        self.get_logger().info(f'{feedback.distance_remaining:.2f} m remaining')



    def goal_reached_callback(self, future):
        """Handle the result of the navigation goal."""
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached successfully!')
            self.ready_for_next_goal_ = True
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Goal was canceled before reaching the target.')
            self.ready_for_next_goal_ = False
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Goal was aborted (navigation failed).')
            self.ready_for_next_goal_ = True
        else:
            self.get_logger().warn(f'Goal ended with unknown status: {status}')
            self.ready_for_next_goal_ = False



    #code to stop current goal. For behaviour switching.
    def cancel_current_goal(self):      
        self.planner_go_to_pose2d(self.get_pose_2d())

    
    


    def planner_move_forwards(self, distance):
        """Simply move forward by the specified distance"""

        pose_2d = self.get_pose_2d()

        pose_2d.x += distance * math.cos(pose_2d.theta)
        pose_2d.y += distance * math.sin(pose_2d.theta)

        self.planner_go_to_pose2d(pose_2d)

    def planner_go_to_first_artifact(self):
        """Go to a pre-specified artifact location"""

        goal_pose2d = Pose2D(
            x = 18.1,
            y = 6.6,
            theta = math.pi/2
        )
        self.planner_go_to_pose2d(goal_pose2d)

    def planner_return_home(self):
        """Return to the origin"""

        goal_pose2d = Pose2D(
            x = 0.0,
            y = 0.0,
            theta = math.pi
        )
        self.planner_go_to_pose2d(goal_pose2d)

    def planner_random_walk(self):
        """Go to a random location, which may be invalid"""

        # Select a random location
        goal_pose2d = Pose2D(
            x = random.uniform(self.xlim_[0], self.xlim_[1]),
            y = random.uniform(self.ylim_[0], self.ylim_[1]),
            theta = random.uniform(0, 2*math.pi)
        )
        self.planner_go_to_pose2d(goal_pose2d)



    

    def plan_tsp_order(self, points_world):
        """
        Given a list of world-coordinate points, compute the optimal visiting order
        based on actual traversable paths through the occupancy map.

        Args:
            points_world: list of (x, y) in world coordinates

        Returns:
            ordered_indices: list of point indices in visit order
        """

        if self.obstacle_map_ is None:
            self.get_logger().error("No map data available!")
            return []

        def world_to_grid(x, y):
            gx = int((x - self.map_origin_[0]) / self.map_resolution_)
            gy = int((y - self.map_origin_[1]) / self.map_resolution_)
            gx = np.clip(gx, 0, self.obstacle_map_.shape[1]-1)
            gy = np.clip(gy, 0, self.obstacle_map_.shape[0]-1)
            return (gx, gy)

        points_grid = [world_to_grid(x, y) for (x, y) in points_world]

        def is_free(x, y):
            if 0 <= x < self.obstacle_map_.shape[1] and 0 <= y < self.obstacle_map_.shape[0]:
                return self.obstacle_map_[y, x] == 0
            return False

        def neighbors(x, y):
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = x+dx, y+dy
                if is_free(nx, ny):
                    yield (nx, ny)

        def astar(start, goal):
            """Return cost of shortest path using A* (grid-based)."""
            open_set = [(0, start)]
            g_score = {start: 0}
            gx, gy = goal
            while open_set:
                f, (x, y) = heapq.heappop(open_set)
                if (x, y) == goal:
                    return g_score[(x, y)]
                for nx, ny in neighbors(x, y):
                    ng = g_score[(x, y)] + np.hypot(nx - x, ny - y)
                    if (nx, ny) not in g_score or ng < g_score[(nx, ny)]:
                        g_score[(nx, ny)] = ng
                        f_score = ng + np.hypot(nx - gx, ny - gy)
                        heapq.heappush(open_set, (f_score, (nx, ny)))
            return np.inf  

        n = len(points_grid)
        cost_matrix = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                if i == j:
                    cost_matrix[i, j] = 0
                else:
                    cost = astar(points_grid[i], points_grid[j])
                    cost_matrix[i, j] = cost

        manager = pywrapcp.RoutingIndexManager(n, 1, 0)
        routing = pywrapcp.RoutingModel(manager)

        def distance_callback(from_idx, to_idx):
            return int(cost_matrix[manager.IndexToNode(from_idx)][manager.IndexToNode(to_idx)] * 100)  # scale to int
        transit_idx = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_idx)

        params = pywrapcp.DefaultRoutingSearchParameters()
        params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        solution = routing.SolveWithParameters(params)

        if not solution:
            self.get_logger().warn("TSP solver failed, returning original order.")
            return list(range(n))

        index = routing.Start(0)
        ordered_indices = []
        while not routing.IsEnd(index):
            ordered_indices.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))

        return ordered_indices

    def planner_travelsalesman(self):
        if self.travelsalesmanplan_ == None:
            self.travelling_sales_points_.append(self.widest)
            self.travelling_sales_points_.append(self.narrowest)
            self.travelsalesmanplan_ = self.plan_tsp_order(self.travelling_sales_points_)
            self.current_tsp_index = 0  

        tsp_order = self.travelsalesmanplan_
        point_idx = tsp_order[self.current_tsp_index]
        goal_point = self.travelling_sales_points_[point_idx]


        goal_pose2d = Pose2D(
            x=goal_point[0],
            y=goal_point[1],
            theta=random.uniform(0, 2*math.pi)
        )

        self.planner_go_to_pose2d(goal_pose2d)

        # advance index for next call
        self.current_tsp_index = (self.current_tsp_index + 1) % len(self.travelling_sales_points_)



    def planner_random_goal(self):
        """Go to a random location out of a predefined set"""

        # Hand picked set of goal locations
        random_goals = [[15.2, 2.2],
                        [30.7, 2.2],
                        [43.0, 11.3],
                        [36.6, 21.9],
                        [33.0, 30.4],
                        [40.4, 44.3],
                        [51.5, 37.8],
                        [16.0, 24.1],
                        [3.4, 33.5],
                        [7.9, 13.8],
                        [14.2, 37.7]]

        # Select a random location
        goal_valid = False
        while not goal_valid:
            idx = random.randint(0,len(random_goals)-1)
            goal_x = random_goals[idx][0]
            goal_y = random_goals[idx][1]

            # Only accept this goal if it's within the current costmap bounds
            if goal_x > self.xlim_[0] and goal_x < self.xlim_[1] and \
               goal_y > self.ylim_[0] and goal_y < self.ylim_[1]:
                goal_valid = True
            else:
                self.get_logger().warn(f'Goal [{goal_x}, {goal_y}] out of bounds')

        goal_pose2d = Pose2D(
            x = goal_x,
            y = goal_y,
            theta = random.uniform(0, 2*math.pi)
        )
        self.planner_go_to_pose2d(goal_pose2d)

    def main_loop(self):
        """
        Set the next goal pose and send to the action server
        See https://docs.nav2.org/concepts/index.html
        """
        
        # Don't do anything until SLAM is launched
        if not self.tf_buffer.can_transform(
                'map',
                'base_link',
                rclpy.time.Time()):
            self.get_logger().warn('Waiting for transform... Have you launched a SLAM node?')
            return

        #######################################################
        # Update flags related to the progress of the current planner

        # Check if previous goal still running
        if not self.ready_for_next_goal_:
            # self.get_logger().info(f'Previous goal still running')
            return

        self.ready_for_next_goal_ = False

        if self.planner_type_ == PlannerType.GO_TO_FIRST_ARTIFACT:
            self.get_logger().info('Successfully reached first artifact!')
            self.reached_first_artifact_ = True
        if self.planner_type_ == PlannerType.RETURN_HOME:
            self.get_logger().info('Successfully returned home!')
            self.returned_home_ = True

        #######################################################
        # Select the next planner to execute
        # Update this logic as you see fit!
        if not self.reached_first_artifact_:
            self.planner_type_ = PlannerType.GO_TO_FIRST_ARTIFACT
        #elif not self.returned_home_:
            #self.planner_type_ = PlannerType.RETURN_HOME
        elif not self.frontierClusters_ == [] and not self.closeRangeInspection_ == True: 
            self.planner_type_ = PlannerType.SELECT_AND_GO_TO_FRONTIER
        elif self.closeRangeInspection_ == True :
            self.planner_type_ = PlannerType.CLOSERANGE_INSPECTION
        else:
            self.planner_type_ = PlannerType.TRAVELSALESMAN

        #######################################################
        # Execute the planner by calling the relevant method
        # Add your own planners here!
        self.get_logger().info(f'Calling planner: {self.planner_type_.name}')
        if self.planner_type_ == PlannerType.MOVE_FORWARDS:
            self.planner_move_forwards(10)
        elif self.planner_type_ == PlannerType.GO_TO_FIRST_ARTIFACT:
            self.planner_go_to_first_artifact()
        elif self.planner_type_ == PlannerType.RETURN_HOME:
            self.planner_return_home()
        elif self.planner_type_ == PlannerType.RANDOM_WALK:
            self.planner_random_walk()
        elif self.planner_type_ == PlannerType.TRAVELSALESMAN:
            self.planner_travelsalesman()
        elif self.planner_type_ == PlannerType.RANDOM_GOAL:
            self.planner_random_goal()
        elif self.planner_type_ == PlannerType.SELECT_AND_GO_TO_FRONTIER:
            self.planner_choose_and_go_to_frontier_but_good()
        elif self.planner_type_ == PlannerType.HEADSNAP:
            self.planner_choose_and_go_to_frontier_but_good()
        elif self.planner_type_ == PlannerType.CLOSERANGE_INSPECTION:
            self.planner_do_close_range_inspection()
        else:
            self.get_logger().error('No valid planner selected')
            self.destroy_node()

    
        #######################################################

def main():
    # Initialise
    rclpy.init()

    # Create the cave explorer
    cave_explorer = CaveExplorer()

    while rclpy.ok():
        rclpy.spin(cave_explorer)