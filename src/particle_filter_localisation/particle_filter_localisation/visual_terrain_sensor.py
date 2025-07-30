#!/usr/bin/env python3

import math
import struct

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, Int32
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def wrap_angle(angle):
    """Function to wrap an angle between 0 and 2*Pi"""

    while angle < 0.0:
        angle = angle + 2 * math.pi

    while angle > 2 * math.pi:
        angle = angle - 2 * math.pi

    return angle

def random_normal(stddev):
    """Returns a random number with normal distribution, 0 mean and a standard deviation of stddev"""
    return np.random.normal(0.0, stddev)

class VisualTerrainMap:

    confusion_matrix = np.array(
        [[0.90, 0.02, 0.02, 0.02, 0.02, 0.02],
         [0.02, 0.90, 0.02, 0.02, 0.02, 0.02],
         [0.02, 0.02, 0.90, 0.02, 0.02, 0.02],
         [0.02, 0.02, 0.02, 0.90, 0.02, 0.02],
         [0.02, 0.02, 0.02, 0.02, 0.90, 0.02],
         [0.02, 0.02, 0.02, 0.02, 0.02, 0.90]])

    def __init__(self, filename_class_map, filename_class_colour_map, filename_obstacles_map,
                 resolution, origin):

        # read in the images
        self.class_map_ = cv2.imread(filename_class_map)
        self.class_colour_map_ = cv2.imread(filename_class_colour_map)
        self.obstacles_map_ = cv2.imread(filename_obstacles_map)

        self.class_map_ = self.class_map_[:,:,0]

        # print(self.class_map_)
        # print(self.class_colour_map_)
        #print(self.obstacles_map_)

        # figure out the transformation
        self.resolution_ = resolution
        self.origin_ = origin

        #print(self.resolution_)
        #print(self.origin_)

        # output it as a colored point cloud with labels, for rviz
        self.shape_ = self.class_colour_map_.shape

        points = []

        for i in range(0,self.shape_[0]-1):
            for j in range(0,self.shape_[1]-1):

                # extract the colour
                if self.obstacles_map_[i,j,0] == 0:
                    # obstacle
                    r = 0
                    g = 0
                    b = 0
                    a = 255
                else:
                    # not an obstacle
                    col = self.class_colour_map_[i,j,:]
                    r = col[2]
                    g = col[1]
                    b = col[0]
                    a = 255

                # extract the location
                x = j * self.resolution_ + self.origin_[0]
                y = (self.shape_[0]-1 - i) * self.resolution_ + self.origin_[1]
                z = -0.1               

                # append
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                # print hex(rgb)
                pt = [x, y, z, rgb]
                points.append(pt)

        # Assemble message
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
          PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
          PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
          PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1)
          ]

        header = Header()
        header.frame_id = "odom"
        self.pc2_msg_ = point_cloud2.create_cloud(header, fields, points)

        # Reverse engineer the colour scheme for visualisation
        self.colormap_ = []

        for idx in range(len(self.confusion_matrix[0])):

            found = False

            # Find the idx in the image
            for i in range(self.shape_[0]):
                for j in range(self.shape_[1]):
                    if self.class_map_[i, j] == idx:
                        found = True
                        break
                if found == True:
                    break

            # If it exists, extract the colour from the coloured map
            if found:
            
                # Store it
                col = self.class_colour_map_[i,j,:]
                col = np.flip(col)
                self.colormap_.append(col)

            else:
                # Add arbitrary colour
                self.colormap_.append([0, 0, 0])

    def get_ground_truth(self, odom):

        # Transform to pixel coordinates
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        j = round((x - self.origin_[0]) / self.resolution_)
        i = round(self.shape_[0]-1 - (y - self.origin_[1]) / self.resolution_)

        # Query map
        ground_truth_class = self.class_map_[i, j]

        return ground_truth_class



class VisualTerrainSensor(Node):
    def __init__(self):
        super().__init__('visual_terrain_sensor')

        # Parameters
        self.declare_parameter('filename_class_map', '')
        self.declare_parameter('filename_class_colour_map', '')
        self.declare_parameter('filename_obstacles_map', '')
        self.declare_parameter('resolution', 0.0)
        self.declare_parameter('origin', [0.0, 0.0, 0.0])

        # Read in the terrain map
        self.visual_terrain_map_ = VisualTerrainMap(
            self.get_parameter('filename_class_map').value,
            self.get_parameter('filename_class_colour_map').value,
            self.get_parameter('filename_obstacles_map').value,
            self.get_parameter('resolution').value,
            self.get_parameter('origin').value
        )

        # Parameters
        self.noise_stddev_ = 0.349066 # 20 degrees        

        # Publishers
        self.terrain_pub_ = self.create_publisher(Int32, 'terrain_class', 1)
        self.terrain_viz_pub_ = self.create_publisher(PointCloud2, 'terrain_viz',
                                                      QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                            depth=1))

        # Visualisation marker for debugging
        self.marker2_ = Marker()
        self.marker2_.header.frame_id = "odom"
        self.marker2_.ns = "ns3"
        self.marker2_.id = 1
        self.marker2_.type = Marker.CYLINDER
        self.marker2_.action = Marker.ADD
        self.marker2_.pose.position.x = 0.0
        self.marker2_.pose.position.y = 0.0
        self.marker2_.pose.position.z = 0.0
        self.marker2_.pose.orientation.x = 0.0
        self.marker2_.pose.orientation.y = 0.0
        self.marker2_.pose.orientation.z = 0.0
        self.marker2_.pose.orientation.w = 1.0
        self.marker2_.scale.x = 0.4
        self.marker2_.scale.y = 0.4
        self.marker2_.scale.z = 0.015
        self.marker2_.color.a = 1.0
        self.marker2_.color.r = 1.0
        self.marker2_.color.g = 1.0
        self.marker2_.color.b = 1.0
        self.marker3_ = Marker()
        self.marker3_.header.frame_id = "odom"
        self.marker3_.ns = "ns3"
        self.marker3_.id = 2
        self.marker3_.type = Marker.CYLINDER
        self.marker3_.action = Marker.ADD
        self.marker3_.pose.position.x = 0.0
        self.marker3_.pose.position.y = 0.0
        self.marker3_.pose.position.z = 0.0
        self.marker3_.pose.orientation.x = 0.0
        self.marker3_.pose.orientation.y = 0.0
        self.marker3_.pose.orientation.z = 0.0
        self.marker3_.pose.orientation.w = 1.0
        self.marker3_.scale.x = 0.45
        self.marker3_.scale.y = 0.45
        self.marker3_.scale.z = 0.01
        self.marker3_.color.a = 1.0
        self.marker3_.color.r = 0.0
        self.marker3_.color.g = 0.0
        self.marker3_.color.b = 0.0
        self.marker_pub_ = self.create_publisher(MarkerArray, 'marker', 100)

        self.visual_terrain_map_.pc2_msg_.header.stamp = self.get_clock().now().to_msg()
        self.terrain_viz_pub_.publish(self.visual_terrain_map_.pc2_msg_ )

        # Subscribers
        self.pose_sub_ = self.create_subscription(Odometry, 'ground_truth', self.pose_callback, 1) # Subscribes to ground truth location

    def pose_callback(self, odom_msg):
        """Receive an odometry message and publish a new message"""

        # Get map class
        ground_truth_class = self.visual_terrain_map_.get_ground_truth(odom_msg)

        # Add confusion noise
        distribution = self.visual_terrain_map_.confusion_matrix[ground_truth_class]
        observed_class = np.random.choice(np.arange(len(distribution)), p=distribution)

        # print("observation", ground_truth_class, observed_class)

        # Publish it
        msg = Int32(data=int(observed_class))
        self.terrain_pub_.publish(msg)

        # Visualisation for debugging
        col = self.visual_terrain_map_.colormap_[observed_class]
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        self.marker2_.pose.position.x = x
        self.marker2_.pose.position.y = y
        self.marker2_.color.r = col[0] / 255.0
        self.marker2_.color.g = col[1] / 255.0
        self.marker2_.color.b = col[2] / 255.0

        self.marker3_.pose.position.x = x
        self.marker3_.pose.position.y = y
                
        marker_array = MarkerArray()
        marker_array.markers = [self.marker2_, self.marker3_]
        self.marker_pub_.publish(marker_array)


def main():
    # Initialise
    rclpy.init()

    # Create the terrain sensor
    visual_terrain_sensor = VisualTerrainSensor()

    while rclpy.ok():
        rclpy.spin(visual_terrain_sensor)

if __name__ == '__main__':
    main()

