import math
import struct

import numpy as np
import scipy
import scipy.io
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class Map:
    def __init__(self, parent_logger, filename, resolution, do_distance_transform):
        self.parent_logger_ = parent_logger
        self.parent_logger_.warn(f'Loading map from {filename} (resolution = {resolution})')

        # Extract the map data from a file
        mat = scipy.io.loadmat(filename)

        # mat file contains variables:
        # xs ys zs texture texture_obstacles obstacle_map
        # xs and ys are arrays containing list of x and y values of grid
        # other variables are matrices over a grid of (x,y) values
        # obstacle_map is 2D matrix of 1's (=obstacle) and 0's (=no obstacle)
        # texture and texture_obstacles have an (r,g,b) colour for each (x,y) location
        # texture is the original colour, texture_obstacles is a false colour adding obstacles in red shade

        self.x_indices_ = mat['data']['xs'][0,0][0,:]
        self.y_indices_ = mat['data']['ys'][0,0][0,:]
        self.terrain_height_map_ = mat['data']['zs'][0,0]
        self.texture_ = mat['data']['texture'][0,0]
        self.texture_obstacles_ = mat['data']['texture_obstacles'][0,0]
        self.obstacle_map_ = mat['data']['obstacle_map'][0,0]

        # offset the terrain map z's
        z_multiplier = 2 # to make it look more interesting
        z_offset = np.nanmin(self.terrain_height_map_)
        self.terrain_height_map_ = z_multiplier * (self.terrain_height_map_ - z_offset)

        print('obstacle_map shape', self.obstacle_map_.shape)

        self.min_x_ = 0
        self.min_y_ = 0
        self.max_x_ = len(self.x_indices_)
        self.max_y_ = len(self.y_indices_)

        self.resolution_ = resolution

        # Declare the distance transform map
        self.distance_transform_map_ = None

        # Compute the distance transform
        self.do_distance_transform_ = do_distance_transform
        if self.do_distance_transform_:
            self.generate_distance_transform_map()

    def generate_distance_transform_map(self):
        """This function creates a distance transform map, based on self.obstacle_map_"""

        # Create an empty distance transform matrix
        # Same size as the obstacle map
        # Once complete, each cell in self.distance_transform_map_ will denote the distance
        # to the nearest obstacle, as defined in the self.obstacle_map_
        # To keep things simple, define distances in units of pixels (don't convert to meters)
        (rows, cols) = self.obstacle_map_.shape
        self.distance_transform_map_ = np.full((rows, cols), None)

        self.parent_logger_.warn('Computing distance transform map...')

        ####################
        ## YOUR CODE HERE ##
        ## Task 9         ##
        ####################

        

























        

        

    def generate_pcl2_msg(self, stamp_now):
        """The map is published as a PointCloud2 message, created here"""
        
        self.parent_logger_.warn('Generating terrain cloud...')

        points = []

        for i in range(0,self.texture_obstacles_.shape[0]-1):
            for j in range(0,self.texture_obstacles_.shape[1]-1):

                # extract the colour
                [r,g,b] = self.texture_obstacles_[i,j,:] # With red colouring of obstacles
                # [r,g,b] = self.texture_[i,j,:] # Original colouring

                # extract the location
                x = self.y_indices_[i]
                y = self.x_indices_[j]
                z = self.terrain_height_map_[i,j]

                # print(x, ', ', y, ', ', z)
                # print(r, ', ', g, ', ', b)

                # append
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
                # print hex(rgb)
                pt = [x, y, z, rgb]
                points.append(pt)

        # print('num points: ', len(points))

        # Assemble message
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
          PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
          PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
          PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
          ]

        header = Header()
        header.frame_id = "map"
        pc2_msg = point_cloud2.create_cloud(header, fields, points)
        pc2_msg.header.stamp = stamp_now

        return pc2_msg

    def generate_pcl2_msg_distance_transform(self, stamp_now):
        """The map is published as a PointCloud2 message for the distance transform, created here"""

        # Skip if hasn't been created
        if self.do_distance_transform_ == False:
            return None

        
        self.parent_logger_.warn('Generating distance transform cloud...')

        # Get the maximum distance, use as scaling
        # Convert the Nones to zeros, just in case (there shouldn't be any)
        self.distance_transform_map_[(self.distance_transform_map_==None)] = 0
        max_distance = np.max(self.distance_transform_map_)

        self.parent_logger_.warn('max_distance: ' + str(max_distance))

        points = []

        for i in range(0,self.texture_obstacles_.shape[0]-1):
            for j in range(0,self.texture_obstacles_.shape[1]-1):

                # extract the colour
                #[r,g,b] = self.texture_obstacles_[i,j,:] # With red colouring of obstacles
                # [r,g,b] = self.texture_[i,j,:] # Original colouring

                # distance transform colouring (black to white)
                if self.distance_transform_map_[i,j] == 0:
                    d = 0
                    r = 128
                    g = 0
                    b = 0
                else:
                    d = int(255 * self.distance_transform_map_[i,j] / max_distance)
                    r = d
                    g = d
                    b = d

                # extract the location
                x = self.y_indices_[i]
                y = self.x_indices_[j]
                z = self.terrain_height_map_[i,j]+0.01 # 1*float(d)/255

                # print(x, ', ', y, ', ', z)
                # print(r, ', ', g, ', ', b)

                # append
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
                # print hex(rgb)
                pt = [x, y, z, rgb]
                points.append(pt)

        # print('num points: ', len(points))

        # Assemble message
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
          PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
          PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
          PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
          ]

        header = Header()
        header.frame_id = "map"
        pc2_msg = point_cloud2.create_cloud(header, fields, points)
        pc2_msg.header.stamp = stamp_now

        return pc2_msg

    def generate_path_msg(self, path, z_offset=0):
        """Convert the points in the path into a Path message"""

        msg = Path()
        msg.header.frame_id = 'map'
        for node in path:
            p = self.pixel_to_world(node.x, node.y)
            pose = PoseStamped()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = p[2]+0.125+z_offset
            pose.pose.orientation.w = 1.0
            pose.header.frame_id = 'map'
            msg.poses.append(pose)
        return msg

    def pixel_to_world(self, x, y):
        """Convert pixel coordinates to world coordinates"""

        x = math.floor(x)
        y = math.floor(y)
        try:
            return [self.y_indices_[x], self.x_indices_[y], self.terrain_height_map_[x,y]]
        except Exception as e:
            print(e)
            print("invalid coordinates in pixel_to_world(): ", x, ", ", y)
            return [0.0, 0.0, 0.0]    

    def world_to_pixel(self, x, y):
        """Convert world coordinates to pixel coordinates"""

        try:
            return [np.argmax(self.y_indices_> x), np.argmax(self.x_indices_> y)]
        except Exception as e:
            print(e)
            print("invalid coordinates in world_to_pixel(): ", x, ", ", y)
            return [0, 0]  

    def is_occupied(self, x, y):
        """Whether the given point is occupied"""

        shape = self.obstacle_map_.shape

        # Out of bounds
        if x < 0 or x >= shape[0] or y < 0 or y >= shape[1]:
            return True

        if self.obstacle_map_[x,y] < 0.5:
            # Not an obstacle
            return False
        else:
            # Obstacle
            return True
