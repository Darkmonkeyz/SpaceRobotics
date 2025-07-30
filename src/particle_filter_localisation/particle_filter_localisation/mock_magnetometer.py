#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


def wrap_angle(angle):
    """Function to wrap an angle between 0 and 2*Pi"""
    while angle < 0.0:
        angle = angle + 2 * math.pi

    while angle > 2 * math.pi:
        angle = angle - 2 * math.pi

    return angle

def random_normal(stddev):
    """Returns a random number with normal distribution, 0 mean and a standard deviation of 'stddev'"""
    return np.random.normal(0.0, stddev)


class Magnetometer(Node):
    def __init__(self):
        super().__init__('magnetometer')

        # Parameters
        self.noise_stddev_ = 0.349066 # 20 degrees
        self.msg_time_ = self.get_clock().now()
        self.msg_period_ = rclpy.duration.Duration(nanoseconds=1e9)

        # Subscribers
        self.pose_sub_ = self.create_subscription(Odometry, 'ground_truth', self.pose_callback, 1) # Subscribes to ground truth location

        # Publishers
        self.compass_pub_ = self.create_publisher(Float32, 'compass', 1)

    def pose_callback(self, odom_msg):
        """Receive an odometry message"""
        
        time_now = self.get_clock().now()

        if self.get_clock().now() - self.msg_time_ >= self.msg_period_:
            self.msg_time_ = time_now

            # Convert it to a heading in radians
            pose_theta = 2. * math.acos(odom_msg.pose.pose.orientation.w)

            if odom_msg.pose.pose.orientation.z < 0.:
                pose_theta = -pose_theta

            # Add Guassian noise
            pose_theta += random_normal(self.noise_stddev_)

            # Wrap it
            pose_theta = wrap_angle(pose_theta)

            # Publish the compass reading
            msg = Float32(data=pose_theta)
            self.compass_pub_.publish(msg)

def main():
    # Initialise
    rclpy.init()

    # Create the magnetometer
    magnetometer = Magnetometer()

    while rclpy.ok():
        rclpy.spin(magnetometer)

if __name__ == '__main__':
    main()
