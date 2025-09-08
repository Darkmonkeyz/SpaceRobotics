#!/usr/bin/env python3

import copy

from geometry_msgs.msg import Point

from.graph import is_occluded

class PathSmoother():
    def __init__(self, parent_node, graph, alpha, beta):
        self.parent_node_ = parent_node
        self.graph_ = graph
        
        self.alpha_ = alpha
        self.beta_ = beta

        self.path_smooth_ = []
        
        
    def sumEuclidianDistance(self, previousPath, newPath):
        sum = 0
        for i in range(len(previousPath)):
            sum += (previousPath[i].x - newPath[i].x)**2 + (previousPath[i].y - newPath[i].y)**2
            #self.parent_node_.get_logger().info(f'sum = {sum}')
        return sum

    def smooth_path(self, path_nodes):
        """Smooth the path to remove sharp corners resulting from the grid-based planning"""

        self.parent_node_.get_logger().info('Smoothing path...')

        # Convert into into a geometry_msgs.Point[]
        path = []

        for node in path_nodes:
            p = Point()
            p.x = float(node.x)
            p.y = float(node.y)
            path.append(p)

        # Initialise the smooth path
        path_smooth = copy.deepcopy(path)

        # Loop until the smoothing converges
        # In each iteration, update every waypoint except the first and last waypoint

        ####################
        ## YOUR CODE HERE ##
        ## Task 5         ##
        ####################
        runningFlag = True
        epsilon = 0.001
        previousPath = copy.deepcopy(path)
        newPath = copy.deepcopy(path)
        #self.parent_node_.get_logger().info(f'alpha: {self.alpha_} beta: {self.beta_}')
        while runningFlag == True:
            for i in range(1, len(path)-1):
                newPath[i].x = newPath[i].x - (self.alpha_ + 2* self.beta_)*newPath[i].x + self.alpha_*path[i].x + self.beta_*(newPath[i-1].x + newPath[i+1].x)
                #self.parent_node_.get_logger().info(f'newx = {newPath[i].x} oldx = {previousPath[i].x}')
                newPath[i].y = newPath[i].y - (self.alpha_ + 2* self.beta_)*newPath[i].y + self.alpha_*path[i].y + self.beta_*(newPath[i-1].y + newPath[i+1].y)
            
            for i in range(len(path)-1):
               if is_occluded(self.graph_.map_.obstacle_map_, [newPath[i].x, newPath[i].y], [newPath[i+1].x, newPath[i+1].y]):
                    newPath = copy.deepcopy(previousPath)
            
            #self.parent_node_.get_logger().info(f'sumDistance = {self.sumEuclidianDistance(previousPath, newPath)}')
            
            if (self.sumEuclidianDistance(previousPath, newPath) < epsilon):
                runningFlag = False
                
            previousPath = copy.deepcopy(newPath)




















        self.path_smooth_ = newPath