import math
import random

import matplotlib
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from .graph_search import GraphSearch
import numpy as np


class Graph:
    def __init__(self, parent_logger, map, grid_step_size, prm_num_nodes, use_energy_costs, use_prm, show_connectivity, prm_max_edge_length, use_distance_transform_graph):
        self.parent_logger_ = parent_logger
        self.map_ = map

        self.use_energy_costs_ = use_energy_costs
        self.show_connectivity_ = show_connectivity

        self.nodes_ = []
        self.groups_ = None

        # Visualisation Marker (you can ignore this)
        self.marker_nodes_ = Marker()
        self.marker_nodes_.header.frame_id = "map"
        self.marker_nodes_.ns = "nodes"
        self.marker_nodes_.id = 0
        self.marker_nodes_.type = Marker.POINTS
        self.marker_nodes_.action = Marker.ADD
        self.marker_nodes_.pose.position.x = 0.0
        self.marker_nodes_.pose.position.y = 0.0
        self.marker_nodes_.pose.position.z = 0.0
        self.marker_nodes_.pose.orientation.x = 0.0
        self.marker_nodes_.pose.orientation.y = 0.0
        self.marker_nodes_.pose.orientation.z = 0.0
        self.marker_nodes_.pose.orientation.w = 1.0
        self.marker_nodes_.scale.x = .03
        self.marker_nodes_.scale.y = .03
        self.marker_nodes_.scale.z = .03
        self.marker_nodes_.color.a = 1.0
        self.marker_nodes_.color.r = 1.0
        self.marker_nodes_.color.g = 0.2
        self.marker_nodes_.color.b = 0.2

        self.marker_start_ = Marker()
        self.marker_start_.header.frame_id = "map"
        self.marker_start_.ns = "start"
        self.marker_start_.id = 0
        self.marker_start_.type = Marker.POINTS
        self.marker_start_.action = Marker.ADD
        self.marker_start_.pose.position.x = 0.0
        self.marker_start_.pose.position.y = 0.0
        self.marker_start_.pose.position.z = 0.0
        self.marker_start_.pose.orientation.x = 0.0
        self.marker_start_.pose.orientation.y = 0.0
        self.marker_start_.pose.orientation.z = 0.0
        self.marker_start_.pose.orientation.w = 1.0
        self.marker_start_.scale.x = .2
        self.marker_start_.scale.y = .2
        self.marker_start_.scale.z = .2
        self.marker_start_.color.a = 1.0
        self.marker_start_.color.r = 1.0
        self.marker_start_.color.g = 1.0
        self.marker_start_.color.b = 0.2

        self.marker_visited_ = Marker()
        self.marker_visited_.header.frame_id = "map"
        self.marker_visited_.ns = "visited"
        self.marker_visited_.id = 0
        self.marker_visited_.type = Marker.POINTS
        self.marker_visited_.action = Marker.ADD
        self.marker_visited_.pose.position.x = 0.0
        self.marker_visited_.pose.position.y = 0.0
        self.marker_visited_.pose.position.z = 0.0
        self.marker_visited_.pose.orientation.x = 0.0
        self.marker_visited_.pose.orientation.y = 0.0
        self.marker_visited_.pose.orientation.z = 0.0
        self.marker_visited_.pose.orientation.w = 1.0
        self.marker_visited_.scale.x = .05
        self.marker_visited_.scale.y = .05
        self.marker_visited_.scale.z = .05
        self.marker_visited_.color.a = 1.0
        self.marker_visited_.color.r = 0.2
        self.marker_visited_.color.g = 0.2
        self.marker_visited_.color.b = 1.0

        self.marker_unvisited_ = Marker()
        self.marker_unvisited_.header.frame_id = "map"
        self.marker_unvisited_.ns = "unvisited"
        self.marker_unvisited_.id = 0
        self.marker_unvisited_.type = Marker.POINTS
        self.marker_unvisited_.action = Marker.ADD
        self.marker_unvisited_.pose.position.x = 0.0
        self.marker_unvisited_.pose.position.y = 0.0
        self.marker_unvisited_.pose.position.z = 0.0
        self.marker_unvisited_.pose.orientation.x = 0.0
        self.marker_unvisited_.pose.orientation.y = 0.0
        self.marker_unvisited_.pose.orientation.z = 0.0
        self.marker_unvisited_.pose.orientation.w = 1.0
        self.marker_unvisited_.scale.x = .06
        self.marker_unvisited_.scale.y = .06
        self.marker_unvisited_.scale.z = .06
        self.marker_unvisited_.color.a = 1.0
        self.marker_unvisited_.color.r = 0.3
        self.marker_unvisited_.color.g = 1.0
        self.marker_unvisited_.color.b = 0.3
        
        self.marker_edges_ = Marker()
        self.marker_edges_.header.frame_id = "map"
        self.marker_edges_.ns = "edges"
        self.marker_edges_.id = 0
        self.marker_edges_.type = Marker.LINE_LIST
        self.marker_edges_.action = Marker.ADD
        self.marker_edges_.pose.position.x = 0.0
        self.marker_edges_.pose.position.y = 0.0
        self.marker_edges_.pose.position.z = 0.0
        self.marker_edges_.pose.orientation.x = 0.0
        self.marker_edges_.pose.orientation.y = 0.0
        self.marker_edges_.pose.orientation.z = 0.0
        self.marker_edges_.pose.orientation.w = 1.0
        if self.use_energy_costs_:
            # Make easier to see
            self.marker_edges_.scale.x = 0.025
            self.marker_edges_.scale.y = 0.025
            self.marker_edges_.scale.z = 0.025
        else:
            self.marker_edges_.scale.x = 0.008
            self.marker_edges_.scale.y = 0.008
            self.marker_edges_.scale.z = 0.008
        self.marker_edges_.color.a = 1.0
        self.marker_edges_.color.r = 1.0
        self.marker_edges_.color.g = 1.0
        self.marker_edges_.color.b = 0.4
        self.marker_edges_.colors = []
        
        # Select between grid or PRM
        if use_distance_transform_graph:
            self.create_distance_transform_graph(prm_num_nodes, prm_max_edge_length)
        else:        
            if use_prm:
                self.create_PRM(prm_num_nodes, prm_max_edge_length)
            else:
                self.create_grid(grid_step_size)

        # Compute the graph connectivity
        if self.show_connectivity_:
            self.find_connected_groups()

    def create_grid(self, grid_step_size):
        """Build regular grid across map"""

        # Create nodes
        idx = 0
        for x in range(self.map_.min_x_, self.map_.max_x_-1, grid_step_size):
            for y in range(self.map_.min_y_, self.map_.max_y_-1, grid_step_size):

                # Check if it is occupied
                occupied = self.map_.is_occupied(x,y)

                # Create the node
                if not occupied:
                    self.nodes_.append(Node(x,y,idx))
                    idx = idx + 1

        # Create edges
        count = 0
        # distance_threshold = math.sqrt(2*(grid_step_size*1.01)**2) # Chosen so that diagonals are connected, but not 2 steps away
        distance_threshold = grid_step_size*1.01 # only 4 connected
        for node_i in self.nodes_:
            count = count + 1
            # self.parent_logger_.info(f'{count} of {len(self.nodes_)}')

            for node_j in self.nodes_:

                # Don't create edges to itself
                if node_i != node_j:

                    # Check if the nodes are close to each other
                    distance = node_i.distance_to(node_j)
                    if distance < distance_threshold:

                        # Check edge is collision free
                        if node_i.is_connected(self.map_.obstacle_map_, node_j):

                            if self.use_energy_costs_:

                                # Set the edge costs as estimated energy consumption

                                ####################
                                ## YOUR CODE HERE ##
                                ## TASK 6         ##
                                ####################
                                energy_cost = distance # Comment this out once you've done this Task
                                # energy_cost = ??
                                














                            else:

                                # Define the edge cost as standard 2D Euclidean distance in pixel coordinates
                                energy_cost = distance

                            # Create the edge
                            node_i.neighbours.append(node_j)
                            node_i.neighbour_costs.append(energy_cost)


    def create_PRM(self, num_nodes, distance_threshold):
        """Build PRM across map"""
        
        idx = 0

        # Create nodes
        # hint: it will be similar to the create_grid method above

        ####################
        ## YOUR CODE HERE ##
        ## Task 7         ##
        ####################

        # while len(self.nodes_) < num_nodes:
        #   ??









        # Create edges
        count = 0
        for node_i in self.nodes_:
            count = count + 1
            # self.parent_logger_.info(f'{count} of {len(self.nodes_)}')

            for node_j in self.nodes_:

                # Don't create edges to itself
                if node_i != node_j:

                    # Check if the nodes are close to each other
                    distance = node_i.distance_to(node_j)
                    if distance < distance_threshold:

                        # Check edge is collision free
                        if node_i.is_connected(self.map_.obstacle_map_, node_j):

                            if self.use_energy_costs_:

                                ############################
                                ## YOUR CODE HERE         ##
                                ## TASK 6 -- after TASK 7 ##
                                ############################
                                energy_cost = distance # Comment this out once you've done this Task
                                # energy_cost = ??
                                













                            else:

                                # Define the edge cost as standard 2D Euclidean distance in pixel coordinates
                                energy_cost = distance

                            # Create the edge
                            node_i.neighbours.append(node_j)
                            node_i.neighbour_costs.append(energy_cost)


    def create_distance_transform_graph(self, num_nodes, distance_threshold):
        """Build distance transform graph that places vertices at peaks and saddle points of distance transform"""
        
        # Create nodes
        # hint: it will be similar to the create_PRM method above

        idx = 0

        ####################
        ## YOUR CODE HERE ##
        ## Task 10        ##
        ####################
        distance_transform_map = self.map_.distance_transform_map_





















        # Create edges
        count = 0
        for node_i in self.nodes_:
            count = count + 1
            self.parent_logger_.info(f'{count} of {len(self.nodes_)}')

            for node_j in self.nodes_:

                # Don't create edges to itself
                if node_i != node_j:

                    # Check if the nodes are close to each other
                    distance = node_i.distance_to(node_j)
                    if distance < distance_threshold:

                        # Check edge is collision free
                        if node_i.is_connected(self.map_.obstacle_map_, node_j):

                            if self.use_energy_costs_:

                                ############################
                                ## YOUR CODE HERE         ##
                                ## TASK 6 -- after TASK 10##
                                ############################
                                energy_cost = distance # Comment this out once you've done this Task
                                # energy_cost = ??
                                












                            else:

                                # Define the edge cost as standard 2D Euclidean distance in pixel coordinates
                                energy_cost = distance

                            # Create the edge
                            node_i.neighbours.append(node_j)
                            node_i.neighbour_costs.append(energy_cost)


    def get_closest_node(self, xy):
        """
        Find closest node to the given xy point
            input: xy is a point in the form of an array, such that x=xy[0] and y=xy[1]. 
            output: return the index of the node in self.nodes_ that has the lowest Euclidean distance to the point xy. 
        """

        best_dist = 999999999
        best_index = None
        for i in range(len(self.nodes_)):

            ####################
            ## YOUR CODE HERE ##
            ## Task 1         ##
            ####################
            
            xDelt = xy[0]-self.nodes_[i].x
            
            yDelt = xy[1]-self.nodes_[i].y
            dist = math.sqrt((xDelt*xDelt) + (yDelt * yDelt))         
            if best_dist > dist:
                best_dist = dist
                best_index = i
            





            
        
        return best_index


    def find_connected_groups(self):
        """
        Return a list of numbers that has length equal to the number of nodes
        The number in the list refers to an arbitrary "group number"
        Two nodes should be in the same group if you can find a path from the first node to the second node
        Also see GraphSearch.find_connected_nodes()
        """

        # Setup GraphSearch object
        graph_search = GraphSearch(self.parent_logger_, self, False, 0.0)

        # Setup groups list. Initially put all nodes in group "0"
        groups = [0]*len(self.nodes_)

        ####################
        ## YOUR CODE HERE ##
        ## Task 8         ##
        ####################

        # Current group
        group_number = 1













        # Save it here so it will show up in the visualisation as different colours
        self.groups_ = groups


    def generate_marker_msgs(self):
        """Create visualisation markers for the graph"""

        if self.groups_ == None:
            self.marker_nodes_.points = []
            for node_i in self.nodes_:
                p = self.map_.pixel_to_world(node_i.x, node_i.y)
                point = Point(x=p[0], y=p[1], z=p[2]+0.1)
                self.marker_nodes_.points.append(point)
        else:
            # Plot each group with a different colour
            cmap = matplotlib.cm.get_cmap('Set1')
            colors = cmap.colors[0:-2]

            # print(self.groups_)

            self.marker_nodes_.points = []
            self.marker_nodes_.colors = []

            if self.show_connectivity_:
                self.marker_nodes_.scale.x = .06
                self.marker_nodes_.scale.y = .06
                self.marker_nodes_.scale.z = .06

            for node_idx in range(len(self.nodes_)):

                # print("node_idx", node_idx)
                node_i = self.nodes_[node_idx]
                p = self.map_.pixel_to_world(node_i.x, node_i.y,)
                point = Point(x=p[0], y=p[1], z=p[2]+0.1)
                self.marker_nodes_.points.append(point)
                
                # Set a colour
                group_id = self.groups_[node_idx]
                c = colors[group_id % len(colors)]
                col = ColorRGBA(r=c[0], g=c[1], b=c[2], a=1.0)
                self.marker_nodes_.colors.append(col)

        if self.use_energy_costs_:
            # Computer upper and lower limits of energy costs
            costs_list = []
            if self.use_energy_costs_:
                for node_i in self.nodes_:
                    for node_j_index in range(len(node_i.neighbour_costs)):
                        # Use color based on the larger of i->j or j->i cost
                        cost_ij = node_i.neighbour_costs[node_j_index]
                        node_j = node_i.neighbours[node_j_index]

                        # find i in j's neighbours
                        found = False
                        for k in range(len(node_j.neighbours)):
                            if node_j.neighbours[k].idx == node_i.idx:
                                found = True
                                break

                        cost = cost_ij
                        if found:
                            cost_ji = node_j.neighbour_costs[k]
                            if cost_ji > cost_ij:
                                cost = cost_ji
                        costs_list.append(cost)
            costs_min = min(costs_list)
            costs_max = max(costs_list)


        self.marker_edges_.points = []
        self.marker_edges_.colors = []
        for node_i in self.nodes_:
            for node_j_index in range(len(node_i.neighbours)):

                node_j = node_i.neighbours[node_j_index]
                

                p = self.map_.pixel_to_world(node_i.x, node_i.y)
                point = Point(x=p[0], y=p[1], z=p[2]+0.05)
                self.marker_edges_.points.append(point)
                p = self.map_.pixel_to_world(node_j.x, node_j.y)
                point = Point(x=p[0], y=p[1], z=p[2]+0.05)
                self.marker_edges_.points.append(point)

                if self.use_energy_costs_:
                    # Use color based on the larger of i->j or j->i cost
                    cost_ij = node_i.neighbour_costs[node_j_index]
                    node_j = node_i.neighbours[node_j_index]

                    # find i in j's neighbours
                    found = False
                    for k in range(len(node_j.neighbours)):
                        if node_j.neighbours[k].idx == node_i.idx:
                            found = True
                            break

                    cost = cost_ij
                    if found:
                        cost_ji = node_j.neighbour_costs[k]
                        if cost_ji > cost_ij:
                            cost = cost_ji

                    relative_cost = ( (cost-costs_min) / (costs_max - costs_min) )
                    # yellow = low cost
                    # red = high cost
                    r = 1.0
                    g = ( 1.0 - relative_cost ) ** 2.0
                    b = ( 1.0 - relative_cost ) ** 2.0
                    #r = 1.0 * relative_cost
                    #g = ( 1.0 - relative_cost ) ** 4
                    #b = 0.0
                    # if cost < 0.5:
                    #     r = relative_cost*2
                    #     g = 1.0
                    #     b = relative_cost*2
                    # else:
                    #     r = 1.0
                    #     g = (1.0-relative_cost)*2
                    #     b = (1.0-relative_cost)*2

                    col = ColorRGBA(r=r, g=g, b=b, a=1.0)
                    self.marker_edges_.colors.append(col)
                    self.marker_edges_.colors.append(col)

        return self.marker_nodes_, self.marker_edges_
    
    def generate_start_end_markers(self, start_idx, goal_idx):
        """Only publish the start and end markers once per path"""

        self.marker_start_.points = []
        node_i = self.nodes_[start_idx]
        p = self.map_.pixel_to_world(node_i.x, node_i.y)
        point = Point(x=p[0], y=p[1], z=p[2]+0.09)
        self.marker_start_.points.append(point)

        node_i = self.nodes_[goal_idx]
        p = self.map_.pixel_to_world(node_i.x, node_i.y)
        point = Point(x=p[0], y=p[1], z=p[2]+0.07)
        self.marker_start_.points.append(point)

        return self.marker_start_

    def generate_search_markers(self, visited_set, unvisited_set):
        """Visualise the nodes with these node indices"""

        self.marker_visited_.points = []
        for i in visited_set:
            node_i = self.nodes_[i]
            p = self.map_.pixel_to_world(node_i.x, node_i.y)
            point = Point(x=p[0], y=p[1], z=p[2]+0.07)
            self.marker_visited_.points.append(point)

        self.marker_unvisited_.points = []
        for i in unvisited_set:
            node_i = self.nodes_[i]
            p = self.map_.pixel_to_world(node_i.x, node_i.y)
            point = Point(x=p[0], y=p[1], z=p[2]+0.08)
            self.marker_unvisited_.points.append(point)

        return self.marker_visited_, self.marker_unvisited_

class Node:
    def __init__(self, x, y, idx):

        # Index of the node in the graph
        self.idx = idx

        # Position of node
        self.x = x
        self.y = y

        # Neighbouring edges
        self.neighbours = []
        self.neighbour_costs = []

        # Search parameters
        self.cost_to_node = 999999999 # A large number
        self.cost_to_node_to_goal_heuristic = 999999999 # A large number
        self.parent_node = None # Invalid parent

    def distance_to(self, other_node):
        return math.sqrt((self.x-other_node.x)**2 + (self.y-other_node.y)**2)

    def is_connected(self, img, other_node):
        p1 = [self.x, self.y]
        p2 = [other_node.x, other_node.y]
        return not is_occluded(img, p1, p2)

def is_occluded(img, p1, p2, threshold=0.5):
    """
    Find if two points can be joined or if there's an obstacle in between
    Draws a line from p1 to p2
    Stops at the first pixel that is a "hit", i.e. above the threshold
    Returns the pixel coordinates for the first hit
    """

    # Extract the vector
    x1 = float(p1[0])
    y1 = float(p1[1])
    x2 = float(p2[0])
    y2 = float(p2[1])

    step = 1.0

    dx = x2 - x1
    dy = y2 - y1
    l = math.sqrt(dx**2. + dy**2.)
    if l == 0:
        return False
    dx = dx / l
    dy = dy / l

    max_steps = int(l / step)

    for i in range(max_steps):

        # Get the next pixel
        x = int(round(x1 + dx*i))
        y = int(round(y1 + dy*i))

        # Check if it's outside the image
        if x < 0 or x >= img.shape[0] or y < 0 or y >= img.shape[1]:
            return False

        # Check for "hit"
        if img[x, y] >= threshold:
            return True

    # No hits found
    return False
