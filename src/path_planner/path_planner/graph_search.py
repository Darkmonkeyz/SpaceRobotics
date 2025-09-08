import math


class GraphSearch:
    def __init__(self, parent_logger, graph, use_naive_planner, heuristic_weight, visualise_search_fn = None):
        self.parent_logger_ = parent_logger
        self.graph_ = graph
        self.visualise_search_ = visualise_search_fn

        self.use_naive_planner_ = use_naive_planner
        self.heuristic_weight_ = heuristic_weight

    def plan_path(self, start_idx, goal_idx):
        """Start path planning with specified method"""

        if self.use_naive_planner_:
            # Do naive planner
            self.path_ = self.naive_path_planner(start_idx, goal_idx)
        else:
            # Do A* planner
            self.astar_path_planner(start_idx, goal_idx)
            self.path_ = self.generate_path(goal_idx)
            
            
    def nodeToNodeDist(self, first_idx, last_idx):
        first = self.graph_.nodes_[first_idx]
        last = self.graph_.nodes_[last_idx]
        distFromGoal = math.sqrt(math.pow((first.x - last.x),2) + math.pow((first.y - last.y),2)) 
        return distFromGoal

    def naive_path_planner(self, start_idx, goal_idx):
        """Plan path with naive method"""
        goal = self.graph_.nodes_[goal_idx]
        path = []

        current = self.graph_.nodes_[start_idx]
        path.append(current)
        self.parent_logger_.info("Planning started")

        # Incrementally create the path
        while current.idx != goal_idx:


            ####################
            ## YOUR CODE HERE ##
            ## Task 2         ##
            ####################
            bestNeighbourDist = 99999999999
            bestNeighbour = None
            for neighbour in current.neighbours:
                
                count_visits = path.count(neighbour)
                
                if count_visits < 3:
                    distFromGoal = self.nodeToNodeDist(neighbour.idx, goal.idx) + 100 * count_visits
                    if distFromGoal < bestNeighbourDist:
                        bestNeighbourDist = distFromGoal
                        bestNeighbour = neighbour
            if bestNeighbour == None:
                break
            else:
                current = bestNeighbour
                path.append(current)
        
            
            
            
            












            
        self.parent_logger_.info('Goal found!')
        return path

    def astar_path_planner(self, start_idx, goal_idx):
        """Plan path with A*"""

        # Set all parents and costs to zero
        for n in self.graph_.nodes_:
            n.cost_to_node = 999999999 # a large number
            n.cost_to_node_to_goal_heuristic = 999999999 # a large number
            n.parent_node = None # invalid to begin with

        # Setup sets. These should contain indices (i.e. numbers) into the self.graph_.nodes_ array
        unvisited_set = []
        visited_set = []

        # Add start node to unvisited set
        unvisited_set.append(start_idx)
        self.graph_.nodes_[start_idx].cost_to_node = 0
        self.graph_.nodes_[start_idx].cost_to_node_to_goal_heuristic = 0

        # Loop until solution found or graph is disconnected
        while len(unvisited_set) > 0:

            
           
            ####################
            ## YOUR CODE HERE ##
            ## Task 3         ##
            ####################
            # Select a node
            # hint: self.get_minimum_cost_node(unvisited_set) will help you find the node with the minimum cost
            node_idx = unvisited_set[self.get_minimum_cost_node(unvisited_set)]  #wow garbage design smh. this function returns the index of the unvisited set with the lowest instead of the idx of the node
            #self.parent_logger_.info(f'node_idx {node_idx}...')
            #self.parent_logger_.info(f'unvisited {len(unvisited_set)}...')
            #for idx in range(len(unvisited_set)):
            #    self.parent_logger_.info(f'unvisited {idx}...')    
            #self.parent_logger_.info(f'start_node_idx {start_idx}...')
            #self.parent_logger_.info(f'unvisitedset {unvisited_set[0]}...')
            
            

            

            ####################
            ## YOUR CODE HERE ##
            ## Task 3         ##
            ####################
            # Move the node to the visited set
            visited_set.append(node_idx)
            unvisited_set.remove(node_idx)



            

            ####################
            ## YOUR CODE HERE ##
            ## Task 3         ##
            ####################
            # Termination criteria
            # Finish early (i.e. "return") if the goal is found
            if node_idx == goal_idx:
                self.parent_logger_.info('Goal found!')
                return




            # For each neighbour of the node
            for neighbour_idx in range(len(self.graph_.nodes_[node_idx].neighbours)):

                # For convenience, extract the neighbour and the edge cost from the arrays
                neighbour = self.graph_.nodes_[node_idx].neighbours[neighbour_idx]
                neighbour_edge_cost = self.graph_.nodes_[node_idx].neighbour_costs[neighbour_idx]

                # Check if neighbours is already in visited
                if neighbour.idx in visited_set:
                    
                    # Do nothing
                    pass
                
                else:

                    

                    ####################
                    ## YOUR CODE HERE ##
                    ## Task 3         ##
                    ####################
                    # Compute the cost of this neighbour node
                    # hint: cost_to_node = cost-of-previous-node + cost-of-edge 
                    # hint: cost_to_node_to_goal_heuristic = cost_to_node + self.heuristic_weight_ * A*-heuristic-score
                    # hint: neighbour.distance_to() function is likely to be helpful for the heuristic-score
                    heuristicAStar = self.nodeToNodeDist(neighbour.idx, goal_idx)
                    #self.parent_logger_.info(f'unvisited {heuristicAStar}...')
                    #self.graph_.nodes_[neighbour_idx].cost_to_node
                    newCostToNode = self.graph_.nodes_[node_idx].cost_to_node + neighbour_edge_cost
                    #self.graph_.nodes_[neighbour_idx].cost_to_node_to_goal_heuristic 
                    newCostToNodeHeuristic= newCostToNode + self.heuristic_weight_ * heuristicAStar

                    


                    # Check if neighbours is already in unvisited
                    if neighbour.idx in unvisited_set:

                        # If the cost is lower than the previous cost for this node
                        # Then update it to the new cost
                        # Also, update the parent pointer to point to the new parent 

                        ####################
                        ## YOUR CODE HERE ##
                        ## Task 3         ##
                        ####################
                        #pass # you can remove this line after you've completed the task
                        if newCostToNode < neighbour.cost_to_node:
                            neighbour.parent_node = node_idx
                            neighbour.cost_to_node = newCostToNode
                            neighbour.cost_to_node_to_goal_heuristic = newCostToNodeHeuristic




                    else:

                        # Add it to the unvisited set
                        unvisited_set.append(neighbour.idx)

                        # Initialise the cost and the parent pointer
                        # hint: this will be similar to your answer above

                        ####################
                        ## YOUR CODE HERE ##
                        ## Task 3         ##
                        ####################
                        neighbour.parent_node = node_idx
                        neighbour.cost_to_node = newCostToNode
                        neighbour.cost_to_node_to_goal_heuristic = newCostToNodeHeuristic




            # Visualise the current search status in RVIZ
            self.visualise_search_(visited_set, unvisited_set)
                   

    def get_minimum_cost_node(self, unvisited_set):
        """Find the vertex with the minimum cost"""

        # There's more efficient ways of doing this...
        min_cost = 99999999
        min_idx = None
        for idx in range(len(unvisited_set)):
            cost = self.graph_.nodes_[unvisited_set[idx]].cost_to_node_to_goal_heuristic
            if cost < min_cost:
                min_cost = cost
                min_idx = idx
        return min_idx

    def generate_path(self, goal_idx):
        """Generate the path by following the parents from the goal back to the start"""

        path = []

        current = self.graph_.nodes_[goal_idx]
        path.append(current)

        ####################
        ## YOUR CODE HERE ##
        ## Task 4         ##
        ####################
        for i in range(len(self.graph_.nodes_)):
            
            if current.parent_node != None:
                parent = self.graph_.nodes_[current.parent_node]
                path.append(parent)
                current = parent
            else:
                return path
            






        
        return path

    def find_connected_nodes(self, start_idx):
        """
        Return a list of all nodes that are reachable from start_idx node
        
        Hint 1 : this should be very similar to astar_path_planner(self, start_idx, goal_idx)
        Except there's no goal_idx, and it returns a list of all node indices that have a valid path from start_idx
        
        Hint 2: Can we use A* heuristic if there's no goal?
        """

        ####################
        ## YOUR CODE HERE ##
        ## Task 8         ##
        ####################

        # Set all parents and costs to zero
        for n in self.graph_.nodes_:
            n.cost_to_node = 9999999 # a large number
            n.cost_to_node_to_goal_heuristic = 999999999 # a large number
            n.parent_node = None # invalid to begin with

        # Setup sets
        unvisited_set = []
        visited_set = []

        # Add start node to visited set
        unvisited_set.append(start_idx)
        self.graph_.nodes_[start_idx].cost_to_node = 0
        self.graph_.nodes_[start_idx].cost_to_node_to_goal_heuristic = 0

        # Loop until solution found
        while len(unvisited_set) > 0:

            # Select a node
            # hint: self.get_minimum_cost_node(unvisited_set) will help you find the node with the minimum cost

            #########################
            ## YOUR CODE GOES HERE ##
            #########################



            # Move the node to the visited set




            # For each neighbour of the node
            for neighbour_idx in range(len(self.graph_.nodes_[node_idx].neighbours)):

                # For convenience, extract the neighbour and the edge cost from the arrays
                neighbour = self.graph_.nodes_[node_idx].neighbours[neighbour_idx]
                neighbour_cost = self.graph_.nodes_[node_idx].neighbour_costs[neighbour_idx]

                # Check if neighbours is already in visited
                if neighbour.idx in visited_set:
                    
                    # Do nothing
                    pass
                
                else:

                    # Compute the cost of this neighbour node
                    
                    ##########################
                    ## YOUR CODE GOES HERE  ##
                    ##########################
                    
                    


                    # Check if neighbours is already in unvisited
                    if neighbour.idx in unvisited_set:

                        pass # you can remove this line after you've completed the following

                        # If the cost is lower than the previous cost for this node
                        # Then update it to the new cost
                        # Also, update the parent pointer to point to the new parent 

                        ##########################
                        ## YOUR CODE GOES HERE  ##
                        ## FIX THE ?? BELOW     ##
                        ##########################
                        # if ??:
                        #     neighbour.parent_node = ??
                        #     neighbour.cost_to_node = ??

                        

                    else:

                        # Add it to the unvisited set
                        unvisited_set.append(neighbour.idx)

                        # Initialise the cost and the parent pointer
                        # hint: this will be similar to your answer above

                        ##########################
                        ## YOUR CODE GOES HERE  ##
                        ## FIX THE ?? BELOW     ##
                        ##########################
                        # neighbour.parent_node = ??
                        # neighbour.cost_to_node = ??

                        
        
        return visited_set