# Stephen Snelson Shanthosh Raaj Gabe Syzybalski
# ENPM661 - RO01 Spring 2026
# Project 5 - Competition with RRT
# Alogrithm

from map import Map
from robot import Robot
from trajectory import Trajectory
from random import randint,uniform,expovariate
from math import sqrt,floor,ceil,log2,atan2,cos,sin, pi, radians
from collections import deque
from abc import ABC, abstractmethod

class AlgorithmBase(ABC):
    def __init__(self,map:Map,robot:Robot):
        self._map = map
        self._robot = robot
        self._nodes = [] # list[(x,y),(x,y),etc]
        self._parent = {} # (x,y):(parent_x),(parent_y)
        self._visited = set()
        self._path_to_goal = deque()
        self._raw_path = deque()
        self._path_length = float
        self._resolution = 20 #mm
        self._raw_path_dist = 0
        self._smooth_path_dist = 0
        self._give_analysis = False
        self._cost = {self.get_start_waypoint():0}
        self._test_radius = self._resolution*5
        self._node_buckets = {}
        self._bucket_size = self._test_radius
    
    @property
    def nodes(self) -> list[tuple[float,float]]:
        return self._nodes
    
    @property
    def parents(self) -> dict[tuple[float,float]]:
        return self._parent
    
    @property
    def analysis(self) -> bool:
        return self._give_analysis
    
    @analysis.setter
    def analysis(self,input:bool) -> None:
        self._give_analysis = input
    
    @abstractmethod
    def solve(self) -> deque[tuple[float,float]]:
        """solves the provided map through RRT/RRT*/Improved RRT*"""
        pass    

    def reset(self)->None:
        """resets path and internal data structures to prevent mixing"""
        
        self._nodes.clear()
        self._parent.clear()
        self._visited.clear()
        self._raw_path.clear()
        self._path_to_goal.clear()
        self._raw_path_dist = 0
        self._smooth_path_dist = 0
        self._node_buckets.clear()
        self._cost.clear()
        self._cost = {self.get_start_waypoint():0}
        
    def get_distance(self,first:tuple[float,float],second:tuple[float,float])->float:
        """gets the distance between two coordinates

        Args:
            first (tuple[float,float]): first coordinates (x,y)
            second (tuple[float,float]): second coordinates (x,y)

        Returns:
            float: distance between coords in mm
        """
        x1,y1 = first
        x2,y2 = second
        dx = x2-x1
        dy = y2-y1        
        return sqrt(dx**2 + dy**2)
        
    def bucketize(self,coords:tuple[float,float])->tuple[float,float]:
        """returns the very localized bucketized coords for duplicity checking - not for use during RRT*/I-RRT* near-node finding

        Args:
            coords (tuple(float,float)): coordinates to be bucketized

        Returns:
            tuple[float,floats]: (bucketized_x,bucketized_y)
        """
        euclidean_dist = 2 #2mm buckets
        
        x,y = coords
        x_bucket = int(floor(x/euclidean_dist+ .5))
        y_bucket = int(floor(y/euclidean_dist+ .5))
        
        return (x_bucket,y_bucket)
        
    def steer(self,start:tuple[float,float],end:tuple[float,float]) -> tuple[float,float]:
        """returns a node in the direction of the sample point one resolution distance away

        Args:
            start (tuple[float,float]): nearest node
            end (tuple[float,float]): sampled point

        Returns:
            tuple[float,float]: step point
        """         
        x1,y1 = start
        x2,y2 = end  
        dx = (x2-x1)
        dy = (y2-y1)
        dist = self.get_distance(start,end)
        
        #if total line distance is less than resolution and start/end are free, return clear
        if dist <= self._resolution:
            return end        
    
        t = self._resolution/dist
        x = x1 + t*dx
        y = y1 + t*dy            
            
        return (x,y)  
    
    def smooth(self,path:deque[tuple[float,float]])->deque[tuple[float,float]]:
        """smooth the raw path output by the RRT planner by removed unnecessary nodes. Uses concept of visibility around obstacles

        Args:
            path (deque[tuple[float,float]]): raw_ path from RRT planner

        Returns:
            deque[tuple[float,float]]: smoothed path
        """
        
        if len(path) <= 3:
            return path.copy()
        
        work_path = path.copy()
        smooth_path = deque()
        
        #grab start point and add it to path
        anchor_node = work_path.popleft()
        
        #grab next two nodes (check node is the node between anchor node and next node)
        check_node = work_path.popleft()
        next_node = work_path.popleft()        
        
        #iterate while working path is populated        
        while work_path:
            # if a straight line can be drawn from anchor node to next node, remove check node and 
            # get the next node ("next" becomes the check)
            if self._map.check_edge_free(anchor_node,next_node):
                check_node = next_node
                next_node = work_path.popleft()
                continue
            
            # if a line cannot be drawn, the check node is crucial. anchor added to smooth path, check becomes anchor,
            # next become check, and a new next node gets pulled from raw path queue
            else:
                smooth_path.append(anchor_node)
                anchor_node = check_node
                check_node = next_node
                next_node = work_path.popleft()
        
        #add the last two nodes to the list so they are not dropped
        smooth_path.append(anchor_node)
        smooth_path.append(check_node)
        smooth_path.append(next_node)
        
        x_start,y_start,ori = self._map.start   
        smooth_path.appendleft((x_start,y_start))
        
        return smooth_path
    
    def get_start_waypoint(self) -> tuple[float,float]:
        """Starts the robot 300 mm directly infront of start point to help with trajectory smoothing
        
        returns: tuple offset infront of the original node"""
        
        x,y,ori = self._map.start
        offset = 300
        
        x1 = x+offset*cos(radians(ori))
        y1 = y+offset*sin(radians(ori))
        
        return (x1,y1)        
        
    
    def give_analytics(self,iterations:int):
        """Provides analytics for path smoothing and total sample iterations

        Args:
            iterations (int): # of test nodes sampled
        """
        
        print(f"Required Iterations: {iterations}")
        print(f"Raw Path Nodes: {len(self._raw_path)}")
        print(f"Smooth Path Nodes: {len(self._path_to_goal)}")

        raw_dist_path = self._raw_path.copy()
        
        x1,y1=raw_dist_path.pop()
        
        while raw_dist_path:
            x2,y2=raw_dist_path.pop()
            dist = self.get_distance((x1,y1),(x2,y2))
            self._raw_path_dist += dist
            x1,y1 = x2,y2
        
        print(f"Raw Path Distance: {self._raw_path_dist}")
        
        smooth_dist_path = self._path_to_goal.copy()
        
        x1,y1=smooth_dist_path.pop()
        while smooth_dist_path:
            x2,y2=smooth_dist_path.pop()
            dist = self.get_distance((x1,y1),(x2,y2))
            self._smooth_path_dist += dist
            x1,y1 = x2,y2
        
        print(f"Smooth Path Distance: {self._smooth_path_dist}")
        
    def find_best_parent(self,active_node:tuple[float,float],local_nodes:list[tuple[float,float]])->tuple[tuple[float,float],float]:
        """finds the best parent node based on cost. Uses very-large buckets to allow localized node sampling. 
            Not for use in duplicity checking

        Args:
            active_node (tuple[float,float]): coordinates which require parents
            local_nodes list(tuple(float,float)):all nodes in the local bucket

        Returns:
            tuple[tuple[float,float],float]: ((parent_cords x,y),cost)
        """
        
        best_cost = None
        best_parent = None        
        
        #iterate through nodes in set radius to find (lowest cost + distance)
        for node in local_nodes:
            cost = self._cost[node]+self.get_distance(node,active_node)
            
            #if better cost found update parent,cost
            if (best_cost == None or best_cost >= cost) and self._map.check_edge_free(node,active_node):
                best_parent = node
                best_cost = cost            

        return (best_parent,best_cost)
    
    def get_near_node_bucket(self, active_node:tuple[float,float])->tuple[int,int]:
        """returns the large bucket to collect nodes in location. Not for use during duplicity checking.

        Args:
            active_node (tuple[float,float]): coordinates to be bucketed (x,y)

        Returns:
            tuple[int,int]: bucketized coords (x,y)
        """
        x,y = active_node
        return (int(floor(x/self._test_radius)),int(floor(y/self._test_radius)))
    
    def add_node_to_bucket(self, active_node:tuple[float,float])-> None:
        """adds node to large bucket for faster local node sampling. Not for use during duplicity checking

        Args:
            node (tuple[float,float]): node to be added to bucket (x,y)
            
        """
        #determine which bucket node fits into
        bucket = self.get_near_node_bucket(active_node)
        
        #if bucket doesnt exist in dictionary, add bucket to dictionary
        if bucket not in self._node_buckets:
            self._node_buckets[bucket]=[]

        #add node to bucket
        self._node_buckets[bucket].append(active_node)
    
    def get_near_nodes(self,active_node:tuple[float,float]) -> list[tuple[float,float]]:
        """provides nodes from all local large buckets for local node/distance/cost comparison. not for use in duplicity checking.

        Args:
            active_node (tuple[float,float]): coordinates being evaluated

        Returns:
            list[tuple[float,float]]: list of coordinate nodes in nearby bucket
        """
        x_bucket,y_bucket = self.get_near_node_bucket(active_node)
        near_nodes = []
        
        search_range = ceil(self._test_radius/self._bucket_size)
        
        for dx in range(-search_range,search_range+1):
            for dy in range(-search_range,search_range+1):
                bucket = (x_bucket+dx,y_bucket+dy)
                
                if bucket not in self._node_buckets:
                    continue
                
                for node in self._node_buckets[bucket]:
                    if self.get_distance(node,active_node) <= self._test_radius:
                        near_nodes.append(node)
        return near_nodes
    
    def rewire(self,active_node:tuple[float,float],local_nodes:tuple[float,float])->None:
        """Adds rewiring to verify no better path has been found

        Args:
            active_node (tuple[float,float]): node being processed
            local_nodes (tuple[float,float]]): list of local nodes to check rewiring from
        """
        
        #iterate through each local node    
        for node in local_nodes:
            if node == active_node:
                continue
            
            #can a straight line be drawn to the active node unobstructed
            if not self._map.check_edge_free(active_node,node):
                continue
            
            #calculate cost through the active node
            new_cost = self._cost[active_node] + self.get_distance(active_node,node)
            
            #if cost is less, make active node the parent and update cost
            if new_cost < self._cost[node]:
                self._parent[node] = active_node
                self._cost[node] = new_cost
        

class RRT(AlgorithmBase):
    """Map Planner utilizing RRT (not RRT_star or Improved_RRT)

    Args:
        AlgorithmBase (_type_): Subclass off Alrogithm Base
    """
    
    def __init__(self,map:Map,robot:Robot):
        super().__init__(map,robot)
        
            
    def solve(self) -> deque[tuple[float,float]]:
        """generate a path to goal to solve the maze using base RRT"""       
        
        #clear previous data in case of multiple iterations
        self.reset()
        
        #sampling parameters
        max_x,max_y = self._map.dimensions
        sample_x = max_x + self._map.goal_space
        goal_found = False
        iterator = 0
        
        start = self.get_start_waypoint()
        
        #add start node to node lsit nd visited set
        self._nodes.append(start)
        self._visited.add(self.bucketize(start))
        
        #iterate until a path to the final point has been found or 10000 iterations
        while iterator <= 10000 and not goal_found:
            iterator += 1
            
            #create new 
            x = randint(0,sample_x)
            y = randint(0,max_y)
            sampling_node = (x,y)
            
            #initial checks: if already visited, if not in free space
            if self.bucketize(sampling_node) in self._visited:
                continue                       

            
            #find nearest node to sample node            
            distance = None
            nearest_node = None           
            
            for node in self._nodes:
                dist = self.get_distance(sampling_node,node)                
                
                if distance == None or dist <= distance:                    
                    #update nearest node and distance to nearest node
                    nearest_node = node
                    distance = dist            
            
            active_node = self.steer(nearest_node,sampling_node)
                        
            #is active node in free space
            if not self._map.check_free_space(active_node):
                continue
            
            #checks if edge from active node to nearest is clear of obstacles
            if self._map.check_edge_free(nearest_node,active_node):           
                
                if self.bucketize(active_node) in self._visited:
                        continue                   
                    
                    
                self._visited.add(self.bucketize(active_node))
                self._nodes.append(active_node)
                self._parent[active_node] = nearest_node       
                    
                #if node has clear path to goal zone, move directly there
                if self._map.check_edge_free(active_node,(max_x+300,active_node[1])):
                    #goal_found
                    goal_found = True
                    final_node = (max_x,active_node[1])                    
                    self._parent[final_node] = active_node
                    
                    #add final 2 nodes to path
                    self._raw_path.appendleft(final_node)
                    break     
        
        while final_node != self.get_start_waypoint() and goal_found:
            
            self._raw_path.appendleft(self._parent[final_node])       
            final_node = self._parent[final_node]
        
        if not goal_found:
            raise RuntimeError(f"Goal Not Found after {iterator} iterations")
        
        print(f"Solution Found")
        
        #smooth path to avoid unnecessary turns
        self._path_to_goal = self.smooth(self._raw_path)

        #Analytics for debugging and anaylsis purposes
        if self._give_analysis:
            self.give_analytics(iterator)
        
        return self._path_to_goal
    
class RRTStar(AlgorithmBase):
    def __init__(self,map:Map,robot:Robot):
        super().__init__(map,robot)

    def solve(self) -> deque[tuple[float,float]]:
        """generate a path to goal to solve the maze using RRT*"""       
        
        #clear previous data in case of multiple iterations
        self.reset()
        
        #sampling parameters
        max_x,max_y = self._map.dimensions
        sample_x = max_x + self._map.goal_space
        goal_found = False
        iterator = 0
        
        #add start node to node lsit nd visited set
        start = self.get_start_waypoint()
        self._nodes.append(start)
        self._visited.add(self.bucketize(start))
        self.add_node_to_bucket(start)
        
        #iterate until a path to the final point has been found or 10000 iterations
        while iterator <= 10000 and not goal_found:
            iterator += 1
            
            #create new 
            x = randint(0,sample_x)
            y = randint(0,max_y)
            sampling_node = (x,y)
            
            #initial checks: if already visited, if not in free space
            if self.bucketize(sampling_node) in self._visited:
                continue                      

            #find nearest node to sample node            
            distance = None
            nearest_node = None           
            
            for node in self._nodes:
                dist = self.get_distance(sampling_node,node)                
                
                if distance == None or dist <= distance:                    
                    #update nearest node and distance to nearest node
                    nearest_node = node
                    distance = dist            
            
            active_node = self.steer(nearest_node,sampling_node)
                        
            #is active node in free space
            if not self._map.check_free_space(active_node):
                continue
            
            local_nodes = self.get_near_nodes(active_node)
            
            best_parent, best_cost = self.find_best_parent(active_node,local_nodes)
            
            if best_parent == None or best_cost == None:
                continue
            
            #checks if edge from active node to nearest is clear of obstacles
            if self._map.check_edge_free(best_parent,active_node):           
                
                #if node has already been evaluated/visited
                if self.bucketize(active_node) in self._visited:
                        continue                  
                
                #add node to visited lists
                self._visited.add(self.bucketize(active_node))
                self._nodes.append(active_node)
                self.add_node_to_bucket(active_node)
                
                #record parent nodes and cost
                self._parent[active_node] = best_parent
                self._cost[active_node] = best_cost                
                
                #check local nodes to see if rewiring is possible
                self.rewire(active_node,local_nodes)
                    
                #if node has clear path to goal zone, move directly there
                if self._map.check_edge_free(active_node,(max_x+300,active_node[1])):
                    #goal_found
                    goal_found = True
                    final_node = (max_x,active_node[1])                    
                    self._parent[final_node] = active_node
                    
                    #add final 2 nodes to path
                    self._raw_path.appendleft(final_node)
                    break     
        
        while final_node != self.get_start_waypoint() and goal_found:
            
            self._raw_path.appendleft(self._parent[final_node])       
            final_node = self._parent[final_node]
        
        if not goal_found:
            raise RuntimeError(f"Goal Not Found after {iterator} iterations")
        
        print(f"Solution Found")
        
        #smooth path to avoid unnecessary turns
        self._path_to_goal = self.smooth(self._raw_path)

        #Analytics for debugging and anaylsis purposes
        if self._give_analysis:
            self.give_analytics(iterator)
        
        return self._path_to_goal    

class RRTStarAPEI(AlgorithmBase):
    """Planner Classing Utilizing APEI-RRT* from subject paper"""
    # Paper: Optimizing Initial Path Finding in Informed-RRT* with a Novel Map-Adaptice Sampling Technique"""
    
    def __init__(self,map:Map,robot:Robot):
        super().__init__(map,robot)
        self._miss_ratio = 0.5 # xi from paper, between 0-1
        self._miss_ratio_delta = 0.2 #omega from paper between 0.1-0.3
        self._sample_spread = 2.0  #set based on environemtnal context
        self._eccentricity = 0.7 #e from paper
        self._best_cost = None
        self._best_goal_node = None       

    def solve(self) -> deque[tuple[float,float]]:
        """generate a path to goal to solve the maze using APE I-RRT*"""
        #APEI - Adaptive Probibalistic Ellipsoid Informed
        
        #clear previous data in case of multiple iterations
        self.reset()
        
        #sampling parameters
        max_x,max_y = self._map.dimensions
        sample_x = max_x + self._map.goal_space
        goal_found = False
        iterator = 0
        
        #add start node to node lsit nd visited set
        start = self.get_start_waypoint()
        self._nodes.append(start)
        self._visited.add(self.bucketize(start))
        self.add_node_to_bucket(start)
        
        #iterate until a path to the final point has been found or 10000 iterations
        while iterator <= 30000 and not goal_found:
            iterator += 1
        
            sampling_node = self.sample_rrt_apei(goal_found)
            
            if not self._map.check_free_space(sampling_node):
                self.update_miss_ratio(obstructed = True)
                continue
            else:
                self.update_miss_ratio(obstructed=False)
            
            #initial checks: if already visited, if not in free space
            if self.bucketize(sampling_node) in self._visited:
                continue                      

            #find nearest node to sample node            
            distance = None
            nearest_node = None           
            
            for node in self._nodes:
                dist = self.get_distance(sampling_node,node)                
                
                if distance == None or dist <= distance:                    
                    #update nearest node and distance to nearest node
                    nearest_node = node
                    distance = dist            
            
            active_node = self.steer(nearest_node,sampling_node)
                        
            #is active node in free space
            if not self._map.check_free_space(active_node):
                continue
            
            local_nodes = self.get_near_nodes(active_node)
            
            best_parent, best_cost = self.find_best_parent(active_node,local_nodes)
            
            if best_parent == None or best_cost == None:
                continue
            
            #checks if edge from active node to nearest is clear of obstacles
            if self._map.check_edge_free(best_parent,active_node):           
                
                #if node has already been evaluated/visited
                if self.bucketize(active_node) in self._visited:
                        continue                  
                
                #add node to visited lists
                self._visited.add(self.bucketize(active_node))
                self._nodes.append(active_node)
                self.add_node_to_bucket(active_node)
                
                #record parent nodes and cost
                self._parent[active_node] = best_parent
                self._cost[active_node] = best_cost                
                
                #check local nodes to see if rewiring is possible
                self.rewire(active_node,local_nodes)
                    
                #if node has clear path to goal zone, move directly there
                if self._map.check_edge_free(active_node,(max_x+300,active_node[1])):                
                    
                    #goal_found
                    goal_found = True
                    final_node = (max_x,active_node[1])                 
                    candidate_cost = self._cost[active_node] + self.get_distance(active_node,final_node)
                    
                    if self._best_cost is None or candidate_cost < self._best_cost:                            
                        self._parent[final_node] = active_node
                        self._best_goal_node = final_node
                        self._best_cost = candidate_cost
                
                    #add final nodes to path
                    self._raw_path.appendleft(self._best_goal_node)  
                    final_node = self._best_goal_node
        
        while final_node != self.get_start_waypoint() and goal_found:
            
            self._raw_path.appendleft(self._parent[final_node])       
            final_node = self._parent[final_node]
        
        if not goal_found:
            raise RuntimeError(f"Goal Not Found after {iterator} iterations")
        
        print(f"Solution Found")
        
        #smooth path to avoid unnecessary turns
        self._path_to_goal = self.smooth(self._raw_path)

        #Analytics for debugging and anaylsis purposes
        if self._give_analysis:
            self.give_analytics(iterator)
            
        print(self._path_to_goal)
        
        return self._path_to_goal
    
    def sample_rrt_apei(self,goal_found:bool)->tuple[float,float]:
        """provides a sample node to the algorithm to evaluate

        Args:
            goal_found (bool): is initial path being found (goal_found:Flase), or imporved (goal_found:true)

        Returns:
            tuple[float,float]: (x,y) coordinates
        """        
        #goal not found sampling in normal APEI
        #taken from Alogirthm 1 of paper
        if not goal_found:
            start = self.get_start_waypoint()
            max_x,max_y = self._map.dimensions
            goal = (max_x,max_y/2)
            
            for _ in range(10):  # try up to 10 times
                theta = uniform(0,2*pi)
                
                rate = max((1-self._miss_ratio)*self._sample_spread,1e-6)
                gamma = expovariate(rate)
                
                a = self.get_distance(start,goal)/2
                b = a * sqrt(1-self._eccentricity**2)
                
                x_sampled = log2(2+gamma)*a*cos(theta)
                y_sampled = gamma * b*sin(theta)
                
                angle = atan2(goal[1] - start[1],goal[0]-start[0])
                
                rotated_x = x_sampled*cos(angle) - y_sampled*sin(angle)
                rotated_y = x_sampled*sin(angle) + y_sampled*cos(angle)
                
                midpoint = ((start[0]+goal[0])/2,(start[1]+goal[1])/2)
                
                sample = (rotated_x + midpoint[0], rotated_y + midpoint[1])
                
                if self._map.check_free_space(sample) and sample[0]>=0:            
                    return sample

            # fallback if all attempts fail
            return (randint(0,max_x), randint(0,max_y))                   
            
        #goal found sampling inside I-RRT* ellipse until best cost found
        else:
            start = self.get_start_waypoint()         
            goal = self._best_goal_node

            cost_min = self.get_distance(start,goal)
            cost_best = self._best_cost
            
            if cost_best is None or cost_best <= cost_min:
                return goal
        
            tmax_x, max_y = self._map.dimensions

            for _ in range(10):  # try up to 10 times
                theta = uniform(0,2*pi)
                
                rate = max((1-self._miss_ratio)*self._sample_spread,1e-6)
                gamma = expovariate(rate)
                
                a = self.get_distance(start,goal)/2
                b = a * sqrt(1-self._eccentricity**2)
                
                x_sampled = log2(2+gamma)*a*cos(theta)
                y_sampled = gamma * b*sin(theta)
                
                angle = atan2(goal[1] - start[1],goal[0]-start[0])
                
                rotated_x = x_sampled*cos(angle) - y_sampled*sin(angle)
                rotated_y = x_sampled*sin(angle) + y_sampled*cos(angle)
                
                midpoint = ((start[0]+goal[0])/2,(start[1]+goal[1])/2)
                
                sample = (rotated_x + midpoint[0], rotated_y + midpoint[1])
                
                if self._map.check_free_space(sample) and sample[0]>=0:            
                    return sample

            # fallback if all attempts fail
            return (randint(0,max_x), randint(0,max_y))
            

    def update_miss_ratio(self,obstructed:bool)->None:
        """update miss ratio per paper equation 4

        Args:
            obstructed (bool): _description_
        """
        #if the sample point is obstructed, ratio decreases
        if obstructed:
            self._miss_ratio = self._miss_ratio_delta + (1-self._miss_ratio_delta)*self._miss_ratio
        
        #if sample point is clear, ratio increases
        else:
            self._miss_ratio = (1-self._miss_ratio_delta)*self._miss_ratio
            
    def compute_rotation(self):
        """Builds the rotation of the ellipse x-axis along path_to_goal direction"""
        
    def transform_sample(self,local_point:tuple[float,float]):
        """Transforms the sample in to the ellipse rotation"""
        
    
    