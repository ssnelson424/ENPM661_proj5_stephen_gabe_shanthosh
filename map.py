# Stephen Snelson Shanthosh Raaj Gabe Syzybalski
# ENPM661 - RO01 Spring 2026
# Project 5 - Competition with RRT
# Map

from math import cos,sin,radians, sqrt, ceil
import matplotlib.pyplot as plt

class Obstacle():
    """Obstacle for robot to avoid within the map"""
    def __init__(self,length:int,width:int,center_x:int,center_y:int,rotation:int, inflation:int):
        """initialize an obstacle object
        
        Args:
            length (int): obstacle size in the x-direction
            width (int): obstacle size in the y-direction
            center_x (int): location of the center of the obstacle in the x-axis
            center_y (int): location of the center of the obstacle in the y-axis
            rotation (int): rotation from "length" being along x-axis (counter-clockwise is +)
            inflation (int): cms to inflate the obstacle by to prevent robot itereference
        """
        self._length = length
        self._width  = width
        self._x = center_x
        self._y = center_y
        self._inflation = inflation
        
        if not -90 < rotation <90:
            raise ValueError("Rotation must be between -90 and 90 degrees")
        else:
            self._rot = radians(rotation)            
    
    
    def check_contains(self,point:tuple[float,float])->bool:
        """checks if a given x/y cord is within the obstacle

        Args:
            point (tuple): coordinate x/y

        Returns:
            bool: true - cord obstructed, false - cord not obstructed
        """
        x,y = point
        x = x - self._x
        y = y - self._y
        
        if self._rot != 0:
            
            local_x = x*cos(-self._rot) - y*sin(-self._rot)
            local_y = x*sin(-self._rot) + y*cos(-self._rot)
            x = local_x
            y = local_y
            
        if -.5*self._length - self._inflation <= x <= .5*self._length + self._inflation and \
                -.5*self._width - self._inflation <= y <= .5*self._width + self._inflation:

            return True
        
        return False         
    


class Map():
    def __init__(self,length:int,width:int,planing_inflation:int, movement_inflation:int):
        """initialize the map

        Args:
            length (int): length of the map in mm
            width (int): width/height of the map in mm
            inflation (int):size to increase each obstacle by to prevent robot collision

        Raises:
            ValueError: length:width ratio must be 2:1
        """
        self._length = length
        self._width = width
        self._obstacles = []
        self._obstacles_move = []
        self._start = (0,width/2,0)
        self._end = (length,0,0)
        self._goal_dist = 200
        
        if length/width != 2:
            raise ValueError("Map must be ratio 2:1")
        
        #create scale factor based on original length of 4000 mm
        self._scale_factor = length / 4000
        self._inflation_plan = planing_inflation
        self._inflation_move = movement_inflation
        
        self.set_obstacles(self._scale_factor,self._inflation_plan)
        self.set_obstacles_move(self._scale_factor,self._inflation_move)
        
    @property
    def start(self)->tuple[float,float,int]:
        return self._start
    
    @property
    def end(self)->tuple[float,float,int]:
        return self._end
    
    @property
    def dimensions(self)->tuple[int,int]:
        return (self._length,self._width)
    
    @property
    def goal_space(self)->int:
        return self._goal_dist
    
    
    def set_obstacles(self, scale_factor:float, inflation:int) -> None:
        """adds the obstacles to the map representation

        Args:
            scale_factor (float): adjustement scale factor from the given 4x2 map
            infaltion (int): increase to all obstacles to avoid robot contact
        """
        
        #create wall obstacles
        first_wall = Obstacle(1400*scale_factor,50*scale_factor,748.3*scale_factor,1381.3*scale_factor,-60,inflation)
        second_wall = Obstacle(1350*scale_factor,50*scale_factor,1611.3*scale_factor,597.1*scale_factor,60,inflation)
        third_wall = Obstacle(50*scale_factor,1450*scale_factor,3000*scale_factor,1275*scale_factor,0,inflation)
        
        #add walls to obstacle list
        self._obstacles.append(first_wall)
        self._obstacles.append(second_wall)
        self._obstacles.append(third_wall)
        
        #create box obstacles
        box_1 = Obstacle(304*scale_factor,304*scale_factor,420*scale_factor,450*scale_factor,0,inflation)
        box_2 = Obstacle(304*scale_factor,304*scale_factor,1335*scale_factor,1550*scale_factor,0,inflation)
        box_3 = Obstacle(304*scale_factor,304*scale_factor,2200*scale_factor,1740*scale_factor,0,inflation)
        
        #add box obstacles to obstacle list
        self._obstacles.append(box_1)
        self._obstacles.append(box_2)
        self._obstacles.append(box_3)
    
    def set_obstacles_move(self, scale_factor:float, inflation:int) -> None:
        """adds the obstacles to the map representation

        Args:
            scale_factor (float): adjustement scale factor from the given 4x2 map
            infaltion (int): increase to all obstacles to avoid robot contact
        """
        
        #create wall obstacles
        first_wall = Obstacle(1400*scale_factor,50*scale_factor,748.3*scale_factor,1381.3*scale_factor,-60,inflation)
        second_wall = Obstacle(1350*scale_factor,50*scale_factor,1611.3*scale_factor,597.1*scale_factor,60,inflation)
        third_wall = Obstacle(50*scale_factor,1450*scale_factor,3000*scale_factor,1275*scale_factor,0,inflation)
        
        #add walls to obstacle list
        self._obstacles_move.append(first_wall)
        self._obstacles_move.append(second_wall)
        self._obstacles_move.append(third_wall)
        
        #create box obstacles
        box_1 = Obstacle(304*scale_factor,304*scale_factor,420*scale_factor,450*scale_factor,0,inflation)
        box_2 = Obstacle(304*scale_factor,304*scale_factor,1335*scale_factor,1550*scale_factor,0,inflation)
        box_3 = Obstacle(304*scale_factor,304*scale_factor,2200*scale_factor,1740*scale_factor,0,inflation)
        
        #add box obstacles to obstacle list
        self._obstacles_move.append(box_1)
        self._obstacles_move.append(box_2)
        self._obstacles_move.append(box_3)
        
    def check_free_space(self,point:tuple[float,float]) -> bool:
        """Verifiyies if a point is within free space during the planning phase

        Args:
            point (tuple): coordinates to check
        
        Return
            bool: True - free space, false - obstructed space
        """
        x,y = point
        inflation = self._inflation_plan
        
        #check within map boundaries (y)
        if 0+inflation > y or y > self._width-inflation:
             return False
        
        #check within map boundaries (x) exclude entry gate
        if (0+inflation > x) and (750*self._scale_factor+inflation > y or self._width - 750*self._scale_factor-inflation < y):
            return False
        
        #iterate through obstacle list
        for obstacle in self._obstacles:
            if obstacle.check_contains(point):
                return False
        else:
            return True
    
    def check_edge_free(self, start:tuple[float,float],end:tuple[float,float]) -> bool:
        """checks if the start and end points are free, then samples the line between

        Args:
            start (tuple[float,float]): start coordinates
            end (tuple[float,float]): end coordinates

        Returns:
            bool: true - trajectory is free, false - trajectory is obstructed
        """
        x1,y1 = start
        x2,y2 = end  
        dx = (x2-x1)
        dy = (y2-y1)
        dist = sqrt((dx)**2 + (dy)**2)   
        
        resolution = 3 #resolution in mm for collision checking along edges
        
        #if start point in obstacle space, return false
        if not self.check_free_space(start):
            return False
        
        #if end point in obstacle space, return false
        if not self.check_free_space(end):
            return False
        
        #if total line distance is less than resolution and start/end are free, return clear
        if dist <= resolution:
            return True
                        
        # determine equation of line           
        step_count = ceil(dist/resolution)
        
        #loop through each step (start/end points already checked)
        for step in range(1,step_count):
            t = step/step_count
            x = x1 + t*dx
            y = y1 + t*dy
            
            if not self.check_free_space((x,y)):
                return False
            
        else:
            return True
    
    def check_free_space_move(self,point:tuple[float,float]) -> bool:
        """Verifiyies if a point is within free space during the move phase

        Args:
            point (tuple): coordinates to check
        
        Return
            bool: True - free space, false - obstructed space
        """
        x,y = point
        inflation = self._inflation_move
        
        #check within map boundaries (y)
        if 0+inflation > y or y > self._width-inflation:
             return False
        
        #check within map boundaries (x) exclude entry gate
        if (0+inflation > x) and (750*self._scale_factor+inflation > y or self._width - 750*self._scale_factor-inflation < y):
            return False
        
        #iterate through obstacle list
        for obstacle in self._obstacles_move:
            if obstacle.check_contains(point):
                return False
        else:
            return True
    
    
    def check_edge_free_move(self, start:tuple[float,float],end:tuple[float,float]) -> bool:
        """checks if the start and end points are free, then samples the line between

        Args:
            start (tuple[float,float]): start coordinates
            end (tuple[float,float]): end coordinates

        Returns:
            bool: true - trajectory is free, false - trajectory is obstructed
        """
        
        x1,y1 = start
        x2,y2 = end  
        dx = (x2-x1)
        dy = (y2-y1)
        dist = sqrt((dx)**2 + (dy)**2)   
        
        resolution = 3 #resolution in mm for collision checking along edges
        
        #if start point in obstacle space, return false
        if not self.check_free_space_move(start):
            return False
        
        #if end point in obstacle space, return false
        if not self.check_free_space_move(end):
            return False
        
        #if total line distance is less than resolution and start/end are free, return clear
        if dist <= resolution:
            return True
                        
        # determine qty of steps in the path segment          
        step_count = ceil(dist/resolution)
        
        #loop through each step (start/end points already checked)
        for step in range(1,step_count):
            t = step/step_count
            x = x1 + t*dx
            y = y1 + t*dy
            
            if not self.check_free_space_move((x,y)):
                return False
            
        else:
            return True        
        
    
    def plot_map(self, nodes=None, parents=None, path_1=None, path_2=None, path_3 = None):
        """plot 2D map fo debugging purposes"""
        #create storage lists for plotting
        obstacle_x = []
        obstacle_y = []
        plan_x = []
        plan_y = []
        move_x = []
        move_y = []
        goal_x = []
        goal_y = []
        
        #iterate through every x and y to create plot visual
        for x in range(0,self._length):
            for y in range (0,self._width):
                if x >= self._length-3:
                    goal_x.append(x)
                    goal_y.append(y)
                elif not self.check_free_space_move((x,y)):
                    move_x.append(x)
                    move_y.append(y)                
                elif not self.check_free_space((x,y)):
                    plan_x.append(x)
                    plan_y.append(y)

        #plot each set of x/y lists
        #plt.scatter(free_x, free_y, c="white", label = "Empty Space")
        plt.scatter(plan_x,plan_y, c ="grey", edgecolors="grey", label = "Plan Obstacles", s=2)
        plt.scatter(move_x,move_y, c ="black", edgecolors="black", label = "Move Obstacles", s=2)
        plt.scatter(goal_x,goal_y, c='green', label = "Goal Line",s=6)
        plt.scatter(0,self._width/2,c='blue', label = "Start Point",s=6)
        
        # plot RRT tree
        if nodes is not None and parents is not None:
            for node in nodes:
                if node in parents:
                    parent = parents[node]
                    plt.plot([parent[0], node[0]],[parent[1], node[1]],linewidth=0.5,alpha=0.5,color="cyan")

        # plot final path
        if path_1 is not None:
            path_1 = list(path_1)
            for i in range(len(path_1) - 1):
                p1 = path_1[i]
                p2 = path_1[i + 1]
                plt.plot([p1[0], p2[0]],[p1[1], p2[1]],linewidth=3,color="blue",label='Raw Path' if i == 0 else "")
                
        # plot smoothed path (thick, solid)
        if path_2 is not None:
            path_2 = list(path_2)
            for i in range(len(path_2) - 1):
                p1 = path_2[i]
                p2 = path_2[i + 1]
                plt.plot([p1[0], p2[0]], [p1[1], p2[1]], linewidth=3, color='red',label='Smoothed Path' if i == 0 else "")
                
        # plot filleted / curved path
        if path_3 is not None:
            path_3 = list(path_3)
            for i in range(len(path_3) - 1):
                p1 = path_3[i]
                p2 = path_3[i + 1]
                plt.plot([p1[0], p2[0]],[p1[1], p2[1]],linewidth=2,color="magenta",label="Filleted Path" if i == 0 else "")
        
        #plot labels
        plt.title("Stephen Gabe Shanthosh - Path Smoothing")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend(loc='upper left')
        
        #show plot
        print("Program will close upon closure of the Plot window.")
        plt.show()        
        
        
# if __name__ == "__main__":
#     game_board = Map(6000,3000,20)

#     game_board.plot_map()