# Stephen Snelson Shanthosh Raaj Gabe Syzybalski
# ENPM661 - RO01 Spring 2026
# Project 5 - Competition with RRT
# Main/Run File

from algorithm import AlgorithmBase,RRT,RRTStar,RRTStarAPEI
from robot import Robot
from map import Map
from collections import deque
import time


def main():
    
    start_time = time.perf_counter()
    
    game_board = Map(8000,4000,270,110)
    turtlebot = Robot(map = game_board, rpm1=60,rpm2=20)
    #rrt = RRT(game_board,turtlebot)
    rrt = RRTStar(game_board,turtlebot)
    
    #output analytics from RRT class for debugging + measurements
    rrt.analysis = True
    
    path_to_goal = rrt.solve()
        
    comp_time=time.perf_counter()
        
    print(f"Computation complete. Elapsed Time:{(comp_time-start_time):.2f}")
    formatted_path = [(f"{x:.2f}",f"{y:.2f}")for x,y, in path_to_goal]
    print(f"path: len: {len(path_to_goal)}, path: {formatted_path}")
    # print(f"Plotting")
    
    game_board.plot_map(nodes=rrt.nodes,parents=rrt.parents,path_1=rrt._raw_path,path_2=path_to_goal)
    
    plot_time=time.perf_counter()
    
    turtlebot.follow_waypoints(path_to_goal)    

    end_time = time.perf_counter()
    
    #prints time excluding the time to plot
    print(f"Simulation Complete. Elapsed Time:{(end_time-start_time-(plot_time-comp_time)):.2f}")
    
    
    

if __name__ == "__main__":
    main()
