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
    
    game_board = Map(6000,3000,110)
    turtlebot = Robot()
    #rrt = RRT(game_board,turtlebot)
    rrt = RRTStarAPEI(game_board,turtlebot)
    
    #output analytics from RRT class for debugging + measurements
    rrt.analysis = True
    
    path_to_goal = rrt.solve()
    
    for i in range(len(path_to_goal) - 1):
        if not game_board.check_edge_free(path_to_goal[i], path_to_goal[i + 1]):
            print("Bad edge:", path_to_goal[i], path_to_goal[i + 1])
    
    end_time = time.perf_counter()
    
    print(f"Computation complete. Elapsed Time:{end_time-start_time:2f}")
    
    game_board.plot_map(nodes=rrt.nodes,parents=rrt.parents,path_1=rrt._raw_path,path_2=path_to_goal)     
    

if __name__ == "__main__":
    main()