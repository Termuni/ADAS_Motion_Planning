import numpy as np
import math
import matplotlib.pyplot as plt
import random
from map_1 import map
# from map_2 import map

from ex04_Dijkstra import get_action, collision_check

show_animation  = True

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.f = 0
        self.g = 0
        self.h = 0

    def __eq__(self, other):
        if self.position == other.position:
            return True

def calc_Euc_Dist(target_pos, ego_pos):
    return np.sqrt((target_pos[0]-ego_pos[0])**2 + (target_pos[1]+ego_pos[1])**2)

def a_star(start, goal, map_obstacle):
    eyps = 1.0
    
    #노드 선언
    start_node = Node(None, start)
    goal_node = Node(None, goal)
    
    goal_pos = np.array([goal_node.position[0], goal_node.position[1]])
    
    open_list = []
    closed_list = []
    
    open_list.append(start_node)
    #open list 뭐 있으면 계속 가기
    
    while open_list is not None:
        cur_node = open_list[0]
        cur_idx = 0
        
        # Find node with lowest cost + euclidian length
        for index, node in enumerate(open_list):
            if node.f < cur_node.f:
                cur_node = node
                cur_idx = index
                
        # If goal, return optimal path -> 골이면 멈추고 끝내기
        if cur_node == goal_node:
            optimal_path = []
            node = cur_node
            while node is not None:
                optimal_path.append(node.position)
                node = node.parent
            return optimal_path
            
        # If not goal, move from open list to closed list
        open_list.pop(cur_idx)
        closed_list.append(cur_node)
        
        # Generate child candidate
        action_set = get_action()
        
        for action in action_set:
            childNode_Expected_Pos = (cur_node.position[0] + action[0], cur_node.position[1] + action[1])
            # If collision expected, do nothing
            if collision_check(map_obstacle, childNode_Expected_Pos):
                continue
            # If not collision, create child node
            childNode = Node(cur_node, childNode_Expected_Pos)
            
            # If already in closed list, do nothing
            if childNode in closed_list:
                continue
            
            # If not in closed list, update open list
            # childNode.f = cur_node.f + action[2]
            childNode.g = cur_node.g + action[2]
            # childNode.h = calc_Euc_Dist(goal_pos, node.position)
            childNode.h = np.sqrt((cur_node.position[0]-goal_node.position[0])**2 + (cur_node.position[1]-goal_node.position[1])**2)
            childNode.f = childNode.g + childNode.h * eyps
            
            #   Update Cost if it is in open list
            if childNode in open_list:
                idx = open_list.index(childNode)
                if childNode.f < open_list[idx].f:
                    open_list[idx] = childNode
                    # open_list[idx].parent = childNode.parent
                    # open_list[idx].f = childNode.f
            else:
                open_list.append(childNode)
            
        # show graph
        if show_animation:
            plt.plot(cur_node.position[0], cur_node.position[1], 'yo', alpha=0.5)
            if len(closed_list) % 100 == 0:
                plt.pause(0.03)

def main():

    start, goal, omap = map()

    if show_animation == True:
        plt.figure(figsize=(8,8))
        plt.plot(start[0], start[1], 'bs',  markersize=7)
        plt.text(start[0], start[1]+0.5, 'start', fontsize=12)
        plt.plot(goal[0], goal[1], 'rs',  markersize=7)
        plt.text(goal[0], goal[1]+0.5, 'goal', fontsize=12)
        plt.plot(omap[0], omap[1], '.k',  markersize=10)
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("X [m]"), plt.ylabel("Y [m]")
        plt.title("a_star algorithm", fontsize=20)

    opt_path = a_star(start, goal, omap)
    print("Optimal path found!")
    opt_path = np.array(opt_path)
    if show_animation == True:
        plt.plot(opt_path[:,0], opt_path[:,1], "m.-")
        plt.show()

if __name__ == "__main__":
    main()
