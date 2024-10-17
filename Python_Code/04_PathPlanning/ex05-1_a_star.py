import numpy as np
import math
import matplotlib.pyplot as plt
import random
from map_2 import map
import time

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
                
# Define possible actions
# action = [dx, dy, cost]
def get_action():
    action_set = [[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1], [1, 1, np.sqrt(2)], [1, -1, np.sqrt(2)], [-1, 1, np.sqrt(2)], [-1, -1, np.sqrt(2)]]
    return action_set

# Collision check function
def collision_check(omap, node):
    if node.position in zip(omap[0], omap[1]):
        return True
    return False

# Dijkstra algorithm implementation
def dijkstra(start, goal, map_obstacle):
    start_node = Node(None, start)
    goal_node = Node(None, goal)
    
    open_list = []
    closed_list = []
    weight = 2

    flag = False

    open_list.append(start_node)
    while open_list:
        # Find node with lowest cost
        current_node = min(open_list, key=lambda o: o.f)
        open_list.remove(current_node)
        closed_list.append(current_node)
        if flag and len(closed_list)>1000:
            break
        # If goal is reached, return the optimal path
        if current_node == goal_node:
            flag = True 
            path = [[],[]]
            current = current_node
            while current is not None:
                path[0].append(current.position[0])
                path[1].append(current.position[1])
                current = current.parent
            plt.plot(path[0], path[1], ".r", markersize=12)
            
        # Generate child candidates
        action_set = get_action()
        for action in action_set:
            node_position = (current_node.position[0] + action[0], current_node.position[1] + action[1])
            
            # Create new node
            new_node = Node(current_node, node_position)
            
            # If collision expected, skip this node
            if collision_check(map_obstacle, new_node):
                continue
            
            # If node is already in closed list, skip it
            if new_node in closed_list:
                continue
            
            # Set cost for the node
            new_node.g = current_node.g + action[2]
            new_node.h = np.sqrt((current_node.position[0]-goal_node.position[0])**2 + (current_node.position[1]-goal_node.position[1])**2)
            new_node.f = new_node.g + new_node.h#*weight
            # If node is already in open list with a higher cost, skip it
            if new_node in open_list:
                existing_node = open_list[open_list.index(new_node)]
                if existing_node.f > new_node.f:
                    open_list.remove(existing_node)
                    open_list.append(new_node)
            else:
                open_list.append(new_node)
        
        # Show graph
        if show_animation:
            plt.plot(current_node.position[0], current_node.position[1], 'yo', alpha=0.5)
            if len(closed_list) % 30 == 0:
                plt.pause(0.03)

# Main function
def main():
    start, goal, omap = map()

    if show_animation:
        plt.figure(figsize=(8, 8))
        plt.plot(start[0], start[1], 'bs', markersize=7)
        plt.text(start[0], start[1] + 0.5, 'start', fontsize=12)
        plt.plot(goal[0], goal[1], 'rs', markersize=7)
        plt.text(goal[0], goal[1] + 0.5, 'goal', fontsize=12)
        plt.plot(omap[0], omap[1], '.k', markersize=10)
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("X [m]"), plt.ylabel("Y [m]")
        plt.title("A Star algorithm", fontsize=20)

    opt_path = dijkstra(start, goal, omap)
    print("Optimal path found!")
    

    plt.show()

if __name__ == "__main__":
    main()