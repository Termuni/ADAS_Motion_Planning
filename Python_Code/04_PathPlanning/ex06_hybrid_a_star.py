import numpy as np
import math
import matplotlib.pyplot as plt
import random
from map_3 import map

show_animation  = True

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.heading = 0.0
        self.f = 0
        self.g = 0
        self.h = 0
        
    def __eq__(self, other):
        if self.position == other.position:
            return True

# Check if position of node is same( if distance < threshold, regard as same node)
def isSamePosition(node_1, node_2, epsilon_position=0.3):
    dist = heuristic(node_1, node_2)
    ans = False
    if(dist < epsilon_position):
        ans = True
    return ans

def isSameYaw(node_1, node_2, epsilon_yaw=0.2):
    ans = False
    if(np.abs(node_1.heading-node_2.heading) < epsilon_yaw):
        ans = True
    return ans

# Action set, Moving only forward direction              
def get_action(R,Vx,delta_time_step):
    yaw_rate = Vx/R
    distance_travel = Vx*delta_time_step
    # yaw_rate, delta_time_step, cost
    action_set = [[yaw_rate, delta_time_step, distance_travel], 
                  [-yaw_rate, delta_time_step, distance_travel],
                  [yaw_rate/2, delta_time_step, distance_travel],
                  [-yaw_rate/2, delta_time_step, distance_travel],
                  [0.0, delta_time_step, distance_travel]]
    return action_set

# Vehicle movement
def vehicle_move(position_parent, yaw_rate, delta_time, Vx):
    x_parent = position_parent[0]
    y_parent = position_parent[1]
    yaw_parent = position_parent[2]
    
    # if yaw_rate != 0 (left or right turn)
    if (yaw_rate != 0):
        R_signed = Vx / yaw_rate
        delta_yaw = yaw_rate * delta_time
        x_child = x_parent + R_signed * (np.sin(yaw_parent + delta_yaw) - np.sin(yaw_parent))
        y_child = y_parent + R_signed * (np.cos(yaw_parent) - np.cos(yaw_parent + delta_yaw))
        yaw_child = yaw_parent + delta_yaw
    # move straight
    else:
        x_child = x_parent + Vx * delta_time * np.cos(yaw_parent)
        y_child = y_parent + Vx * delta_time * np.sin(yaw_parent)
        yaw_child = yaw_parent
        
    # yaw processing (ensure yaw is between 0 and 2*pi)
    if yaw_child > 2 * np.pi:
        yaw_child -= 2 * np.pi
    if yaw_child < 0:
        yaw_child += 2 * np.pi

    # return position : [x, y, yaw]
    return [x_child, y_child, yaw_child]


# Collision check : path overlaps with any of obstacle
def collision_check_2(position_child, obstacle_list):
    for obstacle in obstacle_list:
        if(np.power(obstacle[2], 2) >= (np.power(position_child[0] - obstacle[0], 2) + np.power(position_child[1] - obstacle[1], 2))):
            return True
    return False
    
def collision_check(position_parent, yaw_rate, delta_time_step, obstacle_list, Vx):
    # obstacle_list = [[3, 5, 1], ... , [15, 5, 1]]
    vehicle_pos = vehicle_move(position_parent, yaw_rate, delta_time_step, Vx)
    
    for obstacle in obstacle_list:
        if(np.power(obstacle[2], 2) >= (np.power(vehicle_pos[0] - obstacle[0], 2) + np.power(vehicle_pos[1] - obstacle[1], 2))):
            return True
    
    return False

# Check if the node is in the searching space
def isNotInSearchingSpace(position_child, space):
    # space = [-2.0, 15.0, -2.0, 15.0]  
    if ((position_child[0] < space[0]) or (position_child[0] > space[1]) or (position_child[1] < space[2]) or (position_child[1] > space[3])):
        return True
    return False
        
def heuristic(cur_node, goal_node):
    dist = np.sqrt((cur_node.position[0] - goal_node.position[0])**2 + (cur_node.position[1]  - goal_node.position[1])**2)
    return dist

def a_star(start, goal, space, obstacle_list, R, Vx, delta_time_step, weight):
    eyps = weight
    
    start_node = Node(None, start)
    goal_node = Node(None, goal)
    
    open_list = []
    closed_list = []
    
    open_list.append(start_node)
    
    while open_list is not None:
        cur_node = open_list[0]
        cur_idx = 0
        
        # Find node with lowest cost
        for index, node in enumerate(open_list):
            if node.f < cur_node.f:
                cur_node = node
                cur_idx = index
                
        # If goal, return optimal path
        if isSamePosition(cur_node, goal_node, epsilon_position=0.6) and isSameYaw(cur_node, goal_node):
            optimal_path = []
            node = cur_node
            while node is not None:
                optimal_path.append(node.position)
                node = node.parent
            return optimal_path
        
        # If not goal, move from open list to closed list
        open_list.pop(cur_idx)
        closed_list.append(cur_node)
        if len(closed_list) > 1000:
            temp_list = closed_list[500:]
            closed_list.clear()
            closed_list = temp_list
        
        # Generate child candidate
        action_set = get_action(R, Vx, delta_time_step)
        for action in action_set:
            childNode_Expected_Pos = vehicle_move(cur_node.position, action[0], action[1], Vx)
            # If not in searching space, do nothing
            # If not in searching space, do nothing
            if isNotInSearchingSpace(childNode_Expected_Pos, space):
                continue

            # If collision expected, do nothing
            if collision_check(childNode_Expected_Pos, Vx/R, delta_time_step, obstacle_list, Vx):
                continue

            # If not collision, create child node
            childNode = Node(cur_node, childNode_Expected_Pos)

            # If already in closed list, do nothing
            if childNode in closed_list:
                continue

            # If not in closed list, update open list
            childNode.g = cur_node.g + action[2]
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
                plt.pause(0.1)
                
                

def main():

    start, goal, obstacle_list, space = map()

    if show_animation == True:
        theta_plot = np.linspace(0,1,101) * np.pi * 2
        plt.figure(figsize=(8,8))
        plt.plot(start[0], start[1], 'bs',  markersize=7)
        plt.text(start[0], start[1]+0.5, 'start', fontsize=12)
        plt.plot(goal[0], goal[1], 'rs',  markersize=7)
        plt.text(goal[0], goal[1]+0.5, 'goal', fontsize=12)
        for i in range(len(obstacle_list)):
            x_obstacle = obstacle_list[i][0] + obstacle_list[i][2] * np.cos(theta_plot)
            y_obstacle = obstacle_list[i][1] + obstacle_list[i][2] * np.sin(theta_plot)
            plt.plot(x_obstacle, y_obstacle,'k-')
        plt.axis(space)
        plt.grid(True)
        plt.xlabel("X [m]"), plt.ylabel("Y [m]")
        plt.title("Hybrid a star algorithm", fontsize=20)

    opt_path = a_star(start, goal, space, obstacle_list, R=5.0, Vx=2.0, delta_time_step=0.5, weight=1.1)
    print("Optimal path found!")
    opt_path = np.array(opt_path)
    if show_animation == True:
        plt.plot(opt_path[:,0], opt_path[:,1], "m.-")
        plt.show()


if __name__ == "__main__":
    main()

    

