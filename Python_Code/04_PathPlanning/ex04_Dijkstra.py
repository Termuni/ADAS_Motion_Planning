import numpy as np
import math
import matplotlib.pyplot as plt
import random
from map_1 import map

show_animation  = True

class Node:
    '''
    parent -> 이 친구는 이전 노드
    '''
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.f = 0

    def __eq__(self, other):
        if self.position == other.position:
            return True
                
def get_action():
    # 어떤 agent가 움직일 수 있는 방향 제시
    # 상하좌우 + 대각선 4방향
    # action = [dx, dy, cost]
    # action_set = [action1, action2, ...]
    action_set = [[0,-1,1], [0,1,1], [-1,0,1], [1,0,1], 
                  [1,-1,np.sqrt(2)], [1,1,np.sqrt(2)], [-1,1,np.sqrt(2)], [-1,-1,np.sqrt(2)]]
    return action_set

def collision_check(omap, node): #맵과 현재 노드를 받음, 현재 노드의 x,y와 omap의 x,y와 같은지 판단
    # 나중에는 내 경로에 포함 되는지 확인
    # 가는 노드가 장애물 노드인지 아닌지 확인하는 방식으로 진행
    # grid에 장애물 이라고 써있으면 안 가면 됨
    # Check if node position == obstacle position
    nx = node[0]
    ny = node[1]
    ox = omap[0]
    oy = omap[1]
    col = False
    for i in range(len(ox)):
        if (nx == ox[i]) and (ny == oy[i]):
            col = True
            break
    return col # True or False

def dijkstra(start, goal, map_obstacle):
    '''
    start에서 goal까지 가는 것이 목표
    open list -> 안 가본, 조사 해야 하는 노드 선언
    closed list -> 가본 노드 저장
    '''
    #노드 선언
    start_node = Node(None, start)
    goal_node = Node(None, goal)
    
    
    open_list = []
    closed_list = []
    
    open_list.append(start_node)
    #open list 뭐 있으면 계속 가기
    while open_list is not None:
        cur_node = open_list[0]
        cur_idx = 0
        
        # Find node with lowest cost
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
            # plt.plot(cur_node.position[0], cur_node.position[1], 'yo', alpha=0.5)
            # if len(closed_list) % 100 == 0:
            #     plt.pause(0.03)
            return optimal_path
            
        # If not goal, move from open list to closed list
        open_list.pop(cur_idx)
        closed_list.append(cur_node)
        print(len(closed_list))
        if len(closed_list) > 1000:
            temp_list = closed_list[500:]
            closed_list.clear()
            closed_list = temp_list
            
        
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
            childNode.f = cur_node.f + action[2]
            
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
        plt.title("Dijkstra algorithm", fontsize=20)

    opt_path = dijkstra(start, goal, omap)
    print("Optimal path found!")
    opt_path = np.array(opt_path)
    if show_animation == True:
        plt.plot(opt_path[:,0], opt_path[:,1], "m.-")
        plt.show()


if __name__ == "__main__":
    main()

    

