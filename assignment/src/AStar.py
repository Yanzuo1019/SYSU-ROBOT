import queue

import numpy as np

from Map import *

# 计算两点间的距离
def cal_dist(pt1, pt2):
    dist = ((pt2[0] - pt1[0]) ** 2 + (pt2[1] - pt1[1]) ** 2) ** 0.5
    return dist

# A*算法搜索节点
class Node(object):
    def __init__(self, hist, goal):
        super().__init__()

        self.hist = hist.copy()
        self.goal = goal
        self.pt = self.hist[-1]
        self.gx = len(self.hist) - 1
        self.hx = cal_dist(self.hist[-1], self.goal)
        self.fx = self.gx + self.hx
    
    def __lt__(self, other):
        return self.fx < other.fx

# A*搜索算法
def AStar(vertices: list, edges: dict, mmap: Map):
    init_vert, goal_vert = vertices[0], vertices[-1]

    init_node = Node([init_vert], goal_vert)
    pq = queue.PriorityQueue()
    pq.put(init_node)

    while not pq.empty():
        node = pq.get()

        if mmap.is_goal(node.pt):
            for i in range(len(node.hist)-1):
                mmap.draw_dot(node.hist[i], color=(0,0,255), thickness=2)
                mmap.draw_line(node.hist[i], node.hist[i+1], color=(0,0,255), thickness=4)
            mmap.draw_dot(node.pt, color=(0,0,255))
            return mmap
        
        for v in edges[node.pt]:
            if v in node.hist: continue

            new_hist = node.hist.copy()
            new_hist.append(v)
            new_node = Node(new_hist, node.goal)
            pq.put(new_node)
    
    return mmap