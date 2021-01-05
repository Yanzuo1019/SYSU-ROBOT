import numpy as np

from Map import *
from AStar import cal_dist

PIXEL_STEP = None
SAMPLE_PROB = None
UPPER_LIMIT = None

def get_nearest(vertices: list, pt: tuple):
    min_dist = np.inf
    min_v = None
    for v in vertices:
        dist = cal_dist(v, pt)
        if dist < min_dist: min_dist, min_v = dist, v
    return min_dist, min_v

def RRT(mmap: Map, pixel_step, sample_prob, upper_limit):
    PIXEL_STEP = pixel_step
    SAMPLE_PROB = sample_prob
    UPPER_LIMIT = upper_limit

    init_node = mmap.init
    vertices = [init_node]
    edges = {init_node: []}
    mmap.draw_dot(init_node)

    sample_count = 0
    while True:
        if np.random.random() < SAMPLE_PROB:
            samplex = np.random.randint(0, mmap.mat.shape[0])
            sampley = np.random.randint(0, mmap.mat.shape[1])
            sample = (samplex, sampley)
        else:
            sample = mmap.goal
        
        dist, pt_nearest = get_nearest(vertices, sample)
        if dist > PIXEL_STEP:
            x = round(pt_nearest[0] + (sample[0] - pt_nearest[0]) / dist * PIXEL_STEP)
            y = round(pt_nearest[1] + (sample[1] - pt_nearest[1]) / dist * PIXEL_STEP)
            sample = (x, y)
        sample_count += 1

        if mmap.check_collision(pt_nearest, sample):
            vertices.append(sample)
            if pt_nearest not in edges: edges[pt_nearest] = [sample] 
            else: edges[pt_nearest].append(sample)
            if sample not in edges: edges[sample] = [pt_nearest]
            else: edges[sample].append(pt_nearest)

            mmap.draw_dot(sample)
            mmap.draw_line(pt_nearest, sample)
        
        if mmap.is_goal(sample):
            return vertices, edges, mmap
        
        if sample_count >= UPPER_LIMIT:
            return None, None, mmap