import numpy as np

from Map import *
from AStar import cal_dist
from RRT import get_nearest

PIXEL_STEP = None
UPPER_LIMIT = None

def combine(vertices_init, edges_init, vertices_goal, edges_goal):
    for v in vertices_goal:
        if v not in vertices_init: vertices_init.append(v)
    for e in edges_goal:
        if e not in edges_init: 
            edges_init[e] = edges_goal[e].copy()
        else:
            for edge in edges_goal[e]:
                if edge not in edges_init[e]: edges_init[e].append(edge)
    return vertices_init, edges_init

def BiRRT(mmap: Map, pixel_step, upper_limit):
    PIXEL_STEP = pixel_step
    UPPER_LIMIT = upper_limit

    init_node = mmap.init
    vertices_init = [init_node]
    edges_init = {init_node: []}
    mmap.draw_dot(init_node, color=(255,0,0))

    goal_node = mmap.goal
    vertices_goal = [goal_node]
    edges_goal = {goal_node: []}
    mmap.draw_dot(goal_node, color=(0,255,0))

    sample_count = 0
    now_graph = "init"
    while True:
        samplex = np.random.randint(0, mmap.mat.shape[0])
        sampley = np.random.randint(0, mmap.mat.shape[1])
        sample = (samplex, sampley)

        if now_graph == "init":
            dist, pt_nearest = get_nearest(vertices_init, sample)
            if dist > PIXEL_STEP:
                x = round(pt_nearest[0] + (sample[0] - pt_nearest[0]) / dist * PIXEL_STEP)
                y = round(pt_nearest[1] + (sample[1] - pt_nearest[1]) / dist * PIXEL_STEP)
                sample = (x, y)
            sample_count += 1

            if mmap.check_collision(pt_nearest, sample):
                vertices_init.append(sample)
                if pt_nearest not in edges_init: edges_init[pt_nearest] = [sample]
                else: edges_init[pt_nearest].append(sample)
                if sample not in edges_init: edges_init[sample] = [pt_nearest]
                else: edges_init[sample].append(pt_nearest)

                mmap.draw_dot(sample, color=(255,0,0))
                mmap.draw_line(pt_nearest, sample, color=(255,0,0))

                new_sample = sample
                dist, pt_nearest = get_nearest(vertices_goal, new_sample)
                if dist > PIXEL_STEP:
                    x = round(pt_nearest[0] + (new_sample[0] - pt_nearest[0]) / dist * PIXEL_STEP)
                    y = round(pt_nearest[1] + (new_sample[1] - pt_nearest[1]) / dist * PIXEL_STEP)
                    new_sample = (x, y)
                
                if mmap.check_collision(pt_nearest, new_sample):
                    vertices_goal.append(new_sample)
                    if pt_nearest not in edges_goal: edges_goal[pt_nearest] = [new_sample]
                    else: edges_goal[pt_nearest].append(new_sample)
                    if new_sample not in edges_goal: edges_goal[new_sample] = [pt_nearest]
                    else: edges_goal[new_sample].append(pt_nearest)

                    mmap.draw_dot(new_sample, color=(0,255,0))
                    mmap.draw_line(pt_nearest, new_sample, color=(0,255,0))

                    while True:
                        pt_nearest = new_sample
                        new_sample = sample
                        dist = cal_dist(pt_nearest, new_sample)
                        if dist > PIXEL_STEP:
                            x = round(pt_nearest[0] + (new_sample[0] - pt_nearest[0]) / dist * PIXEL_STEP)
                            y = round(pt_nearest[1] + (new_sample[1] - pt_nearest[1]) / dist * PIXEL_STEP)
                            new_sample = (x, y)
                        if mmap.check_collision(pt_nearest, new_sample):
                            vertices_goal.append(new_sample)
                            if pt_nearest not in edges_goal: edges_goal[pt_nearest] = [new_sample]
                            else: edges_goal[pt_nearest].append(new_sample)
                            if new_sample not in edges_goal: edges_goal[new_sample] = [pt_nearest]
                            else: edges_goal[new_sample].append(pt_nearest)

                            mmap.draw_dot(new_sample, color=(0,255,0))
                            mmap.draw_line(pt_nearest, new_sample, color=(0,255,0))
                            if dist <= PIXEL_STEP:
                                vertices, edge = combine(vertices_init, edges_init, vertices_goal, edges_goal)
                                return vertices, edge, mmap
                        else: break
            
            if len(vertices_goal) < len(vertices_init):
                now_graph = "goal"

        elif now_graph == "goal":
            dist, pt_nearest = get_nearest(vertices_goal, sample)
            if dist > PIXEL_STEP:
                x = round(pt_nearest[0] + (sample[0] - pt_nearest[0]) / dist * PIXEL_STEP)
                y = round(pt_nearest[1] + (sample[1] - pt_nearest[1]) / dist * PIXEL_STEP)
                sample = (x, y)
            sample_count += 1

            if mmap.check_collision(pt_nearest, sample):
                vertices_goal.append(sample)
                if pt_nearest not in edges_goal: edges_goal[pt_nearest] = [sample]
                else: edges_goal[pt_nearest].append(sample)
                if sample not in edges_goal: edges_goal[sample] = [pt_nearest]
                else: edges_goal[sample].append(pt_nearest)

                mmap.draw_dot(sample, color=(0,255,0))
                mmap.draw_line(pt_nearest, sample, color=(0,255,0))

                new_sample = sample
                dist, pt_nearest = get_nearest(vertices_init, new_sample)
                if dist > PIXEL_STEP:
                    x = round(pt_nearest[0] + (new_sample[0] - pt_nearest[0]) / dist * PIXEL_STEP)
                    y = round(pt_nearest[1] + (new_sample[1] - pt_nearest[1]) / dist * PIXEL_STEP)
                    new_sample = (x, y)
                
                if mmap.check_collision(pt_nearest, new_sample):
                    vertices_init.append(new_sample)
                    if pt_nearest not in edges_init: edges_init[pt_nearest] = [new_sample]
                    else: edges_init[pt_nearest].append(new_sample)
                    if new_sample not in edges_init: edges_init[new_sample] = [pt_nearest]
                    else: edges_init[new_sample].append(pt_nearest)

                    mmap.draw_dot(new_sample, color=(255,0,0))
                    mmap.draw_line(pt_nearest, new_sample, color=(255,0,0))

                    while True:
                        pt_nearest = new_sample
                        new_sample = sample
                        dist = cal_dist(pt_nearest, new_sample)
                        if dist > PIXEL_STEP:
                            x = round(pt_nearest[0] + (new_sample[0] - pt_nearest[0]) / dist * PIXEL_STEP)
                            y = round(pt_nearest[1] + (new_sample[1] - pt_nearest[1]) / dist * PIXEL_STEP)
                            new_sample = (x, y)
                        if mmap.check_collision(pt_nearest, new_sample):
                            vertices_init.append(new_sample)
                            if pt_nearest not in edges_init: edges_init[pt_nearest] = [new_sample]
                            else: edges_init[pt_nearest].append(new_sample)
                            if new_sample not in edges_init: edges_init[new_sample] = [pt_nearest]
                            else: edges_init[new_sample].append(pt_nearest)

                            mmap.draw_dot(new_sample, color=(255,0,0))
                            mmap.draw_line(pt_nearest, new_sample, color=(255,0,0))
                            if dist <= PIXEL_STEP:
                                vertices, edge = combine(vertices_init, edges_init, vertices_goal, edges_goal)
                                return vertices, edge, mmap
                        else: break

            if len(vertices_init) < len(vertices_goal):
                now_graph = "init"

        if sample_count >= UPPER_LIMIT:
            return None, None, mmap
