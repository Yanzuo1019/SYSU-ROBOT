import argparse
import logging

import numpy as np

from Map import *
from AStar import *
from RRT import *
from BiRRT import *

if __name__ == "__main__":
    logger = logging.getLogger()
    logging.basicConfig(
        level=logging.DEBUG, 
        datefmt="%Y-%m-%d %H:%M:%S",
        format='%(asctime)s %(filename)s[line:%(lineno)d] %(levelname)s: %(message)s'
    )

    parser = argparse.ArgumentParser()
    parser.add_argument("--map", type=str, default="map/map1.jpg", 
        help="path of map")
    parser.add_argument("--RRT", action="store_true", 
        help="apply RRT algorithm")
    parser.add_argument("--BiRRT", action="store_true",
        help="apply BiRRT algorithm")
    parser.add_argument("--init", type=str, default=None,
        help="tuple of init position")
    parser.add_argument("--goal", type=str, default=None,
        help="tuple of goal position")
    parser.add_argument("--sample_prob", type=float, default=0.5,
        help="random sample probability in RRT algorithm")
    parser.add_argument("--upper_limit", type=int, default=5000,
        help="maximum sample times")
    parser.add_argument("--pixel_step", type=int, default=10,
        help="pixel movement per sample step")
    args = parser.parse_args()

    if args.RRT:
        logger.info("start applying RRT algorithm")

        logger.info("constructing map...Wait!")
        if args.init is not None and args.goal is not None:
            init = tuple(map(int, args.init[1:-1].split(',')))
            goal = tuple(map(int, args.goal[1:-1].split(',')))
            mmap = Map(args.map, init, goal)
        else:
            mmap = Map(args.map)
        logger.info("MAP CONSTRUCT SUCCESS")
        
        logger.info("executing RRT algorithm...Wait!")
        pixel_step = args.pixel_step
        sample_prob = args.sample_prob
        upper_limit = args.upper_limit
        vertices, edges, mmap = RRT(mmap, pixel_step, sample_prob, upper_limit)
        if vertices is None:
            logger.info("RRT ALGORITHM FAIL")
        else:
            logger.info("RRT ALGORITHM SUCCESS")
            logger.info("executing AStar search...Wait!")
            mmap = AStar(vertices, edges, mmap)
            logger.info("ASTAR SEARCH SUCCESS")

        logger.info("saving map...Wait!")
        output_path = "RRT_pixelstep_{}_sampleprob_{}_upperlimit_{}.jpg".format(pixel_step, sample_prob, upper_limit)
        mmap.save_fig(output_path)
        logger.info("RRT algorithm has finished. Map has already saved in {}".format(output_path))

    if args.BiRRT:
        logger.info("start applying BiRRT algorithm")

        logger.info("constructing map...Wait!")
        if args.init is not None and args.goal is not None:
            init = tuple(map(int, args.init[1:-1].split(',')))
            goal = tuple(map(int, args.goal[1:-1].split(',')))
            mmap = Map(args.map, init, goal)
        else:
            mmap = Map(args.map)
        logger.info("MAP CONSTRUCT SUCCESS")
        
        logger.info("executing BiRRT algorithm...Wait!")
        pixel_step = args.pixel_step
        upper_limit = args.upper_limit
        vertices, edges, mmap = BiRRT(mmap, pixel_step, upper_limit)
        if vertices is None:
            logger.info("BiRRT ALGORITHM FAIL")
        else:
            logger.info("BIRRT ALGORITHM SUCCESS")
            logger.info("executing AStar search...Wait!")
            mmap = AStar(vertices, edges, mmap)
            logger.info("ASTAR SEARCH SUCCESS")

        logger.info("saving map...Wait!")
        output_path = "BiRRT_pixelstep_{}_upperlimit_{}.jpg".format(pixel_step, upper_limit)
        mmap.save_fig(output_path)
        logger.info("BiRRT algorithm has finished with map already saved in {}".format(output_path))