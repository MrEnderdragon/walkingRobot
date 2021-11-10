import cv2
import numpy as np
from queue import PriorityQueue
import time


def manhattan(coord1, coord2):
    return abs(coord2[0]-coord1[0]) + abs(coord2[1]-coord1[1])


def euclid(coord1, coord2):
    return np.sqrt((coord2[0]-coord1[0])**2 + (coord2[1]-coord1[1])**2)


def aStar(shell, unknowns, canWalk, goal, verbose=False, **args):
    """
        :param shell: shell of obstacles (true means is there)
        :param unknowns: unknown parts of obstacles (true means unknown)
        :param canWalk: field of view, or general places that can be walked (true means can walk)
        :param goal: (row, col) of goal point, relative to current position facing forwards
        :param verbose: print status?
        :param args: 'robotWidth (mm), step (mm), distFunc, goalFunc, voroFunc, voroWeight'
        :return: onPath, path, closestNode
    """

    rows, cols = shell.shape

    robotWidth = args["robotWidth"] if "robotWidth" in args else 112.6 + 100
    step = args["step"] if "step" in args else 50

    distFunc = args["distFunc"] if "distFunc" in args else manhattan
    goalFunc = args["goalFunc"] if "goalFunc" in args else manhattan
    voroFunc = args["voroFunc"] if "voroFunc" in args else manhattan

    voroWeight = args["voroWeight"] if "voroWeight" in args else 0.1

    goal = (goal[0]+int(rows/2), goal[1])

    unwalkable = shell.astype(np.bool_) + unknowns.astype(np.bool_)

    kernel = np.ones((1 + int(robotWidth / step), 1 + int(robotWidth / step)), np.uint8)
    obstacles = cv2.dilate(unwalkable.astype(np.uint8), kernel, iterations=1).astype(np.bool_)
    canWalk = cv2.dilate(canWalk.astype(np.uint8), kernel, iterations=1)

    walkMap = canWalk - obstacles

    nodeTo = np.zeros((rows, cols, 2), np.uint32)-1
    distTo = np.zeros((rows, cols), np.float_)+np.inf
    obsDist = np.zeros((rows, cols), np.float_)
    visited = np.zeros((rows, cols), np.bool_)

    start = time.time()

    if verbose:
        print("voronoi start")

    unwalkCoords = tuple(zip(*np.where(walkMap == 0)))

    for row in range(rows):
        for col in range(cols):
            # obsDist[row, col] = voroFunc(min(unwalkCoords, key=lambda co: voroFunc(co, (row, col))), (row, col))
            index = np.argmin(np.sum(np.abs(np.array(unwalkCoords) - np.array([row, col])), axis=1))
            obsDist[row, col] = voroFunc(unwalkCoords[index], (row, col))

    obsMax = np.max(obsDist)

    q = PriorityQueue()
    q.put((0, (int(shell.shape[0]/2), 0)))
    distTo[int(shell.shape[0]/2), 0] = 0

    closestNode = (0, 0)

    if verbose:
        print("a* start")
        end = time.time()
        print(end-start)

    while not q.empty():
        dist, coords = q.get()
        if visited[coords[0], coords[1]]:
            continue

        visited[coords[0], coords[1]] = 1

        if goalFunc(coords, goal) < goalFunc(closestNode, goal):
            closestNode = coords

        for row in range(coords[0] - 1, coords[0] + 2):
            for col in range(coords[1] - 1, coords[1] + 2):
                if 0 <= row < shell.shape[0] and 0 <= col < shell.shape[1] and walkMap[row, col]:

                    tentDist = distTo[coords[0], coords[1]] + \
                               distFunc(coords, (row, col)) + \
                               obsMax - obsDist[row, col]*voroWeight

                    if distTo[row, col] > tentDist:
                        distTo[row, col] = tentDist
                        nodeTo[row, col] = coords
                        q.put((tentDist + goalFunc((row, col), goal), (row, col)))
                        visited[row, col] = 0

    onPath = np.zeros((rows, cols), np.bool_)
    path = []

    cur = closestNode
    negOne = np.iinfo(np.uint32).max

    while cur[0] != negOne and cur[1] != negOne:
        onPath[cur[0], cur[1]] = 1
        path.insert(0, cur)
        cur = nodeTo[cur[0], cur[1]]

    return onPath, path, closestNode