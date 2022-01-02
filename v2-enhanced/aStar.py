import cv2
import numpy as np
from queue import PriorityQueue
import time
import math
import log
from scipy.spatial.distance import cdist


def checkGoal(coord1, coord2):
    rows = coord2[0]
    cols = coord2[1]
    
    if coord1[0] == rows or coord1[1] == 0 or coord1[1] == cols:
        return 0
    
    x = coord1[0] - rows/2
    y = abs(coord1[1] - cols/2)
    
    return cols/2 - max(x, y)
    

def manhattan(coord1, coord2, arr=False):
    """
        :param coord1: first coordinate, or single goal coordinate
        :param coord2: second coordinate, or array of coordinates
        :param arr: is coord2 an array?
    """
    if arr:
        deltas = np.abs(coord2 - np.array(coord1))
        dists = np.einsum('ij->i', deltas)
        return np.argmin(dists)

    return abs(coord2[0] - coord1[0]) + abs(coord2[1] - coord1[1])


def euclid(coord1, coord2, arr=False):
    """
        :param coord1: first coordinate, or single goal coordinate
        :param coord2: second coordinate, or array of coordinates
        :param arr: is coord2 an array?
    """
    if arr:
        deltas = coord2 - np.array(coord1)
        dist_2 = np.einsum('ij,ij->i', deltas, deltas)
        return np.argmin(dist_2)

    return np.sqrt((coord2[0] - coord1[0]) ** 2 + (coord2[1] - coord1[1]) ** 2)


def aStar(shell, unknowns, canWalk, goal, verbose=False, **args):
    """
        :param shell: shell of obstacles (true means is there)
        :param unknowns: unknown parts of obstacles (true means unknown)
        :param canWalk: field of view, or general places that can be walked (true means can walk)
        :param goal: (row, col) of goal point, relative to current position facing forwards
        :param verbose: print status?
        :param args: robotWidth (mm), step (mm), distFunc, goalFunc, voroFunc, voroWeight, vorMax, ignoreDia, diaWeight, start
        :return: onPath, path, closestNode, obsDist
    """

    rows, cols = shell.shape

    robotWidth = args["robotWidth"] if "robotWidth" in args else 200
    step = args["step"] if "step" in args else 50

    distFunc = args["distFunc"] if "distFunc" in args else manhattan
    goalFunc = checkGoal if goal is None else (args["goalFunc"] if "goalFunc" in args else manhattan)
    # voroFunc = args["voroFunc"] if "voroFunc" in args else manhattan

    voroWeight = args["voroWeight"] if "voroWeight" in args else 0.1
    voroMax = args["voroMax"] if "voroMax" in args else 400

    ignoreDia = args["ignoreDia"] if "ignoreDia" in args else False
    diaWeight = args["diaWeight"] if "diaWeight" in args else 50000

    startCoords = args["start"] if "start" in args else (int(shell.shape[0] / 2), 0)

    goalFar = False

    if goal is None:
        goal = (rows, cols)
        goalFar = True
    else:
        goal = (goal[0] + int(rows / 2), goal[1] + int(cols / 2))

    unwalkable = shell.astype(np.bool_) + unknowns.astype(np.bool_)

    kernel = np.ones((1 + int(math.ceil(robotWidth / step)), 1 + int(math.ceil(robotWidth / step))), np.uint8)
    obsNoDialate = unwalkable.astype(np.bool_)
    obstacles = cv2.dilate(unwalkable.astype(np.uint8), kernel, iterations=1).astype(np.bool_)
    canWalk = canWalk.astype(np.uint8)

    # cv2.imshow("obs", obstacles.astype(np.uint8)*255)
    # cv2.imshow("canWalk", canWalk.astype(np.uint8)*255)

    walkMap = np.clip(canWalk.astype(np.int8) - obsNoDialate.astype(np.int8), 0, None).astype(np.bool_)
    walkMapD = np.clip(canWalk.astype(np.int8) - obstacles.astype(np.int8), 0, None).astype(np.bool_)

    # cv2.imshow("walkmap", walkMap.astype(np.uint8)*255)

    nodeTo = np.zeros((rows, cols, 2), np.uint32) - 1
    distTo = np.zeros((rows, cols), np.float_) + np.inf
    obsDist = np.zeros((rows, cols), np.float_)
    visited = np.zeros((rows, cols), np.bool_)

    start = time.time()

    if verbose:
        log.log("voronoi start")

    contours, _ = cv2.findContours((walkMap if ignoreDia else walkMapD).astype(np.uint8)*255, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    unwalkCoords = []
    for contour in contours:
        for item in contour:
            unwalkCoords.append((item[0][1], item[0][0]))

    # unwalkCoords = np.array(tuple(zip(*np.where(walkMap == 0))))

    walkCoords = np.array(tuple(zip(*np.where((walkMap if ignoreDia else walkMapD) > 0))))
    
    dists = np.min(cdist(walkCoords, unwalkCoords), axis=1)
    for ind in range(walkCoords.shape[0]):
        obsDist[walkCoords[ind][0], walkCoords[ind][1]] = min(dists[ind], voroMax / step)

    # for row in range(rows):
    #     for col in range(cols):
    #         if not walkMap[row, col]:
    #             obsDist[row, col] = 0
    #             continue
    #         obsDist[row, col] = min(voroFunc((row, col), unwalkCoords, arr=True), voroMax / step)
    #         # obsDist[row, col] = min(voroFunc((row, col), unwalkCoords[index]), voroMax / step)

    obsMax = np.max(obsDist)

    q = PriorityQueue()
    q.put((0, startCoords))
    distTo[startCoords[0], startCoords[1]] = 0

    closestNode = (startCoords[0], startCoords[1])

    if verbose:
        end = time.time()
        log.log(end - start)
        start = time.time()
        log.log("a* start")

    while not q.empty():
        dist, coords = q.get()
        if visited[coords[0], coords[1]]:
            continue

        visited[coords[0], coords[1]] = 1

        if goalFunc(coords, goal) < goalFunc(closestNode, goal):
            closestNode = coords

        # if goalFunc(coords, (0,0)) > goalFunc(closestNode, (0,0)):
        #     closestNode = coords

        for row in range(coords[0] - 1, coords[0] + 2):
            for col in range(coords[1] - 1, coords[1] + 2):
                if row == coords[0] and col == coords[1]:
                    continue
                if 0 <= row < shell.shape[0] and 0 <= col < shell.shape[1] and walkMap[row, col]:
                    if not ignoreDia and not walkMapD[row, col]:
                        continue

                    tentDist = distTo[coords[0], coords[1]]*(1 if walkMapD[row, col] else diaWeight) + \
                        distFunc(coords, (row, col)) + \
                        obsMax - obsDist[row, col] * voroWeight

                    if distTo[row, col] > tentDist:
                        distTo[row, col] = tentDist
                        nodeTo[row, col] = coords
                        q.put((tentDist + goalFunc((row, col), goal), (row, col)))
                        visited[row, col] = 0

    onPath = np.zeros((rows, cols), np.bool_)
    path = []

    cur = closestNode

    if verbose:
        end = time.time()
        log.log(end - start)
        log.log("path start")

    while cur[0] != startCoords[0] or cur[1] != startCoords[1]:
        onPath[cur[0], cur[1]] = 1
        path.insert(0, cur)
        cur = nodeTo[cur[0], cur[1]]

    onPath[cur[0], cur[1]] = 1
    path.insert(0, cur)

    return onPath, path, closestNode, obsDist, (walkMap if ignoreDia else walkMapD)
