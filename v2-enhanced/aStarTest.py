import cv2
import numpy as np
import aStar
import matplotlib.pyplot as plt
from queue import PriorityQueue
import time

robotWidth = 112.6 + 100  # mm
step = 50  # mm


def manhattan(coord1, coord2):
    return abs(coord2[0]-coord1[0]) + abs(coord2[1]-coord1[1])


def euclid(coord1, coord2):
    return np.sqrt((coord2[0]-coord1[0])**2 + (coord2[1]-coord1[1])**2)


if __name__ == '__main__':
    for i in range(0, 10):
        # Constructing test image
        imgId = "SPLR-" + str(i) + ".png"
        shell = cv2.imread("flatShell/shell" + imgId, cv2.IMREAD_UNCHANGED)
        unknowns = cv2.imread("flatUnknown/unknown" + imgId, cv2.IMREAD_UNCHANGED)
        canWalk = cv2.imread("flatCanWalk/canWalk" + imgId, cv2.IMREAD_UNCHANGED)
        imgRef = cv2.imread("RImages/R" + imgId, cv2.IMREAD_UNCHANGED)

        walkable = shell.astype(np.bool_)+unknowns.astype(np.bool_)

        kernel = np.ones((1+int(robotWidth/step), 1+int(robotWidth/step)), np.uint8)
        obstacles = cv2.dilate(walkable.astype(np.uint8), kernel, iterations=1).astype(np.bool_)

        canWalk = cv2.dilate(canWalk.astype(np.uint8), kernel, iterations=1)

        walkMap = canWalk-obstacles

        nodeTo = np.zeros((shell.shape[0], shell.shape[1], 2), np.uint32)-1
        distTo = np.zeros((shell.shape[0], shell.shape[1]), np.float_)+np.inf
        obsDist = np.zeros((shell.shape[0], shell.shape[1]), np.float_)
        visited = np.zeros((shell.shape[0], shell.shape[1]), np.bool_)

        distFunc = euclid
        goalFunc = euclid
        voroFunc = manhattan

        unwalkCoords = tuple(zip(*np.where(walkMap == 0)))

        print("voronoi start")
        start = time.time()

        rows, cols = obsDist.shape
        for row in range(rows):
            for col in range(cols):
                obsDist[row, col] = voroFunc(min(unwalkCoords, key=lambda co: voroFunc(co, (row, col))), (row, col))

        voroWeight = 0.1
        obsMax = np.max(obsDist)

        q = PriorityQueue()
        q.put((0, (int(shell.shape[0]/2), 0)))
        distTo[int(shell.shape[0]/2), 0] = 0

        goal = (int(shell.shape[0]/2)+10, shell.shape[1]-1)

        closestNode = (int(shell.shape[0]/2), 0)

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

                        tentDist = distTo[coords[0], coords[1]] + distFunc(coords, (row, col)) + obsMax - obsDist[row, col]*voroWeight

                        if distTo[row, col] > tentDist:
                            distTo[row, col] = tentDist
                            nodeTo[row, col] = coords
                            q.put((tentDist + goalFunc((row, col), goal), (row, col)))
                            visited[row, col] = 0

        path = np.zeros((shell.shape[0], shell.shape[1]), np.bool_)

        cur = closestNode

        negOne = np.iinfo(np.uint32).max

        while cur[0] != negOne and cur[1] != negOne:
            path[cur[0], cur[1]] = 1
            cur = nodeTo[cur[0], cur[1]]

        path[goal[0], goal[1]] = 1
        walkMap[goal[0], goal[1]] = 1

        cv2.imshow("walk", canWalk.astype(np.uint8)*255)

        cv2.imshow("test", np.rot90(walkable.astype(np.uint8)*int(255/3) + obstacles.astype(np.uint8)*int(255/3) + canWalk.astype(np.uint8)*int(255/3), 1))
        # cv2.imshow("ttt", obstacles.astype(np.uint8)*255)
        cv2.imshow("imgRef", imgRef)

        path2, _, _ = aStar.aStar(shell, unknowns, canWalk, (10, shell.shape[1]-1), verbose=True, distFunc=aStar.euclid, goalFunc=aStar.euclid)

        # fig = plt.figure(200)
        # imgplot = plt.imshow(walkable.astype(np.uint8)*int(255/3) + obstacles.astype(np.uint8)*int(255/3) + canWalk.astype(np.uint8)*int(255/3))
        plt.figure(100)
        imgplot2 = plt.imshow(np.rot90((walkMap*255/2 + path*255/2), 1))
        plt.figure(200)
        imgplot3 = plt.imshow(np.rot90((walkMap*255/2 + path2*255/2), 1))
        # fig = plt.figure(300)
        # imgplot3 = plt.imshow(distTo)
        plt.figure(400)
        imgplot4 = plt.imshow(np.rot90(obsDist, 1))
        plt.show()
