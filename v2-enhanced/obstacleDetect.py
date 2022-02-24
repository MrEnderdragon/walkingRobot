import hough
import numpy as np
import time
import cv2
import os
import log
import matplotlib.pyplot as plt

focalLen = 441.25  # pixels
baseline = 7.5 * 10  # mm
minSee = focalLen * baseline / 95

maxSize = 3000
# maxSize = 7000
# step = 100
step = 50
objstep = step
# objstep = 200

mid = [maxSize, maxSize, maxSize]

thresh = 35

maxCanOver = 50  # mm
minCanUnder = 400

walkFlatDone = False

if os.path.isfile('./robot_walk.npy'):
    with open('./robot_walk.npy', 'rb') as f:
        walkFlat = np.load(f)
        walkFlatDone = True


def mapArr(xs, ys, depth):
    imgRows, imgCols = depth.shape

    zs = depth[xs, ys]

    xs = (xs - imgRows / 2)
    ys = (ys - imgCols / 2)

    conv = np.divide(zs, focalLen)

    return np.array([ys * conv, conv * focalLen, -xs * conv])


def fits(minVal, maxVal, arr):
    for ind in arr:
        if ind < minVal or ind >= maxVal:
            # pass
            return False
    return True


def scale(coordsIn, midIn):
    return (coordsIn + midIn) / step


def unscale(coordsIn, midIn):
    coordss = coordsIn * step - midIn
    return coordss


def scaleOld(coordsIn, midIn):
    return np.add(coordsIn, midIn)


def detect(vDisp, disp, dep, verbose=False):
    """
        :param vDisp: v disparity
        :param disp: disparity
        :param dep: depth
        :param verbose: print info?
        :return: shellFlat, obsFlat, walkFlat
    """
    global walkFlat, walkFlatDone

    vDisp = vDisp[0:360, ...]
    disp = disp[0:360, ...]
    dep = dep[0:360, ...]

    _, dispSLIC, dispScaled, floor = hough.hough(disp, vDisp, slicc=False, verbose=verbose, m=100, k=500, scl=1.5, its=5)

    shellFlat = np.zeros((int(maxSize/step), int(maxSize/step)), dtype=np.bool_)

    depToUse = dep

    xsFloorLess, ysFloorLess = np.where(floor == 1)
    xsFloor, ysFloor = np.where(floor == 0)

    mapFloorLess = scale(mapArr(xsFloorLess, ysFloorLess, depToUse).reshape(3, -1).T, mid).astype(int)  # X, 3
    mapFloor = scale(mapArr(xsFloor, ysFloor, depToUse).reshape(3, -1).T, mid).astype(int)

    floorCoords = np.where(np.all([np.all(mapFloor > 0, axis=1),
                                   np.all(mapFloor < maxSize / step, axis=1)
                                   ], axis=0))

    uniqueF, countsF = np.unique(mapFloor[floorCoords], return_counts=True, axis=0)  # X, 3
    floorheight = np.max(uniqueF[:, 0])

    floorLessCoords = np.where(np.all([np.all(mapFloorLess > 0, axis=1),
                                       np.all(mapFloorLess < maxSize / step, axis=1),
                                       ((mapFloorLess > minSee / step)[:, 2]),
                                       ((mapFloorLess > floorheight+(maxCanOver/step))[:, 0]),
                                       ((mapFloorLess < floorheight+(minCanUnder/step))[:, 0])
                                       ], axis=0))

    uniqueFL, countsFL = np.unique(mapFloorLess[floorLessCoords], return_counts=True, axis=0)

    shellVox = uniqueFL[countsFL > thresh].T

    shellx, shelly, shellz = shellVox

    shellUnsc = unscale(shellVox.T, mid).T

    xzRatSmall = shellUnsc[0]/shellUnsc[2]
    xzRatBig = (shellUnsc[0]+step)/shellUnsc[2]
    yzRatSmall = shellUnsc[1]/shellUnsc[2]
    yzRatBig = (shellUnsc[1]+step)/shellUnsc[2]

    testZs = np.tile(shellUnsc[2].reshape(-1, 1), reps=(1, int(maxSize/objstep)-1))+np.arange(1, int(maxSize/objstep)).reshape(1, -1)*objstep

    xzStarts = (testZs*xzRatSmall.reshape(-1, 1))
    xzEnds = (testZs*xzRatBig.reshape(-1, 1))
    yzStarts = (testZs*yzRatSmall.reshape(-1, 1))
    yzEnds = (testZs*yzRatBig.reshape(-1, 1))

    obsFlat = np.zeros((int(maxSize/step), int(maxSize/step)), dtype=np.bool_)

    rowss, colss = xzStarts.shape
    for ind in range(rowss):
        for zInd in range(colss):
            cont = False
            for xx in range(xzStarts[ind, zInd].astype(int), xzEnds[ind, zInd].astype(int), step):
                for yy in range(yzStarts[ind, zInd].astype(int), yzEnds[ind, zInd].astype(int), step):
                    coords = scaleOld([xx, yy, testZs[ind, zInd]], mid)
                    if fits(0, maxSize, coords):
                        cont = True
                        if floorheight+(maxCanOver/step) < coords[0]/step < floorheight+(minCanUnder/step) and \
                                obsFlat[int(coords[1] / step), int(coords[2] / step)] == 0:
                            obsFlat[int(coords[1] / step), int(coords[2] / step)] = 1
            if not cont:
                break

    shellFlat[shelly, shellz] = 1

    walkFlat = np.zeros((int(maxSize/step), int(maxSize/step)), dtype=np.bool_)

    xzRat = (dep.shape[1]/2) / focalLen

    rows, cols = walkFlat.shape
    for row in range(rows):
        for col in range(cols):
            if -xzRat*max(col, int(minSee/step)) + rows/2 < row < xzRat*max(col, int(minSee/step)) + rows/2:
                walkFlat[row, col] = 1

    return shellFlat, obsFlat, walkFlat


def rot(coordinates, rotation):  # takes in X by 3
    coordinates[..., 0:2] = coordinates[..., 0:2] @ np.array([[np.cos(rotation), -np.sin(rotation)], [np.sin(rotation), np.cos(rotation)]]).T
    return coordinates


def detectMult(vDisps, disps, deps, rots, verbose=False, display=False, **args):
    """
        :param vDisps: array of v disparity
        :param disps: array of disparity
        :param deps: array of depth
        :param rots: array of camera rotations
        :param verbose: print info?
        :param display: display obstacles?
        :param args: floorThresh, lidarDists, lidarRots
        :return: shellFlat, obsFlat, walkFlat
    """
    global walkFlat, walkFlatDone

    floorThresh = args["floorThresh"] if "floorThresh" in args else 0.5
    lidarDists = np.array(args["lidarDists"]) if "lidarDists" in args else None
    lidarRots = np.array(args["lidarRots"]) if "lidarRots" in args else None

    mapFloorLess = np.zeros((0, 3), dtype=int)
    mapFloor = np.zeros((0, 3), dtype=int)

    shellFlat = np.zeros((int(maxSize*2 / step), int(maxSize*2 / step)), dtype=np.bool_)

    walkFlatI = None

    if not walkFlatDone:
        walkFlat = np.zeros((int(maxSize * 2 / step), int(maxSize * 2 / step)), dtype=np.bool_)
        walkFlatI = np.zeros((int(maxSize * 2 / step), int(maxSize * 2 / step)), dtype=np.uint8)

    # dep = deps[0][0:360, ...]

    start = time.time()

    if verbose:
        log.log("starting detect")

    amInval = 0

    for i in range(len(rots)):
        vDisp = vDisps[i][0:360, ...]
        disp = disps[i][0:360, ...]
        dep = deps[i][0:360, ...]

        _, dispSLIC, dispScaled, floor = hough.hough(disp, vDisp, slicc=False, verbose=verbose, m=100, k=500, scl=1.5, its=5)

        depToUse = dep

        xsFloorLess, ysFloorLess = np.where(np.all([floor == 0, dep != 0], axis=0))
        xsFloor, ysFloor = np.where(np.all([floor == 1, dep != 0], axis=0))

        # if verbose:
        #     log.log("floor: " + str(len(xsFloor)) + " out of " + str(len(xsFloor) + len(xsFloorLess)))

        if len(xsFloor) > (len(xsFloor) + len(xsFloorLess))*floorThresh:
            if verbose:
                log.log("floor: " + str(len(xsFloor)) + " out of " + str(len(xsFloor) + len(xsFloorLess)))

            amInval += 1
        else:
            mapFloor = np.append(mapFloor, np.floor(scale(rot(mapArr(xsFloor, ysFloor, depToUse).reshape(3, -1).T, rots[i]), mid)).astype(int), axis=0)

        mapFloorLess = np.append(mapFloorLess, np.floor(scale(rot(mapArr(xsFloorLess, ysFloorLess, depToUse).reshape(3, -1).T, rots[i]), mid)).astype(int), axis=0)  # X, 3

        # xzRat = (dep.shape[1] / 2) / focalLen
        if not walkFlatDone:
            angle = np.arctan2(((dep.shape[1] - 80) / 2), focalLen)
            rows, cols = walkFlat.shape
            contours = np.array([[int(rows/2), int(cols/2)],
                                 [int(rows/2) + rows * np.cos(rots[i] - angle), int(cols/2) + cols * np.sin(rots[i] - angle)],
                                 [int(rows/2) + rows * np.cos(rots[i] + angle), int(cols/2) + rows * np.sin(rots[i] + angle)]], np.int32)

            cv2.fillConvexPoly(walkFlatI, contours, color=100)
            cv2.circle(walkFlatI, (int(rows/2), int(cols/2)), int(minSee / step), 100, -1)
            # rows, cols = walkFlat.shape
            # for row in range(rows):
            #     for col in range(cols):
            #         rowNew = row - int(rows/2)
            #         colNew = col - int(cols/2)
            #         # if -xzRat * max(col, int(minSee / step)) + rows / 2 < row < xzRat * max(col, int(minSee / step)) + rows / 2:
            #         if rots[i] - angle < np.arctan2(rowNew, colNew) < rots[i] + angle or np.sqrt(rowNew**2 + colNew**2) < minSee/step:
            #             walkFlat[row, col] = 1

    if not walkFlatDone:
        walkFlat = walkFlatI > 10
        with open('./robot_walk.npy', 'wb') as ff:
            np.save(ff, walkFlat)
            walkFlatDone = True
        
    floorheight = 1
    if display:
        floorCoords = np.where(np.all([np.all(mapFloor > 0, axis=1),
                                       np.all(mapFloor < maxSize*2 / step, axis=1),
        
                                       (np.sqrt((mapFloor[:, 2] - mid[2] / step) ** 2 +
                                                (mapFloor[:, 1] - mid[1] / step) ** 2 +
                                                (mapFloor[:, 0] - mid[0] / step) ** 2) > minSee / step),
        
                                       ], axis=0))
        
        uniqueF, countsF = np.unique(mapFloor[floorCoords], return_counts=True, axis=0)  # X, 3
        floorheight = np.min(uniqueF[:, 2]) if len(uniqueF) > 0 else 0

    floorLessCoords = np.where(np.all([np.all(mapFloorLess > 0, axis=1),
                                       np.all(mapFloorLess < maxSize*2 / step, axis=1),

                                       (np.sqrt((mapFloorLess[:, 2] - mid[2] / step) ** 2 +
                                                (mapFloorLess[:, 1] - mid[1] / step) ** 2 +
                                                (mapFloorLess[:, 0] - mid[0] / step) ** 2) > minSee / step),

                                       ((mapFloorLess > floorheight+(maxCanOver/step))[:, 2]),
                                       ((mapFloorLess < floorheight+(minCanUnder/step))[:, 2])
                                       ], axis=0))

    uniqueFL, countsFL = np.unique(mapFloorLess[floorLessCoords], return_counts=True, axis=0)

    if lidarRots is not None and lidarDists is not None and len(lidarRots) > 0:
        print(lidarRots)
        print(lidarDists)

        places = np.array([-lidarDists * np.sin(lidarRots), lidarDists * np.cos(lidarRots), np.zeros(lidarRots.shape[0])]).T  # x, 3

        # places = np.array((3, lidarRots), dtype=int)
        # places[0, ...] = lidarDists * np.cos(lidarRots)
        # places[1, ...] = lidarDists * np.sin(lidarRots)
        # places[2, ...] = 0

        uniqueFL = np.append(uniqueFL, np.floor(scale(places, mid)).astype(int), axis=0)  # X, 3
        countsFL = np.append(countsFL, np.ones(len(lidarRots)) * thresh * 2)  # X, 3

    shellVox = uniqueFL[countsFL > thresh].T  # X by 3 into 3 by X

    shellx, shelly, shellz = shellVox

    if display:
        shellx, shelly, shellz = shellVox
        log.log(len(shellx))
        floorCoords = np.where(np.all([np.all(mapFloor > 0, axis=1),
                                       np.all(mapFloor < maxSize*2 / step, axis=1),

                                       (np.sqrt((mapFloor[:, 2] - mid[2] / step) ** 2 +
                                                (mapFloor[:, 1] - mid[1] / step) ** 2 +
                                                (mapFloor[:, 0] - mid[0] / step) ** 2) > minSee / step),

                                       ], axis=0))
        uniqueF, countsF = np.unique(mapFloor[floorCoords], return_counts=True, axis=0)  # X, 3
        floorVox = uniqueF[countsF > thresh].T
        floorx, floory, floorz = floorVox
        # log.log(len(floorx))
        import mayavi.mlab
        
        # fig = mayavi.mlab.figure('Point Cloud')

        mayavi.mlab.points3d(shellx, shelly, shellz, mode="cube",  scale_factor=0.8, color=(1, 0, 0))
        mayavi.mlab.points3d(floorx, floory, floorz, mode="cube",  scale_factor=0.8, color=(0, 1, 0))
        mayavi.mlab.show()

    shellUnsc = unscale(shellVox.T, mid)  # X by 3
    # shellUnsc = unscale(shellVox.T, mid).T  # 3 by X  

    obsFlat = np.zeros((int(maxSize*2/step), int(maxSize*2/step)), dtype=np.uint8)
    
    if verbose:
        end = time.time()
        log.log(end-start)
        log.log("done detect")

    # start = time.time()

    if verbose:
        start = time.time()
        log.log("starting unknowns")

    for ind, pos in enumerate(shellUnsc):

        if pos[0] == 0 and pos[1] == 0:
            continue

        xyAngs = [0, 0, 0, 0]
        xyAngs[0] = np.arctan2(pos[1], pos[0])
        xyAngs[1] = np.arctan2(pos[1]+1, pos[0])  # * (-1 if pos[1]+1 == 0 and pos[1] < 0 else 1)
        xyAngs[2] = np.arctan2(pos[1], pos[0]+1)
        xyAngs[3] = np.arctan2(pos[1]+1, pos[0]+1)  # * (-1 if pos[1]+1 == 0 and pos[1] < 0 else 1)

        angleRight = np.min(xyAngs)
        angleLeft = np.max(xyAngs)
        
        pCoords = scaleOld([pos[0], pos[1], 0], mid)
        # if obsFlat[int(coords1[1]/step), int(coords1[0]/step)] == 100:
        #    continue
        tSize = int(maxSize*1.5)
        rightCoords = scaleOld([tSize * np.cos(angleRight), tSize * np.sin(angleRight), 0], mid)
        leftCoords = scaleOld([tSize * np.cos(angleLeft), tSize * np.sin(angleLeft), 0], mid)
        # contours = np.array([[int(coords1[1]/step), int(coords1[0]/step)],
        #                      [int(coords2[1]/step), int(coords2[0]/step)],
        #                      [int(coords3[1]/step), int(coords3[0]/step)]],np.int32)
        #
        # cv2.fillConvexPoly(obsFlat, contours, color=100)

        cv2.line(obsFlat, (int(pCoords[1]/step), int(pCoords[0]/step)), (int(rightCoords[1]/step), int(rightCoords[0]/step)), 100, 1)
        cv2.line(obsFlat, (int(pCoords[1]/step), int(pCoords[0]/step)), (int(leftCoords[1]/step), int(leftCoords[0]/step)), 100, 1)
        
    # if verbose:
    #     cv2.imshow("test", test)
            
    obsFlatB = obsFlat > 10
    
    if verbose:
        end = time.time()
        log.log(end-start)
        log.log("done unknowns")

    shellFlat[shellx, shelly] = 1

    valid = len(shellx) > 5

    if not len(shellx) > 5 and verbose:
        log.log("INVALID: SHELL LENGTH < 5")

    # if not amInval < 2 and verbose:
    #     log.log("INVALID: VALID PICTURES < 2")

    return shellFlat, obsFlatB, walkFlat, valid


def detectMultHeights(vDisps, disps, deps, rots, verbose=False, display=False, **args):
    """
        :param vDisps: array of v disparity
        :param disps: array of disparity
        :param deps: array of depth
        :param rots: array of camera rotations
        :param verbose: print info?
        :param display: display obstacles?
        :param args: floorThresh, lidarDists, lidarRots
        :return: shellFlat, obsFlat, walkFlat
    """
    global walkFlat, walkFlatDone

    lidarHeight = args["lidarHeight"] if "lidarHeight" in args else 90+130
    lidarDists = np.array(args["lidarDists"]) if "lidarDists" in args else None
    lidarRots = np.array(args["lidarRots"]) if "lidarRots" in args else None

    # mapFloorLess = np.zeros((0, 3), dtype=int)
    mapFloor = np.zeros((0, 3), dtype=int)
    mapAll = np.zeros((0, 3), dtype=int)
    mapAllCloud = np.zeros((0, 3), dtype=float)

    allFlat = np.zeros((int(maxSize * 2 / step), int(maxSize * 2 / step)), dtype=np.bool_)
    allVox3 = np.zeros((int(maxSize * 2 / step), int(maxSize * 2 / step), int(maxSize * 2 / step)), dtype=np.bool_)
    heightFlat = np.zeros((int(maxSize * 2 / step), int(maxSize * 2 / step)), dtype=np.float_)

    walkFlatI = None

    if not walkFlatDone:
        walkFlat = np.zeros((int(maxSize * 2 / step), int(maxSize * 2 / step)), dtype=np.bool_)
        walkFlatI = np.zeros((int(maxSize * 2 / step), int(maxSize * 2 / step)), dtype=np.uint8)

    # dep = deps[0][0:360, ...]

    start = time.time()

    if verbose:
        log.log("starting detect")

    amInval = 0

    for i in range(len(rots)):
        vDisp = vDisps[i][0:360, ...]
        disp = disps[i][0:360, ...]
        dep = deps[i][0:360, ...]

        _, dispSLIC, dispScaled, floor = hough.hough(disp, vDisp, slicc=False, verbose=verbose, m=100, k=500, scl=1.5, its=5)

        depToUse = dep

        # xsFloorLess, ysFloorLess = np.where(np.all([floor == 0, dep != 0], axis=0))
        xsFloor, ysFloor = np.where(np.all([floor == 1, dep != 0], axis=0))
        xsAll, ysAll = np.where(dep != 0)

        mapFloor = np.append(mapFloor, np.floor(scale(rot(mapArr(xsFloor, ysFloor, depToUse).reshape(3, -1).T, rots[i]), mid)).astype(int), axis=0)

        mapAll = np.append(mapAll, np.floor(scale(rot(mapArr(xsAll, ysAll, depToUse).reshape(3, -1).T, rots[i]), mid)).astype(int), axis=0)  # X, 3
        mapAllCloud = np.append(mapAll, scale(rot(mapArr(xsAll, ysAll, depToUse).reshape(3, -1).T, rots[i]), mid), axis=0)  # X, 3

        if not walkFlatDone:
            angle = np.arctan2(((dep.shape[1] - 80) / 2), focalLen)
            rows, cols = walkFlat.shape
            contours = np.array([[int(rows / 2), int(cols / 2)],
                                 [int(rows / 2) + rows * np.cos(rots[i] - angle),
                                  int(cols / 2) + cols * np.sin(rots[i] - angle)],
                                 [int(rows / 2) + rows * np.cos(rots[i] + angle),
                                  int(cols / 2) + rows * np.sin(rots[i] + angle)]], np.int32)

            cv2.fillConvexPoly(walkFlatI, contours, color=100)
            cv2.circle(walkFlatI, (int(rows / 2), int(cols / 2)), int(minSee / step), 100, -1)

    if not walkFlatDone:
        walkFlat = walkFlatI > 10
        with open('./robot_walk.npy', 'wb') as ff:
            np.save(ff, walkFlat)
            walkFlatDone = True

    floorCoords = np.where(np.all([np.all(mapFloor > 0, axis=1),
                                   np.all(mapFloor < maxSize * 2 / step, axis=1),

                                   (np.sqrt((mapFloor[:, 2] - mid[2] / step) ** 2 +
                                            (mapFloor[:, 1] - mid[1] / step) ** 2 +
                                            (mapFloor[:, 0] - mid[0] / step) ** 2) > minSee / step),

                                   ], axis=0))

    allCoords = np.where(np.all([np.all(mapAll > 0, axis=1),
                                 np.all(mapAll < maxSize * 2 / step, axis=1),

                                 (np.sqrt((mapAll[:, 2] - mid[2] / step) ** 2 +
                                          (mapAll[:, 1] - mid[1] / step) ** 2 +
                                          (mapAll[:, 0] - mid[0] / step) ** 2) > minSee / step),

                                 ], axis=0))

    uniqueF, countsF = np.unique(mapFloor[floorCoords], return_counts=True, axis=0)  # X, 3
    uniqueA, countsA = np.unique(mapAll[allCoords], return_counts=True, axis=0)  # X, 3
    floorheight = np.average(uniqueF[:, 2]) if len(uniqueF) > 0 else 0

    if lidarRots is not None and lidarDists is not None and len(lidarRots) > 0:
        print(lidarRots)
        print(lidarDists)

        places = np.array(
            [-lidarDists * np.sin(lidarRots), lidarDists * np.cos(lidarRots), np.zeros(lidarRots.shape[0])]).T  # x, 3

        # places = np.array((3, lidarRots), dtype=int)
        # places[0, ...] = lidarDists * np.cos(lidarRots)
        # places[1, ...] = lidarDists * np.sin(lidarRots)
        # places[2, ...] = 0

        uniqueA = np.append(uniqueA, np.floor(scale(places, mid)).astype(int), axis=0)  # X, 3
        countsA = np.append(countsA, np.ones(len(lidarRots)) * thresh * 2)  # X, 3

    # shellVox = uniqueFL[countsFL > thresh].T  # X by 3 into 3 by X

    allVox = uniqueA[countsA > thresh].T

    allx, ally, allz = allVox
    allFlat[allx, ally] = 1
    allVox3[allx, ally, allz] = 1

    amAllPoints = mapAllCloud.shape[0]

    for j in range(amAllPoints):
        x = int(mapAllCloud[j, 0])
        y = int(mapAllCloud[j, 1])
        z = int(mapAllCloud[j, 2])
        zF = mapAllCloud[j, 2]

        if 0 < x < maxSize*2/step and 0 < y < maxSize*2/step and 0 < z < maxSize*2/step:
            if allVox3[x, y, z]:
                heightFlat[x, y] = max(heightFlat[x, y], zF)

    heightFlat -= floorheight
    heightFlat = np.maximum(heightFlat, 0)

    print(floorheight)

    if display:
        shellx, shelly, shellz = allVox
        log.log(len(shellx))
        floorCoords = np.where(np.all([np.all(mapFloor > 0, axis=1),
                                       np.all(mapFloor < maxSize * 2 / step, axis=1),

                                       (np.sqrt((mapFloor[:, 2] - mid[2] / step) ** 2 +
                                                (mapFloor[:, 1] - mid[1] / step) ** 2 +
                                                (mapFloor[:, 0] - mid[0] / step) ** 2) > minSee / step),

                                       ], axis=0))
        uniqueF, countsF = np.unique(mapFloor[floorCoords], return_counts=True, axis=0)  # X, 3
        floorVox = uniqueF[countsF > thresh].T
        floorx, floory, floorz = floorVox
        # log.log(len(floorx))
        import mayavi.mlab

        # fig = mayavi.mlab.figure('Point Cloud')

        mayavi.mlab.points3d(shellx, shelly, shellz, mode="cube", scale_factor=0.8, color=(1, 0, 0))
        mayavi.mlab.points3d(floorx, floory, floorz, mode="cube", scale_factor=0.8, color=(0, 1, 0))
        mayavi.mlab.show()

    allUnsc = unscale(allVox.T, mid)  # X by 3

    if verbose:
        end = time.time()
        log.log(end - start)
        log.log("done detect")

    if verbose:
        start = time.time()
        log.log("starting unknowns")

    obsHeightFlat = np.copy(heightFlat)

    for ind, pos in enumerate(allUnsc):

        tmpFlat = np.zeros((int(maxSize * 2 / step), int(maxSize * 2 / step)), dtype=np.uint8)

        if pos[0] == 0 and pos[1] == 0:
            continue

        xyAngs = [0, 0, 0, 0]
        xyAngs[0] = np.arctan2(pos[1], pos[0])
        xyAngs[1] = np.arctan2(pos[1] + 1, pos[0])  # * (-1 if pos[1]+1 == 0 and pos[1] < 0 else 1)
        xyAngs[2] = np.arctan2(pos[1], pos[0] + 1)
        xyAngs[3] = np.arctan2(pos[1] + 1, pos[0] + 1)  # * (-1 if pos[1]+1 == 0 and pos[1] < 0 else 1)

        angleRight = np.min(xyAngs)
        angleLeft = np.max(xyAngs)

        pCoords = scaleOld([pos[0], pos[1], 0], mid)
        tSize = int(maxSize * 1.5)
        rightCoords = scaleOld([tSize * np.cos(angleRight), tSize * np.sin(angleRight), 0], mid)
        leftCoords = scaleOld([tSize * np.cos(angleLeft), tSize * np.sin(angleLeft), 0], mid)

        # pt1 = np.array((int(pCoords[1] / step), int(pCoords[0] / step)))
        # pt2 = np.array((int(rightCoords[1] / step), int(rightCoords[0] / step)))
        # pt3 = np.array((int(leftCoords[1] / step), int(leftCoords[0] / step)))
        # cv2.drawContours(tmpFlat, [np.array([pt1, pt2, pt3])], 0, 1, 2)

        cv2.line(tmpFlat, (int(pCoords[1] / step), int(pCoords[0] / step)), (int(rightCoords[1] / step), int(rightCoords[0] / step)), 1, 1)
        cv2.line(tmpFlat, (int(pCoords[1] / step), int(pCoords[0] / step)), (int((rightCoords[1]+leftCoords[1])/2 / step),
                                                                             int((rightCoords[0]+leftCoords[0])/2 / step)), 1, 2)
        cv2.line(tmpFlat, (int(pCoords[1] / step), int(pCoords[0] / step)), (int(leftCoords[1] / step), int(leftCoords[0] / step)), 1, 1)

        obsHeightFlat = np.maximum(obsHeightFlat, (tmpFlat.astype(np.float_) * float(heightFlat[allVox[0, ind], allVox[1, ind]])))

    if display:
        plt.figure(333)
        plt.imshow(heightFlat)
        plt.figure(222)
        plt.imshow(obsHeightFlat)
        plt.figure(444)
        plt.imshow(allFlat)
        plt.show()

    if verbose:
        end = time.time()
        log.log(end - start)
        log.log("done unknowns")

    valid = len(allx) > 5 and amInval < 2

    if not len(allx) > 5 and verbose:
        log.log("INVALID: SHELL LENGTH < 5")

    if not amInval < 2 and verbose:
        log.log("INVALID: VALID PICTURES < 2")

    return obsHeightFlat, walkFlat, valid
