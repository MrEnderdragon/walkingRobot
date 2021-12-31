import hough
import numpy as np
import time
import cv2
import os

focalLen = 441.25  # pixels
ppmm = 1000/3  # pixels per mm, 1p = 3um
baseline = 7.5 * 10  # mm
minSee = focalLen * baseline / 95

maxSize = 3000
# maxSize = 7000
# step = 100
step = 50
objstep = step
# objstep = 200

mid = [maxSize, maxSize, maxSize/10]

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
        :param args: floorThresh
        :return: shellFlat, obsFlat, walkFlat
    """
    global walkFlat, walkFlatDone

    floorThresh = args["floorThresh"] if "floorThresh" in args else 0.5

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
        print("starting detect")

    for i in range(len(rots)):
        vDisp = vDisps[i][0:360, ...]
        disp = disps[i][0:360, ...]
        dep = deps[i][0:360, ...]

        _, dispSLIC, dispScaled, floor = hough.hough(disp, vDisp, slicc=False, verbose=verbose, m=100, k=500, scl=1.5, its=5)

        depToUse = dep

        xsFloorLess, ysFloorLess = np.where(floor == 0)
        xsFloor, ysFloor = np.where(floor == 1)

        if len(xsFloor) > (len(xsFloor) + len(xsFloorLess))*floorThresh:
            if verbose:
                print("floor: " + str(len(xsFloor)) + " out of " + str(len(xsFloor) + len(xsFloorLess)))
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
        with open('./robot_walk.npy', 'wb') as f:
            np.save(f, walkFlat)
            walkFlatDone = True
        
    # floorCoords = np.where(np.all([np.all(mapFloor > 0, axis=1),
    #                                np.all(mapFloor < maxSize*2 / step, axis=1),
    #
    #                                (np.sqrt((mapFloor[:, 2] - mid[2] / step) ** 2 +
    #                                         (mapFloor[:, 1] - mid[1] / step) ** 2 +
    #                                         (mapFloor[:, 0] - mid[0] / step) ** 2) > minSee / step),
    #
    #                                ], axis=0))

    #uniqueF, countsF = np.unique(mapFloor[floorCoords], return_counts=True, axis=0)  # X, 3
    #floorheight = np.min(uniqueF[:, 2]) if len(uniqueF) > 0 else 0
    floorheight = 1

    floorLessCoords = np.where(np.all([np.all(mapFloorLess > 0, axis=1),
                                       np.all(mapFloorLess < maxSize*2 / step, axis=1),

                                       (np.sqrt((mapFloorLess[:, 2] - mid[2] / step) ** 2 +
                                                (mapFloorLess[:, 1] - mid[1] / step) ** 2 +
                                                (mapFloorLess[:, 0] - mid[0] / step) ** 2) > minSee / step),

                                       ((mapFloorLess > floorheight+(maxCanOver/step))[:, 2]),
                                       ((mapFloorLess < floorheight+(minCanUnder/step))[:, 2])
                                       ], axis=0))

    uniqueFL, countsFL = np.unique(mapFloorLess[floorLessCoords], return_counts=True, axis=0)

    shellVox = uniqueFL[countsFL > thresh].T

    shellx, shelly, shellz = shellVox

    if display:
        shellx, shelly, shellz = shellVox
        print(len(shellx))
        # floorVox = uniqueF[countsF > thresh].T
        # floorx, floory, floorz = floorVox
        # print(len(floorx))
        import mayavi.mlab

        mayavi.mlab.points3d(shellx, shelly, shellz, mode="cube", scale_factor=0.8, color=(1, 0, 0))
        #mayavi.mlab.points3d(floorx, floory, floorz, mode="cube", scale_factor=0.8, color=(0, 1, 0))
        mayavi.mlab.show()

    shellUnsc = unscale(shellVox.T, mid)  # X by 3
    # shellUnsc = unscale(shellVox.T, mid).T  # 3 by X

    obsFlat = np.zeros((int(maxSize*2/step), int(maxSize*2/step)), dtype=np.uint8)
    
    if verbose:
        end = time.time()
        print(end-start)
        print("done detect")

    # start = time.time()

    if verbose:
        start = time.time()
        print("starting unknowns")

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
        
        coords1 = scaleOld([pos[0], pos[1], 0], mid)
        #if obsFlat[int(coords1[1]/step), int(coords1[0]/step)] == 100:
        #    continue
        tSize = int(maxSize*1.5)
        coords2 = scaleOld([tSize * np.cos(angleRight), tSize * np.sin(angleRight), 0], mid)
        coords3 = scaleOld([tSize * np.cos(angleLeft), tSize * np.sin(angleLeft), 0], mid)
        # contours = np.array([[int(coords1[1]/step), int(coords1[0]/step)],
        #                      [int(coords2[1]/step), int(coords2[0]/step)],
        #                      [int(coords3[1]/step), int(coords3[0]/step)]],np.int32)
        #
        # cv2.fillConvexPoly(obsFlat, contours, color=100)
        
        
        cv2.line(obsFlat, (int(coords1[1]/step), int(coords1[0]/step)), (int(coords2[1]/step), int(coords2[0]/step)), 100, 1)
        
        cv2.line(obsFlat, (int(coords1[1]/step), int(coords1[0]/step)), (int(coords3[1]/step), int(coords3[0]/step)), 100, 1)

    
    contours, _ = cv2.findContours(obsFlat, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    # test = np.zeros((int(maxSize*2/step), int(maxSize*2/step)), dtype=np.uint8)
    
    
    unWalkCoords = []
    for contour in contours:
        for item in contour:
            unWalkCoords.append((item[0][1], item[0][0]))
            # test[item[0][1], item[0][0]] = 255
        
    # cv2.imshow("test", test)
            
    obsFlatB = obsFlat > 10
    
    if verbose:
        end = time.time()
        print(end-start)
        print("done unknowns")
    

    shellFlat[shellx, shelly] = 1

    return shellFlat, obsFlatB, walkFlat, np.array(unWalkCoords)
