import hough
import numpy as np

focalLen = 441.25  # pixels
ppmm = 1000/3  # pixels per mm, 1p = 3um
baseline = 7.5 * 10  # mm
minSee = focalLen * baseline / 95

maxSize = 3000
step = 50
objstep = step

mid = [maxSize / 10, maxSize / 2, 0]

thresh = 35

maxCanOver = 50  # mm
minCanUnder = 400


def mapArr(xs, ys, depth):
    imgRows, imgCols = depth.shape

    zs = depth[xs, ys]

    xs = (xs - imgRows / 2)
    ys = (ys - imgCols / 2)

    conv = np.divide(zs, np.sqrt(focalLen ** 2 + np.square(xs) + np.square(ys)))

    return np.array([xs * conv, ys * conv, conv * focalLen])


def fits(minVal, maxVal, arr):
    for ind in arr:
        if ind < minVal or ind >= maxVal:
            # pass
            return False
    return True


def scale(coordsIn, midIn):
    coordsIn[..., 0] *= -1
    return (coordsIn + midIn) / step


def unscale(coordsIn, midIn):
    coordss = coordsIn * step - midIn
    coordss[..., 0] *= -1
    return coordss


def scaleOld(coordsIn, midIn):
    coordsIn[0] *= -1
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
