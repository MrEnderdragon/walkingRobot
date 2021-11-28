import hough
import matplotlib.pyplot as plt
import cv2
import numpy as np
import mayavi.mlab
import time

# focalLen = 19.6 * 10  # mm
focalLen = 441.25  # pixels
ppmm = 1000/3  # pixels per mm, 1p = 3um
baseline = 7.5 * 10  # mm
minSee = focalLen * baseline / 95

maxSize = 3000
# step = int(maxSize / 100)
step = 50
objstep = step

mid = [maxSize / 10, maxSize / 2, 0]

thresh = 35

fig = plt.figure(300)
ax = plt.axes()


# def mapPoints(inX, inY, depth):
#     if depth == 0:
#         return None
#
#     # hypLen = math.sqrt(inX**2 + inY**2 + focalLen**2)
#     conv = depth/focalLen
#     return [inX/ppmm * conv, inY/ppmm * conv, focalLen * conv]

def mapPoints(inX, inY, depth):
    if depth == 0:
        return None

    # x_over_z = inX / focalLen
    # y_over_z = inY / focalLen
    # zz = depth / np.sqrt(focalLen + x_over_z ** 2 + y_over_z ** 2)
    # xx = x_over_z * zz
    # yy = y_over_z * zz

    conv = depth / focalLen  # np.sqrt(focalLen**2 + inX ** 2 + inY ** 2)

    return [inX*conv, inY*conv, focalLen*conv]


# def mapArr(depth):
#     imgRows, imgCols = depth.shape
#
#     xs = np.arange(-imgCols+int(imgCols/2)+1, int(imgCols/2)+1)
#     xs = np.tile(xs, (imgRows, 1))
#
#     ys = np.arange(-imgRows + int(imgRows / 2) + 1, int(imgRows / 2) + 1)
#     ys = np.tile(ys, (imgCols, 1))
#     ys = np.transpose(ys)
#
#     conv = np.divide(depth, np.sqrt(focalLen**2 + np.square(xs) + np.square(ys)))
#
#     return np.array([xs * conv, ys * conv, conv * focalLen])


def mapArr(xs, ys, depth):
    imgRows, imgCols = depth.shape

    zs = dep[xs, ys]

    xs = (xs - imgRows / 2)
    ys = (ys - imgCols / 2)

    conv = np.divide(zs, focalLen)  # np.sqrt(focalLen ** 2 + np.square(xs) + np.square(ys)))

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


def unscaleOld(coordsIn, midIn):
    coordss = np.subtract(coordsIn, midIn)
    coordss[0] *= -1
    return coordss


def eq(in1, in2):
    for ind in range(len(in1)):
        if in1[ind] != in2[ind]:
            return False
    return True


if __name__ == "__main__":

    for i in range(15, 17+1):
        # Constructing test image
        imgId = "CAL-SPLR-" + str(i) + ".png"
        vDisp = cv2.imread("vDisp/vDisp" + imgId, cv2.IMREAD_UNCHANGED)
        disp = cv2.imread("dispImages/disp" + imgId, cv2.IMREAD_UNCHANGED)
        dep = cv2.imread("depImages/depth" + imgId, cv2.IMREAD_UNCHANGED)
        colour = cv2.imread("RImages/R" + imgId, cv2.IMREAD_COLOR)

        start = time.time()

        _, dispSLIC, dispScaled, floor = hough.hough(disp, vDisp, slicc=False, slicRef=colour, verbose=True, m=100, k=500, scl=1.5, its=5)

        vDisp = vDisp[0:360, ...]
        disp = disp[0:360, ...]
        dep = dep[0:360, ...]
        colour = colour[0:360, ...]
        floor = floor[0:360, ...]

        depFloorless = dep * floor

        shellGrid = np.zeros((int(maxSize/step)+1, int(maxSize/step)+1, int(maxSize/step)+1), dtype=np.uint8)
        floorGrid = np.zeros((int(maxSize/step)+1, int(maxSize/step)+1, int(maxSize/step)+1), dtype=np.uint8)
        colorGrid = np.zeros((int(maxSize/step)+1, int(maxSize/step)+1, int(maxSize/step)+1), dtype=np.uint32)

        shellFlat = np.zeros((int(maxSize/step), int(maxSize/step)), dtype=np.bool_)

        depToUse = dep

        xsFloorLess, ysFloorLess = np.where(floor == 1)
        xsFloor, ysFloor = np.where(floor == 0)

        mapFloorLess = scale(mapArr(xsFloorLess, ysFloorLess, depToUse).reshape(3, -1).T, mid).astype(int)  # X, 3
        mapFloor = scale(mapArr(xsFloor, ysFloor, depToUse).reshape(3, -1).T, mid).astype(int)

        # print((mapFloorLess > minSee))
        # print(np.all(mapFloorLess > 0, axis=1))

        floorCoords = np.where(np.all([np.all(mapFloor > 0, axis=1),
                                       np.all(mapFloor < maxSize / step, axis=1)
                                       ], axis=0))

        uniqueF, countsF = np.unique(mapFloor[floorCoords], return_counts=True, axis=0)  # X, 3
        floorheight = np.max(uniqueF[:, 0])

        floorLessCoords = np.where(np.all([np.all(mapFloorLess > 0, axis=1),
                                           np.all(mapFloorLess < maxSize / step, axis=1),
                                           ((mapFloorLess > minSee / step)[:, 2]),
                                           ((mapFloorLess > floorheight+(0/step))[:, 0]),
                                           ((mapFloorLess < floorheight+(4000/step))[:, 0])
                                           ], axis=0))

        uniqueFL, countsFL = np.unique(mapFloorLess[floorLessCoords], return_counts=True, axis=0)

        shellVox = uniqueFL[countsFL > thresh].T
        floorVox = uniqueF[countsF > 0].T

        # print(obs)
        cv2.imshow("floorless", cv2.applyColorMap(cv2.convertScaleAbs(depFloorless*10, alpha=(255.0/65535.0)), cv2.COLORMAP_JET))
        cv2.imshow("slic", cv2.applyColorMap(cv2.convertScaleAbs(dispSLIC*20, alpha=(255.0/65535.0)), cv2.COLORMAP_JET))
        cv2.imshow("floor", floor*255)
        cv2.imshow("col", colour)
        cv2.imshow("vDisp", vDisp*10)

        shellx, shelly, shellz = shellVox
        colorGrid = np.divide(colorGrid, shellGrid)
        cols = colorGrid[shellx, shelly, shellz].astype(np.uint8)
        cx, cy, cz = floorVox

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
        zs = testZs.reshape(1, -1)

        # print(testZs.shape)

        objGrid = np.zeros((int(maxSize/step)+1, int(maxSize/step)+1, int(maxSize/step)+1), dtype=np.uint8)
        obsFlat = np.zeros((int(maxSize/step), int(maxSize/step)), dtype=np.bool_)

        print(floorheight)

        rowss, colss = xzStarts.shape
        for ind in range(rowss):
            for zInd in range(colss):
                # print(ind, zInd)
                cont = False
                for xx in range(xzStarts[ind, zInd].astype(int), xzEnds[ind, zInd].astype(int), step):
                    for yy in range(yzStarts[ind, zInd].astype(int), yzEnds[ind, zInd].astype(int), step):
                        coords = scaleOld([xx, yy, testZs[ind, zInd]], mid)
                        if fits(0, maxSize, coords):
                            cont = True
                            # if floorheight+(50/step) < coords[0]/step < floorheight+(400/step):
                            if True:
                                objGrid[int(coords[0] / step), int(coords[1] / step), int(coords[2] / step)] += 1
                                if obsFlat[int(coords[1] / step), int(coords[2] / step)] == 0:
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

        end = time.time()
        print(end - start)

        ox, oy, oz = np.where(objGrid > 0)

        objects = mayavi.mlab.points3d(-oy, -oz, ox, mode="cube", scale_factor=0.8, color=(0, 0, 0.6), opacity=1)
        # objects = mayavi.mlab.points3d(-oy, -oz, ox, mode="cube", scale_factor=0.8, color=(0, 0, 0.9))
        # nodes2 = mayavi.mlab.points3d(sx, sy, sz, mode="cube", scale_factor=0.8, color=(0, 1, 0))
        shell = mayavi.mlab.points3d(-shelly, -shellz, shellx, mode="cube", scale_factor=0.8, color=(1, 0, 0))
        floor = mayavi.mlab.points3d(-cy, -cz, cx, mode="cube", scale_factor=0.8, color=(0, 1, 0))
        # mayavi.mlab.outline(color=(0, 0, 0))

        shell.glyph.scale_mode = 'scale_by_vector'
        # shell.glyph.color_mode = "color_by_scalar"

        # shell.mlab_source.dataset.point_data.scalars = cols

        cv2.imshow("2d project", (shellFlat + obsFlat).astype(np.uint8)*255)
        cv2.imshow("2d projectt", (shellFlat).astype(np.uint8)*255)
        cv2.imshow("2d projecttr", (walkFlat).astype(np.uint8)*255)

        cv2.imwrite('./flatShell/shell' + str(imgId), shellFlat.astype(np.uint8))
        cv2.imwrite('./flatUnknown/unknown' + str(imgId), obsFlat.astype(np.uint8))
        cv2.imwrite('./flatCanWalk/canWalk' + str(imgId), walkFlat.astype(np.uint8))

        cv2.imwrite('./flatShell/shell255' + str(imgId), (shellFlat*255).astype(np.uint8))
        cv2.imwrite('./flatUnknown/unknown255' + str(imgId), (obsFlat*255).astype(np.uint8))
        cv2.imwrite('./flatCanWalk/canWalk255' + str(imgId), (walkFlat*255).astype(np.uint8))

        mayavi.mlab.show()

        # plt.show()

