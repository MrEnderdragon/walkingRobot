import hough
import matplotlib.pyplot as plt
import cv2
import numpy as np
print("mayavi load")
import mayavi.mlab

focalLen = 19.6 * 10  # mm
focalLen = 441.25  # pixels
ppmm = 1000/3  # pixels per mm, 1p = 3um
baseline = 7.5 * 10  # mm

maxSize = 5000
step = int(maxSize / 100)

mid = [maxSize / 10, maxSize / 2, 0]

thresh = (200/1000000)*(step**3)

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

    conv = depth / np.sqrt(focalLen**2 + inX ** 2 + inY ** 2)

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
    conv = np.divide(zs, np.sqrt(focalLen ** 2 + np.square(xs) + np.square(ys)))

    xs = (xs - imgRows/2)
    ys = (ys - imgCols/2)

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


def scaleOld(coordsIn, midIn):
    coordsIn[0] *= -1
    return np.add(coordsIn, midIn)


def unscale(coordsIn, midIn):
    coordss = np.subtract(coordsIn, midIn)
    coordss[0] *= -1
    return coordss


def eq(in1, in2):
    for ind in range(len(in1)):
        if in1[ind] != in2[ind]:
            return False
    return True


if __name__ == "__main__":

    for i in range(5, 10):
        # Constructing test image
        imgId = "SPLR-" + str(i) + ".png"
        vDisp = cv2.imread("vDisp/vDisp" + imgId, cv2.IMREAD_UNCHANGED)
        disp = cv2.imread("dispImages/disp" + imgId, cv2.IMREAD_UNCHANGED)
        dep = cv2.imread("depImages/depth" + imgId, cv2.IMREAD_UNCHANGED)
        colour = cv2.imread("RImages/R" + imgId, cv2.IMREAD_COLOR)

        vDisp = vDisp[0:350, ...]
        disp = disp[0:360, ...]
        dep = dep[0:360, ...]
        colour = colour[0:360, ...]

        _, dispSLIC, dispScaled, floor = hough.hough(disp, vDisp, slicc=False, slicRef=colour, verbose=True, m=100, k=500, scl=1.5, its=5)

        depFloorless = dep * floor

        shellGrid = np.zeros((int(maxSize/step)+1, int(maxSize/step)+1, int(maxSize/step)+1), dtype=np.uint8)
        floorGrid = np.zeros((int(maxSize/step)+1, int(maxSize/step)+1, int(maxSize/step)+1), dtype=np.uint8)
        colorGrid = np.zeros((int(maxSize/step)+1, int(maxSize/step)+1, int(maxSize/step)+1), dtype=np.uint32)

        flatGrid = np.zeros((int(maxSize/step)+1, int(maxSize/step)+1, 3), dtype=np.uint8)

        depToUse = dep

        xsFloorLess, ysFloorLess = np.where(floor == 1)
        xsFloor, ysFloor = np.where(floor == 0)

        mapFloorLess = scale(mapArr(xsFloorLess, ysFloorLess, depToUse).reshape(3, -1).T, mid).astype(int)
        mapFloor = scale(mapArr(xsFloor, ysFloor, depToUse).reshape(3, -1).T, mid).astype(int)

        floorLessCoords = np.where(np.all([np.all(mapFloorLess > 0, axis=1),
                                           np.all(mapFloorLess < maxSize / step, axis=1),
                                           ], axis=0))

        floorCoords = np.where(np.all([np.all(mapFloor > 0, axis=1),
                                       np.all(mapFloor < maxSize / step, axis=1)], axis=0))

        uniqueFL, countsFL = np.unique(mapFloorLess[floorLessCoords], return_counts=True, axis=0)
        uniqueF, countsF = np.unique(mapFloor[floorCoords], return_counts=True, axis=0)

        shellVox = uniqueFL[countsFL > thresh].T
        floorVox = uniqueF[countsF > thresh].T

        # print(obs)
        cv2.imshow("floorless", cv2.applyColorMap(cv2.convertScaleAbs(depFloorless*10, alpha=(255.0/65535.0)), cv2.COLORMAP_JET))
        cv2.imshow("slic", cv2.applyColorMap(cv2.convertScaleAbs(dispSLIC*10, alpha=(255.0/65535.0)), cv2.COLORMAP_JET))
        cv2.imshow("floor", floor*255)
        cv2.imshow("col", colour)
        cv2.imshow("vDisp", vDisp*10)

        # ox, oy, oz = np.where(np.logical_and(shellGrid > 5, shellGrid < 10))
        # sx, sy, sz = np.where(np.logical_and(shellGrid > 10, shellGrid < 30))
        bx, by, bz = shellVox
        colorGrid = np.divide(colorGrid, shellGrid)
        cols = colorGrid[bx, by, bz].astype(np.uint8)
        cx, cy, cz = floorVox
        # print(cols)
        print("cols", len(cols))
        print("bx", len(bx))
        print("cx", len(cx))

        objGrid = np.zeros((int(maxSize/step)+1, int(maxSize/step)+1, int(maxSize/step)+1), dtype=np.uint8)

        for voxInd in range(len(bx)):

            unscCoords = unscale([bx[voxInd] * step, by[voxInd] * step, bz[voxInd] * step], mid)

            x = unscCoords[0]
            y = unscCoords[1]
            z = unscCoords[2]

            if -10 < bx[voxInd] * step < 500:
                flatGrid[by[voxInd], bz[voxInd]] = [0, 0, 255]

            xzRatSmall = float(x)/z if z != 0 else 0
            xzRatBig = float(x+1)/z if z != 0 else 0
            yzRatSmall = float(y)/z if z != 0 else 0
            yzRatBig = float(y+1)/z if z != 0 else 0

            for zz in range(int(z)+1, maxSize, step):
                cont = False
                for xx in range(int(xzRatSmall * zz), int(xzRatBig * zz)+step, step):
                    for yy in range(int(yzRatSmall * zz), int(yzRatBig * zz)+step, step):
                        coords = scaleOld([xx, yy, zz], mid)
                        if fits(0, maxSize, coords):
                            objGrid[int(coords[0]/step), int(coords[1]/step), int(coords[2]/step)] += 1
                            cont = True
                            if -10 < coords[0] < 500 and eq(flatGrid[int(coords[1]/step), int(coords[2]/step)], [0, 0, 0]):
                                flatGrid[int(coords[1]/step), int(coords[2]/step)] = [255, 0, 0]
                if not cont:
                    break

        for voxInd in range(len(cx)):
            unscCoords = unscale([cx[voxInd] * step, cy[voxInd] * step, cz[voxInd] * step], mid)

            x = unscCoords[0]
            y = unscCoords[1]
            z = unscCoords[2]

            if -10 < cx[voxInd] * step < 500 and eq(flatGrid[cy[voxInd], cz[voxInd]], [0, 0, 0]):
                flatGrid[cy[voxInd], cz[voxInd]] = [0, 255, 0]

            xzRatSmall = float(x) / z if z != 0 else 0
            xzRatBig = float(x + 1) / z if z != 0 else 0
            yzRatSmall = float(y) / z if z != 0 else 0
            yzRatBig = float(y + 1) / z if z != 0 else 0

            for zz in range(int(z) + 1, maxSize, step):
                cont = False
                for xx in range(int(xzRatSmall * zz), int(xzRatBig * zz)+step, step):
                    for yy in range(int(yzRatSmall * zz), int(yzRatBig * zz)+step, step):
                        coords = scaleOld([xx, yy, zz], mid)
                        if fits(0, maxSize, coords):
                            objGrid[int(coords[0] / step), int(coords[1] / step), int(coords[2] / step)] += 1
                            cont = True
                if not cont:
                    break
        #
        ox, oy, oz = np.where(np.logical_and(floorGrid == 0, np.logical_and(objGrid > 0, shellGrid <= thresh)))

        objects = mayavi.mlab.points3d(-oy, -oz, ox, mode="cube", scale_factor=0.8, color=(0.8, 0.9, 0.8), opacity=0.5)
        # nodes2 = mayavi.mlab.points3d(sx, sy, sz, mode="cube", scale_factor=0.8, color=(0, 1, 0))
        shell = mayavi.mlab.points3d(-by, -bz, bx, mode="cube", scale_factor=0.8, color=(0, 0, 1))
        floor = mayavi.mlab.points3d(-cy, -cz, cx, mode="cube", scale_factor=0.8, color=(1, 0, 0))
        # mayavi.mlab.outline(color=(0, 0, 0))

        shell.glyph.scale_mode = 'scale_by_vector'
        # shell.glyph.color_mode = "color_by_scalar"

        # shell.mlab_source.dataset.point_data.scalars = cols

        cv2.imshow("2d project", flatGrid)

        mayavi.mlab.show()

        # plt.show()

