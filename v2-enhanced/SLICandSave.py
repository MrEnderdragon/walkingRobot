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

        _, dispSLIC, dispScaled, floor = hough.hough(disp, vDisp, slicc=True, slicRef=colour, verbose=True, m=50, k=500, scl=1.5, its=5)

        vDisp = vDisp[0:360, ...]
        disp = disp[0:360, ...]
        dep = dep[0:360, ...]
        colour = colour[0:360, ...]
        floor = floor[0:360, ...]

        depFloorless = dep * floor

        depToUse = dep

        dispSLIC = cv2.applyColorMap(cv2.convertScaleAbs(dispSLIC*20, alpha=(255.0/65535.0)), cv2.COLORMAP_JET)

        cv2.imwrite("./SLIC/SLIC" + imgId, dispSLIC)
        cv2.imshow("./SLIC/SLIC", dispSLIC)
        cv2.waitKey()

