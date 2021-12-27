import cv2
import numpy as np
import obstacleDetect
import time
import aStar


robotWidth = 112.6/2+120


def mapVal(inSt, inEn, outSt, outEn, val):
    return (val-inSt)/(inEn-inSt) * (outEn-outSt) + outSt


if __name__ == "__main__":

    vDisps = []
    disps = []
    deps = []
    # rots = [0, np.deg2rad(90)]
    rots = []

    for i in range(0, 3):
        # Constructing test image
        # imgId = "ROT-SPLR-" + str(i) + ".png"
        imgId = "16-" + str(i) + ".png"
        vDisp = cv2.imread("vDisp/vDisp" + imgId, cv2.IMREAD_UNCHANGED)
        disp = cv2.imread("dispImages/disp" + imgId, cv2.IMREAD_UNCHANGED)
        dep = cv2.imread("depImages/depth" + imgId, cv2.IMREAD_UNCHANGED)
        # colour = cv2.imread("RImages/R" + imgId, cv2.IMREAD_COLOR)

        vDisps.append(vDisp)
        disps.append(disp)
        deps.append(dep)
        rots.append(np.deg2rad(mapVal(0, 2, -45, 45, i)))

    shellFlat, obsFlat, walkFlat = obstacleDetect.detectMult(vDisps, disps, deps, rots, True, False)
    onPath, path, closestNode, voro, walkmap = \
        aStar.aStar(shellFlat, obsFlat, walkFlat, (-shellFlat.shape[0] - 1, shellFlat.shape[1] - 1), verbose=True,
                    distFunc=aStar.euclid, goalFunc=aStar.euclid, voroFunc=aStar.euclid, robotWidth=robotWidth,
                    ignoreDia=True, start=(int(shellFlat.shape[0] / 2), int(shellFlat.shape[1] / 2)))

    cv2.imshow("voro", (voro * 255/(400/50)).astype(np.uint8))
    cv2.imshow("path", (onPath * 255).astype(np.uint8))
    cv2.imshow("shell", (shellFlat*255).astype(np.uint8))
    cv2.imshow("obs", (obsFlat*255).astype(np.uint8))
    cv2.imshow("walk", (walkFlat*255).astype(np.uint8))
    cv2.waitKey()
