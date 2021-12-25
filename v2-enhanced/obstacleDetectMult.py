import cv2
import numpy as np
import obstacleDetect


def mapVal(inSt, inEn, outSt, outEn, val):
    return (val-inSt)/(inEn-inSt) * (outEn-outSt) + outSt


if __name__ == "__main__":

    vDisps = []
    disps = []
    deps = []
    # rots = [0, np.deg2rad(90)]
    rots = []

    for i in range(0, 4+1):
        # Constructing test image
        imgId = "ROT-SPLR-" + str(i) + ".png"
        vDisp = cv2.imread("vDisp/vDisp" + imgId, cv2.IMREAD_UNCHANGED)
        disp = cv2.imread("dispImages/disp" + imgId, cv2.IMREAD_UNCHANGED)
        dep = cv2.imread("depImages/depth" + imgId, cv2.IMREAD_UNCHANGED)
        colour = cv2.imread("RImages/R" + imgId, cv2.IMREAD_COLOR)

        vDisps.append(vDisp)
        disps.append(disp)
        deps.append(dep)
        rots.append(np.deg2rad(mapVal(0, 4, -90, 90, i)))

    shellFlat, obsFlat, walkFlat = obstacleDetect.detectMult(vDisps, disps, deps, rots, True, True)

    cv2.imshow("shell", (shellFlat*255).astype(np.uint8))
    cv2.imshow("obs", (obsFlat*255).astype(np.uint8))
    cv2.imshow("walk", (walkFlat*255).astype(np.uint8))
    cv2.waitKey()
