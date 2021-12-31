import cv2
import numpy as np
import obstacleDetect
import aStar
# import time
# import curves
import genPath
import time


robotWidth = 112.6/2+120


def renderImgCoord(inGrid):
    return np.flip(inGrid, axis=0)


def mapVal(inSt, inEn, outSt, outEn, val):
    return (val-inSt)/(inEn-inSt) * (outEn-outSt) + outSt


def processImages(verbose=False):
    vDisps = []
    disps = []
    deps = []
    # rots = [0, np.deg2rad(90)]
    rots = []

    st = 9
    en = 11

    for i in range(st, en+1):
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
        rots.append(np.deg2rad(mapVal(st, en, -45, 45, i)))

    shellFlat, obsFlat, walkFlat = obstacleDetect.detectMult(vDisps, disps, deps, rots, True, False)
    onPath, path, closestNode, voro, walkmap = \
        aStar.aStar(shellFlat, obsFlat, walkFlat, (shellFlat.shape[0]/2, shellFlat.shape[1] - 1), verbose=True,
                    distFunc=aStar.euclid, goalFunc=aStar.euclid, voroFunc=aStar.euclid, robotWidth=robotWidth,
                    ignoreDia=False, start=(int(shellFlat.shape[0] / 2), int(shellFlat.shape[1] / 2)))

    # newCurves, curvedpath = genPath.gen_path((onPath * 255).astype(np.uint8))
    newCurves, curvedpath = genPath.gen_path(path)

    tmp = ""
    
    for i in newCurves:
        for p in i.renderPoints():
            tmp += "(" + str(p[0]) + "," + str(p[1]) + "),"
    
    print(tmp)
    
    if verbose:
        cv2.imshow("voro", (voro * 255/(400/50)).astype(np.uint8))
        cv2.imshow("path", (onPath * 255).astype(np.uint8))
        cv2.imshow("shell", (shellFlat*255).astype(np.uint8))
        cv2.imshow("obs", (obsFlat*255).astype(np.uint8))
        cv2.imshow("walk", (walkFlat*255).astype(np.uint8))
        cv2.imshow("curvedpath", (curvedpath * 255).astype(np.uint8))
        cv2.waitKey()

    return newCurves


def takeImage(q, lock, pipeline, camSleepTime, **args):
    while True:
        newCurves = processImages(False)
        q.put(newCurves)
        time.sleep(camSleepTime)
        


if __name__ == "__main__":    
    processImages(True)

