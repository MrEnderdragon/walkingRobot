import cv2
import numpy as np
import obstacleDetect
import aStar
# import time
import curves


robotWidth = 112.6/2+120


def renderImgCoord(inGrid):
    return np.flip(inGrid, axis=0)


def mapVal(inSt, inEn, outSt, outEn, val):
    return (val-inSt)/(inEn-inSt) * (outEn-outSt) + outSt


if __name__ == "__main__":

    vDisps = []
    disps = []
    deps = []
    # rots = [0, np.deg2rad(90)]
    rots = []

    st = 6
    en = 8

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

    lastlast = None
    last = None

    points = []
    curvedpath = np.zeros(onPath.shape, dtype=np.bool_)

    for ind, cur in enumerate(path):
        curX = (cur[1] - shellFlat.shape[1] / 2) * 50
        curY = -(cur[0] - shellFlat.shape[0] / 2) * 50

        if ind == 0:
            points.append((curX, curY))

        if lastlast is not None and last is not None:
            if last[0] != (curX + lastlast[0]) / 2 or last[1] != (curY + lastlast[1]) / 2:
                points.append(last)

        lastlast = last
        last = (curX, curY)

    if last is not None:
        points.append(last)

    newCurves = []

    if len(points) > 2:
        enddX = (points[0][0] + points[1][0]) / 2
        enddY = (points[0][1] + points[1][1]) / 2
        curv = curves.quadBezier(points[0], ((enddX + points[0][0]) / 2, (enddY + points[0][1]) / 2),
                                 (enddX, enddY))
        newCurves.append(curv)

        for i in curv.renderPoints():
            curvedpath[int(shellFlat.shape[0] / 2 - i[1] / 50), int(i[0] / 50 - shellFlat.shape[1] / 2)] = 1

        for ind in range(1, len(points) - 1):
            sttX = (points[ind - 1][0] + points[ind][0]) / 2
            sttY = (points[ind - 1][1] + points[ind][1]) / 2
            enddX = (points[ind + 1][0] + points[ind][0]) / 2
            enddY = (points[ind + 1][1] + points[ind][1]) / 2
            curv = curves.quadBezier((sttX, sttY), points[ind], (enddX, enddY))
            for i in curv.renderPoints():
                curvedpath[
                    int(shellFlat.shape[0] / 2 - i[1] / 50), int(i[0] / 50 - shellFlat.shape[1] / 2)] = 1
            newCurves.append(curv)

        sttX = (points[len(points) - 2][0] + points[len(points) - 1][0]) / 2
        sttY = (points[len(points) - 2][1] + points[len(points) - 1][1]) / 2
        curv = curves.quadBezier((sttX, sttY),
                                 ((sttX + points[len(points) - 1][0]) / 2,
                                  (sttY + points[len(points) - 1][1]) / 2),
                                 points[len(points) - 1])
        newCurves.append(curv)

        for i in curv.renderPoints():
            curvedpath[int(shellFlat.shape[0] / 2 - i[1] / 50), int(i[0] / 50 - shellFlat.shape[1] / 2)] = 1

    elif len(points) > 1:
        curv = curves.quadBezier(points[0], ((points[1][0] + points[0][0]) / 2,
                                             (points[1][1] + points[0][1]) / 2), points[1])
        newCurves.append(curv)
        for i in curv.renderPoints():
            curvedpath[int(shellFlat.shape[0] / 2 - i[1] / 50), int(i[0] / 50 - shellFlat.shape[1] / 2)] = 1
    else:
        pass

    print([[path[i][0], path[i][1]] for i in range(len(path))])
    points2 = [(points[i][0], points[i][1]) for i in range(len(points))]
    print(str(points2).replace("[", "(").replace("]", ")"))

    cv2.imshow("voro", (voro * 255/(400/50)).astype(np.uint8))
    cv2.imshow("path", (onPath * 255).astype(np.uint8))
    cv2.imshow("shell", (shellFlat*255).astype(np.uint8))
    cv2.imshow("obs", (obsFlat*255).astype(np.uint8))
    cv2.imshow("walk", (walkFlat*255).astype(np.uint8))
    cv2.waitKey()
