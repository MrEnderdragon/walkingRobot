import cv2
import numpy as np
import curves
    
driveAcc = 10  # accuracy of driving for curves (mm)
    
def gen_path(onPath):
    
    width = onPath.shape[0]
    height = onPath.shape[1]
    contours, hierarchy = cv2.findContours(onPath, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
    
    contour = contours[0]
    
    points = []
    for ind in range(len(contour)):
        if ind > 2:
            if (contour[ind][0][0] == contour[ind-2][0][0]) and (contour[ind][0][1] == contour[ind-2][0][1]):
                break
        points.append( ((contour[ind][0][0] - width/2)*50, (contour[ind][0][1] - height/2)*50))

    newCurves = []
    curvedpath = np.zeros(onPath.shape, dtype=np.bool_)

    if len(points) > 2:
        enddX = (points[0][0] + points[1][0]) / 2
        enddY = (points[0][1] + points[1][1]) / 2
        curv = curves.quadBezier(points[0], ((enddX + points[0][0]) / 2, (enddY + points[0][1]) / 2), (enddX, enddY))
        newCurves.append(curv)

        for i in curv.renderPoints():
            curvedpath[int(width / 2 - i[1] / 50), int(i[0] / 50 - height / 2)] = 1

        for ind in range(1, len(points) - 1):
            sttX = (points[ind - 1][0] + points[ind][0]) / 2
            sttY = (points[ind - 1][1] + points[ind][1]) / 2
            enddX = (points[ind + 1][0] + points[ind][0]) / 2
            enddY = (points[ind + 1][1] + points[ind][1]) / 2
            curv = curves.quadBezier((sttX, sttY), points[ind], (enddX, enddY))
            for i in curv.renderPoints():
                curvedpath[int(width / 2 - i[1] / 50), int(i[0] / 50 - height / 2)] = 1
            newCurves.append(curv)

        sttX = (points[len(points) - 2][0] + points[len(points) - 1][0]) / 2
        sttY = (points[len(points) - 2][1] + points[len(points) - 1][1]) / 2
        curv = curves.quadBezier((sttX, sttY),
                                 ((sttX + points[len(points) - 1][0]) / 2, (sttY + points[len(points) - 1][1]) / 2),
                                 points[len(points) - 1])
        newCurves.append(curv)

        for i in curv.renderPoints():
            curvedpath[int(width / 2 - i[1] / 50), int(i[0] / 50 - height/ 2)] = 1

    elif len(points) > 1:
        curv = curves.quadBezier(points[0], ((points[1][0] + points[0][0]) / 2,
                                             (points[1][1] + points[0][1]) / 2), points[1])
        newCurves.append(curv)
        for i in curv.renderPoints():
            curvedpath[int(width / 2 - i[1] / 50), int(i[0] / 50 - height/ 2)] = 1
    else:
        pass
        # break

    cv2.imshow("curvedpath", (curvedpath * 255).astype(np.uint8))
    print("curve points done")
    return newCurves


def generate(driveCurves):
    dirTmp = []
    curOver = 0

    for pInd in range(len(driveCurves)):

        curve = driveCurves[pInd]
        pos, ang = curve.getPosDir(driveAcc, curOver)
        curOver = (curOver + curve.getLength()) % driveAcc

        for j in range(len(pos)):
            dirTmp.append(pos[j])
            dirTmp.append(ang[j])

    return dirTmp

