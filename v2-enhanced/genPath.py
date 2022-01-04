import numpy as np
import curves
import rdp
import math
import log


def gen_path(onPath):
    """
    :param onPath: image for points on the path
    :return: newCurves, curvedpath
    """

    # width = onPath.shape[0]
    # height = onPath.shape[1]
    # contours, _ = cv2.findContours(np.flip(onPath, axis=0), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
    #
    # contour = contours[0]
    #
    # # tmp = ""
    #
    # points = []
    # for ind in range(len(contour)):
    #     if ind > 2:
    #         if (contour[ind][0][0] == contour[ind-2][0][0]) and (contour[ind][0][1] == contour[ind-2][0][1]):
    #             break
    #     points.append(((contour[ind][0][0] - width/2) * 50, (contour[ind][0][1] - height/2) * 50))
    # tmp += "(" + str(contour[ind][0][0]) + "," + str(contour[ind][0][1]) + "),"

    # log.log(points)
    # log.log(tmp)

    tmp = ""

    width = 120
    height = 120

    mappedPoints = []

    for cur in onPath:
        curX = (cur[1] - height / 2) * 50
        curY = -(cur[0] - width / 2) * 50

        mappedPoints.append((curX, curY))

    points = rdp.rdp(mappedPoints, 100.0)

    for ind in range(len(points)):
        tmp += "(" + str(points[ind][0]) + "," + str(points[ind][1]) + "),"
    
    log.log(tmp)
    
    newCurves = []
    curvedpath = np.zeros((width, height), dtype=np.bool_)
    
    if len(points) > 2:
        enddX = (points[0][0] + points[1][0]) / 2
        enddY = (points[0][1] + points[1][1]) / 2
        curv = curves.quadBezier(points[0], ((enddX + points[0][0]) / 2, (enddY + points[0][1]) / 2), (enddX, enddY))
        newCurves.append(curv)

        for i in curv.renderPoints():
            if int(width / 2 - i[1] / 50) < width and int(i[0] / 50 - height / 2) < height:
                curvedpath[int(width / 2 - i[1] / 50), int(i[0] / 50 - height / 2)] = 1

        for ind in range(1, len(points) - 1):
            sttX = (points[ind - 1][0] + points[ind][0]) / 2
            sttY = (points[ind - 1][1] + points[ind][1]) / 2
            enddX = (points[ind + 1][0] + points[ind][0]) / 2
            enddY = (points[ind + 1][1] + points[ind][1]) / 2
            curv = curves.quadBezier((sttX, sttY), points[ind], (enddX, enddY))
            for i in curv.renderPoints():
                if int(width / 2 - i[1] / 50) < width and int(i[0] / 50 - height / 2) < height:
                    curvedpath[int(width / 2 - i[1] / 50), int(i[0] / 50 - height / 2)] = 1
            newCurves.append(curv)

        sttX = (points[len(points) - 2][0] + points[len(points) - 1][0]) / 2
        sttY = (points[len(points) - 2][1] + points[len(points) - 1][1]) / 2
        curv = curves.quadBezier((sttX, sttY),
                                 ((sttX + points[len(points) - 1][0]) / 2, (sttY + points[len(points) - 1][1]) / 2),
                                 points[len(points) - 1])
        newCurves.append(curv)

        for i in curv.renderPoints():
            if int(width / 2 - i[1] / 50) < width and int(i[0] / 50 - height / 2) < height:
                curvedpath[int(width / 2 - i[1] / 50), int(i[0] / 50 - height / 2)] = 1

    elif len(points) > 1:
        curv = curves.quadBezier(points[0], ((points[1][0] + points[0][0]) / 2,
                                             (points[1][1] + points[0][1]) / 2), points[1])
        newCurves.append(curv)
        for i in curv.renderPoints():
            if int(width / 2 - i[1] / 50) < width and int(i[0] / 50 - height / 2) < height:
                curvedpath[int(width / 2 - i[1] / 50), int(i[0] / 50 - height / 2)] = 1
    else:
        pass

    return newCurves, curvedpath, points


def generate1(linePoints, **args):
    """
    :param linePoints:
    :param args: driveAcc: accuracy for curves (mm)
    :return:
    """

    driveAcc = args["driveAcc"] if "driveAcc" in args else 10  # accuracy of driving for curves (mm)

    dirTmp = []
    
    if len(linePoints) < 2:
        return dirTmp

    prevP = linePoints[0]
    for pInd in range(1, len(linePoints)):
        curP = linePoints[pInd]

        angle = math.atan2((curP[1] - prevP[1]), (curP[0] - prevP[0]))
        dirTmp.append(prevP)
        dirTmp.append(angle)
        
        size = math.sqrt((curP[1] - prevP[1])**2 + (curP[0] - prevP[0])**2)
        
        xinc = (curP[0] - prevP[0]) * driveAcc / size
        yinc = (curP[1] - prevP[1]) * driveAcc / size
        for ind in range(int(size/driveAcc)):
            dirTmp.append((prevP[0] + ind * xinc, prevP[1] + ind * yinc))
            dirTmp.append(angle)
        
        prevP = curP
    return dirTmp


def generate(driveCurves, **args):
    """
    :param driveCurves:
    :param args: driveAcc: accuracy for curves (mm)
    :return:
    """

    driveAcc = args["driveAcc"] if "driveAcc" in args else 10  # accuracy of driving for curves (mm)

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
