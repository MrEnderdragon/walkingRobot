import matplotlib.pyplot as plt
import matplotlib.widgets as mpw
import numpy as np
import math
import curves
import time

# fig,ax = plt.subplots()
fig = plt.figure()
ax = plt.axes([0.05, 0.2, 0.9, 0.7])
scaleBox = plt.axes([0.2, 0.02, 0.6, 0.0375])
radiusBox = plt.axes([0.2, 0.0575, 0.6, 0.0375])

ax.grid(color='#aaa', linestyle='-', linewidth=1)

ax.set_xlim(-2000, 2000)
ax.set_ylim(-2000, 2000)

cursor, = ax.plot(5, 5, 'rx')

# points = []
points = [[-500.1, 501], [1, 1.01], [500.02, 500.03], [1.1, 1000.01], [-500.033, 500.022], [1.011, 1.11], [500.11, 500.12]]
points = [[-1500, -0.02], [0, -0.03], [500, 1], [1000, 2], [1500, 2.01], [2000, 2.02]]
lineP = None
areaP = None
areaT = None
firstPoint = None
ppp = None
selected = None

tLeft = None
tRight = None
bLeft = None
bRight = None

tLeftS = None
tRightS = None
bLeftS = None
bRightS = None
tLeftS2 = None
tRightS2 = None
bLeftS2 = None
bRightS2 = None
curS = None

pickedUp = False
pickedPoint = -1

fig.canvas.draw()


def update(text):
    ax.set_xlim(-float(text), float(text))
    ax.set_ylim(-float(text), float(text))


def onclick(event):
    global lineP, areaP, areaT, cursor, pickedUp, pickedPoint, ppp, firstPoint, selected, tLeft, tRight, bLeft, bRight, tLeftS, tRightS, bLeftS, bRightS, curS, tLeftS2, tRightS2, bLeftS2, bRightS2

    if event.y >= 150:

        if pickedUp:
            pickedUp = False
            pickedPoint = -1
            selected.set_xdata([])
            selected.set_ydata([])

        elif str(event.button) == "MouseButton.RIGHT":
            if not pickedUp:
                for i in range(0, len(points)):
                    curPoint = points[i]
                    distance = math.sqrt(((curPoint[0] - event.xdata) ** 2) + ((curPoint[1] - event.ydata) ** 2))
                    if distance < (abs(ax.get_xlim()[0]) + abs(ax.get_xlim()[1])) / 20:
                        pickedUp = True
                        pickedPoint = i
                        selected.set_xdata(curPoint[0])
                        selected.set_ydata(curPoint[1])
                        print(i)
                        break

        elif event.xdata != None and event.ydata != None:
            print(event.button)

            points.insert(len(points), [event.xdata, event.ydata])

            print(points)

            if ppp is None:
                # points.append([event.xdata,event.ydata])
                xs, ys = zip(*points)

                lineP, = ax.plot(xs, ys, 'ro', markersize=1)

                ppp, = ax.plot(xs, ys, 'bo')

                tLeft, = ax.plot(xs, ys, 'go', markersize=1)
                tRight, = ax.plot(xs, ys, 'mo', markersize=1)
                bLeft, = ax.plot(xs, ys, 'co', markersize=1)
                bRight, = ax.plot(xs, ys, 'ko', markersize=1)

                tLeftS, = ax.plot(xs, ys, 'go', markersize=1)
                tRightS, = ax.plot(xs, ys, 'mo', markersize=1)
                bLeftS, = ax.plot(xs, ys, 'co', markersize=1)
                bRightS, = ax.plot(xs, ys, 'ko', markersize=1)

                tLeftS2, = ax.plot(xs, ys, 'go', markersize=1)
                tRightS2, = ax.plot(xs, ys, 'mo', markersize=1)
                bLeftS2, = ax.plot(xs, ys, 'co', markersize=1)
                bRightS2, = ax.plot(xs, ys, 'ko', markersize=1)

                curS, = ax.plot(xs, ys, 'bo', markersize=1.4)

                # print(PolyArea(xs,ys))

                firstPoint, = ax.plot(points[0][0], points[0][1], 'ro')

                selected, = ax.plot([], [], markersize=8, marker='o', markerfacecolor="#7a00fc",
                                    markeredgecolor="#7a00fc")

            # print(points)
            render(points)

            fig.canvas.draw()
            fig.canvas.flush_events()


def onmove(event):
    global pickedUp, pickedPoint, firstPoint, selected

    if event.y >= 150:

        cursor.set_ydata(event.ydata)
        cursor.set_xdata(event.xdata)

        if pickedUp:
            if pickedPoint == 0:
                firstPoint.set_xdata(event.xdata)
                firstPoint.set_ydata(event.ydata)

            # elif (pickedPoint == len(points)-1):
            #     points[0][0] = event.xdata
            #     points[0][1] = event.ydata

            points[pickedPoint][0] = event.xdata
            points[pickedPoint][1] = event.ydata

            render(points)

        fig.canvas.draw()
        fig.canvas.flush_events()


def generate(points):
    tmppp = []
    rendTmppp = []
    dirTmppp = []

    curOver = 0

    for pInd in range(1, len(points) - 2):

        # points[pInd] to (pInd, pInd+2) intersecting (pInd-1, pInd+1) to pInd+1

        # -1 to +1
        slope1 = (points[pInd + 1][1] - points[pInd - 1][1]) / (points[pInd + 1][0] - points[pInd - 1][0])
        yInt1 = points[pInd][1] - slope1 * points[pInd][0]

        # 0 to 2
        slope2 = (points[pInd + 2][1] - points[pInd][1]) / (points[pInd + 2][0] - points[pInd][0])
        yInt2 = points[pInd + 1][1] - slope2 * points[pInd + 1][0]

        pointX = (yInt1 - yInt2) / (slope2 - slope1)
        pointY = pointX * slope1 + yInt1

        curv = curves.quadBezier((points[pInd][0], points[pInd][1]), (pointX, pointY),
                                 (points[pInd + 1][0], points[pInd + 1][1]))
        rendTmppp.append(points[pInd])
        rendTmppp.append([pointX, pointY])
        rendTmppp.append(points[pInd + 1])

        tmppp.extend(curv.renderPoints())

        spacing = 10

        pos, ang = curv.getPosDir(spacing, curOver)

        curOver = (curOver + curv.getLength()) % spacing
        # print(curOver)

        for i in range(len(pos)):
            dirTmppp.append(pos[i])
            dirTmppp.append(ang[i])
            # dirTmppp.append([pos[i][0] + math.cos(ang[i])/1, pos[i][1] + math.sin(ang[i])/1])
            # dirTmppp.append([pos[i][0] + math.sin(ang[i])/1, pos[i][1] - math.cos(ang[i])/1])

    if len(tmppp) < 2:
        tmppp.append([points[0][0], points[0][1]])
        tmppp.append([points[1][0], points[1][1]])
        rendTmppp.append([points[0][0], points[0][1]])
        rendTmppp.append([points[1][0], points[1][1]])

    rendTmppp.append(points[len(points) - 1])

    return tmppp, rendTmppp, dirTmppp


def dist(xy1, xy2):
    return math.sqrt((xy2[0] - xy1[0]) ** 2 + (xy2[1] - xy1[1]) ** 2)


def generateSteps(dPoints):
    maxDist = 200
    width = 110
    height = 183
    legDist = 150
    # leg max outwards reach
    outX = 120
    # leg max inwards reach
    inX = 50
    # legPos=[[0,0],[0,0],[0,0],[0,0]]
    legPos = [[height / 2 + outX, width / 2 + legDist],
              [height / 2 - inX + (outX + inX) * 1 / 3, -width / 2 - legDist],
              [-height / 2 + inX - (outX + inX) * 1 / 3, width / 2 + legDist],
              [-height / 2 - outX, -width / 2 - legDist]]  # initial positions of legs

    legPos = [[height / 2 + math.sqrt(maxDist ** 2 - legDist ** 2) / 2, width / 2 + legDist],
              [height / 2 - math.sqrt(maxDist ** 2 - legDist ** 2) * 1 / 6, -width / 2 - legDist],
              [-height / 2 + math.sqrt(maxDist ** 2 - legDist ** 2) * 1 / 6, width / 2 + legDist],
              [-height / 2 - math.sqrt(maxDist ** 2 - legDist ** 2) / 2,
               -width / 2 - legDist]]  # initial positions of legs

    res = [[], [], [], []]

    for i in range(0, len(dPoints), 2):
        # print(i)
        x = dPoints[i][0]
        y = dPoints[i][1]
        ang = dPoints[i + 1]

        r = math.sqrt((width / 2) ** 2 + (height / 2) ** 2)

        tmpAng = math.atan2(width / 2, height / 2)
        topLeft = [r * math.cos(tmpAng + ang) + x, r * math.sin(tmpAng + ang) + y]

        tmpAng = math.atan2(-width / 2, height / 2)
        topRight = [r * math.cos(tmpAng + ang) + x, r * math.sin(tmpAng + ang) + y]

        tmpAng = math.atan2(width / 2, -height / 2)
        botLeft = [r * math.cos(tmpAng + ang) + x, r * math.sin(tmpAng + ang) + y]

        tmpAng = math.atan2(-width / 2, -height / 2)
        botRight = [r * math.cos(tmpAng + ang) + x, r * math.sin(tmpAng + ang) + y]

        # print(str(dist(legPos[0], topLeft)) + "    (" + str(legPos[0]) + " and " + str(topLeft) + ")")
        # print(str(topLeft) + " " + str(topRight) + " " + str(botLeft) + " " + str(botRight))

        if dist(legPos[0], topLeft) > maxDist:
            # print("too long")
            for j in range(i, len(dPoints), 2):
                xx = dPoints[j][0]
                yy = dPoints[j][1]
                angg = dPoints[j + 1]

                poss = [xx - (width / 2 + legDist) * math.sin(angg),
                        yy + (width / 2 + legDist) * math.cos(angg)]

                # print("testing " + str(dist(poss, topLeft)))

                if dist(poss, topLeft) > maxDist:
                    xxx = dPoints[j - 2][0]
                    yyy = dPoints[j - 2][1]
                    anggg = dPoints[j - 2 + 1]

                    posss = [xxx - (width / 2 + legDist) * math.sin(anggg),
                             yyy + (width / 2 + legDist) * math.cos(anggg)]

                    legPos[0] = posss
                    res[0].append(posss)

                    # print("found, setting leg pos to: " + str(posss))

                    break
        elif dist(legPos[1], topRight) > maxDist:
            # print("too long")
            for j in range(i, len(dPoints), 2):
                xx = dPoints[j][0]
                yy = dPoints[j][1]
                angg = dPoints[j + 1]

                poss = [xx + (width / 2 + legDist) * math.sin(angg),
                        yy - (width / 2 + legDist) * math.cos(angg)]

                # print("testing " + str(dist(poss, topRight)))

                if dist(poss, topRight) > maxDist:
                    xxx = dPoints[j - 2][0]
                    yyy = dPoints[j - 2][1]
                    anggg = dPoints[j - 2 + 1]

                    posss = [xxx + (width / 2 + legDist) * math.sin(anggg),
                             yyy - (width / 2 + legDist) * math.cos(anggg)]

                    legPos[1] = posss
                    res[1].append(posss)

                    # print("found, setting leg pos to: " + str(posss))

                    break
        elif dist(legPos[2], botLeft) > maxDist:
            # print("too long")
            for j in range(i, len(dPoints), 2):
                xx = dPoints[j][0]
                yy = dPoints[j][1]
                angg = dPoints[j + 1]

                poss = [xx - (width / 2 + legDist) * math.sin(angg),
                        yy + (width / 2 + legDist) * math.cos(angg)]

                # print("testing " + str(dist(poss, topRight)))

                if dist(poss, botLeft) > maxDist:
                    xxx = dPoints[j - 2][0]
                    yyy = dPoints[j - 2][1]
                    anggg = dPoints[j - 2 + 1]

                    posss = [xxx - (width / 2 + legDist) * math.sin(anggg),
                             yyy + (width / 2 + legDist) * math.cos(anggg)]

                    legPos[2] = posss
                    res[2].append(posss)

                    # print("found, setting leg pos to: " + str(posss))

                    break
        elif dist(legPos[3], botRight) > maxDist:
            # print("too long")
            for j in range(i, len(dPoints), 2):
                xx = dPoints[j][0]
                yy = dPoints[j][1]
                angg = dPoints[j + 1]

                poss = [xx + (width / 2 + legDist) * math.sin(angg), yy - (width / 2 + legDist) * math.cos(angg)]

                # print("testing " + str(dist(poss, topRight)))

                if dist(poss, botRight) > maxDist:
                    xxx = dPoints[j - 2][0]
                    yyy = dPoints[j - 2][1]
                    anggg = dPoints[j - 2 + 1]

                    posss = [xxx + (width / 2 + legDist) * math.sin(anggg),
                             yyy - (width / 2 + legDist) * math.cos(anggg)]

                    legPos[3] = posss
                    res[3].append(posss)

                    # print("found, setting leg pos to: " + str(posss))

                    break

    return res


def generateStepsSlow(dPoints):
    maxDist = 200
    width = 110
    height = 183
    legDist = 150
    # leg max outwards reach
    outX = 120
    outDist = math.sqrt(outX ** 2 + maxDist ** 2)
    # leg max inwards reach
    inX = 50
    inDist = math.sqrt(inX ** 2 + maxDist ** 2)
    # legPos=[[0,0],[0,0],[0,0],[0,0]]
    legPos = [[height / 2 + outX, width / 2 + legDist],
              [height / 2 - inX + (outX + inX) * 1 / 3, -width / 2 - legDist],
              [-height / 2 + inX - (outX + inX) * 1 / 3, width / 2 + legDist],
              [-height / 2 - outX, -width / 2 - legDist]]  # initial positions of legs

    # legPos = [[height / 2 + math.sqrt(maxDist**2 - legDist**2)/2, width / 2 + legDist],
    #           [height / 2 + math.sqrt(maxDist**2 - legDist**2) * 1 / 6, -width / 2 - legDist],
    #           [-height / 2 - math.sqrt(maxDist**2 - legDist**2) * 1 / 6, width / 2 + legDist],
    #           [-height / 2 - math.sqrt(maxDist**2 - legDist**2)/2, -width / 2 - legDist]]  # initial positions of legs

    tLeftS.set_xdata([legPos[0][0]])
    tLeftS.set_ydata([legPos[0][1]])

    tRightS.set_xdata([legPos[1][0]])
    tRightS.set_ydata([legPos[1][1]])

    bLeftS.set_xdata([legPos[2][0]])
    bLeftS.set_ydata([legPos[2][1]])

    bRightS.set_xdata([legPos[3][0]])
    bRightS.set_ydata([legPos[3][1]])

    res = [[], [], [], []]

    for i in range(0, len(dPoints), 2):
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.01)
        # print(i)
        x = dPoints[i][0]
        y = dPoints[i][1]
        ang = dPoints[i + 1]

        curS.set_xdata([x])
        curS.set_ydata([y])

        r = math.sqrt((width / 2) ** 2 + (height / 2) ** 2)

        tmpAng = math.atan2(width / 2, height / 2)
        topLeft = [r * math.cos(tmpAng + ang) + x, r * math.sin(tmpAng + ang) + y]

        tmpAng = math.atan2(-width / 2, height / 2)
        topRight = [r * math.cos(tmpAng + ang) + x, r * math.sin(tmpAng + ang) + y]

        tmpAng = math.atan2(width / 2, -height / 2)
        botLeft = [r * math.cos(tmpAng + ang) + x, r * math.sin(tmpAng + ang) + y]

        tmpAng = math.atan2(-width / 2, -height / 2)
        botRight = [r * math.cos(tmpAng + ang) + x, r * math.sin(tmpAng + ang) + y]

        tLeftS2.set_xdata([topLeft[0]])
        tLeftS2.set_ydata([topLeft[1]])

        tRightS2.set_xdata([topRight[0]])
        tRightS2.set_ydata([topRight[1]])

        bLeftS2.set_xdata([botLeft[0]])
        bLeftS2.set_ydata([botLeft[1]])

        bRightS2.set_xdata([botRight[0]])
        bRightS2.set_ydata([botRight[1]])

        # print(str(dist(legPos[0], topLeft)) + "    (" + str(legPos[0]) + " and " + str(topLeft) + ")")
        # print(str(topLeft) + " " + str(topRight) + " " + str(botLeft) + " " + str(botRight))

        if dist(legPos[0], topLeft) > inDist:
            # print("too long")
            for j in range(i, len(dPoints), 2):
                xx = dPoints[j][0]
                yy = dPoints[j][1]
                angg = dPoints[j + 1]

                poss = [xx - (width / 2 + legDist) * math.sin(angg),
                        yy + (width / 2 + legDist) * math.cos(angg)]

                xxL = dPoints[j][0]
                yyL = dPoints[j][1]
                anggL = dPoints[j + 1]

                possL = [xxL - (width / 2 + legDist) * math.sin(anggL),
                         yyL + (width / 2 + legDist) * math.cos(anggL)]

                distt = dist(poss, topLeft)
                distL = dist(possL, topLeft)

                # print("testing " + str(dist(poss, topLeft)))

                if distt > outDist and distL < distt:
                    xxx = dPoints[j - 2][0]
                    yyy = dPoints[j - 2][1]
                    anggg = dPoints[j - 2 + 1]

                    posss = [xxx - (width / 2 + legDist) * math.sin(anggg),
                             yyy + (width / 2 + legDist) * math.cos(anggg)]

                    legPos[0] = posss
                    res[0].append(posss)

                    tLeftS.set_xdata([posss[0]])
                    tLeftS.set_ydata([posss[1]])

                    break
        elif dist(legPos[1], topRight) > inDist:
            # print("too long")
            for j in range(i, len(dPoints), 2):
                xx = dPoints[j][0]
                yy = dPoints[j][1]
                angg = dPoints[j + 1]

                poss = [xx + (width / 2 + legDist) * math.sin(angg),
                        yy - (width / 2 + legDist) * math.cos(angg)]

                # print("testing " + str(dist(poss, topRight)))

                if dist(poss, topRight) > outDist:
                    xxx = dPoints[j - 2][0]
                    yyy = dPoints[j - 2][1]
                    anggg = dPoints[j - 2 + 1]

                    posss = [xxx + (width / 2 + legDist) * math.sin(anggg),
                             yyy - (width / 2 + legDist) * math.cos(anggg)]

                    legPos[1] = posss
                    res[1].append(posss)

                    tRightS.set_xdata([posss[0]])
                    tRightS.set_ydata([posss[1]])

                    break
        elif dist(legPos[2], botLeft) > outDist:
            # print("too long")
            for j in range(i, len(dPoints), 2):
                xx = dPoints[j][0]
                yy = dPoints[j][1]
                angg = dPoints[j + 1]

                poss = [xx - (width / 2 + legDist) * math.sin(angg),
                        yy + (width / 2 + legDist) * math.cos(angg)]

                # print("testing " + str(dist(poss, topRight)))

                if dist(poss, botLeft) > inDist:
                    xxx = dPoints[j - 2][0]
                    yyy = dPoints[j - 2][1]
                    anggg = dPoints[j - 2 + 1]

                    posss = [xxx - (width / 2 + legDist) * math.sin(anggg),
                             yyy + (width / 2 + legDist) * math.cos(anggg)]

                    legPos[2] = posss
                    res[2].append(posss)

                    bLeftS.set_xdata([posss[0]])
                    bLeftS.set_ydata([posss[1]])

                    break
        elif dist(legPos[3], botRight) > outDist:
            # print("too long")
            for j in range(i, len(dPoints), 2):
                xx = dPoints[j][0]
                yy = dPoints[j][1]
                angg = dPoints[j + 1]

                poss = [xx + (width / 2 + legDist) * math.sin(angg), yy - (width / 2 + legDist) * math.cos(angg)]

                # print("testing " + str(dist(poss, topRight)))

                if dist(poss, botRight) > inDist:
                    xxx = dPoints[j - 2][0]
                    yyy = dPoints[j - 2][1]
                    anggg = dPoints[j - 2 + 1]

                    posss = [xxx + (width / 2 + legDist) * math.sin(anggg),
                             yyy - (width / 2 + legDist) * math.cos(anggg)]

                    legPos[3] = posss
                    res[3].append(posss)

                    bRightS.set_xdata([posss[0]])
                    bRightS.set_ydata([posss[1]])

                    break

    return res


def render(points):
    tmppp, rendPoints, dirPoints = generate(points)
    res = generateSteps(dirPoints)

    xs, ys = zip(*tmppp)
    xP, yP = zip(*rendPoints)
    ppp.set_xdata(xP)
    ppp.set_ydata(yP)

    # print(dirPoints)
    # print(len(dirPoints))

    # for i in range(0, len(dirPoints), 2):
    #     # print(i)
    #     xx = [dirPoints[i][0], dirPoints[i+1][0]]
    #     yy = [dirPoints[i][1], dirPoints[i+1][1]]
    #     ax.plot(xx,yy, 'go-', markersize = 1)

    lineP.set_xdata(xs)
    lineP.set_ydata(ys)

    tlX, tlY = zip(*res[0])
    tLeft.set_xdata(tlX)
    tLeft.set_ydata(tlY)

    trX, trY = zip(*res[1])
    tRight.set_xdata(trX)
    tRight.set_ydata(trY)

    blX, blY = zip(*res[2])
    bLeft.set_xdata(blX)
    bLeft.set_ydata(blY)

    brX, brY = zip(*res[3])
    bRight.set_xdata(brX)
    bRight.set_ydata(brY)


def renderSlow(no):
    tmppp, rendPoints, dirPoints = generate(points)

    xs, ys = zip(*tmppp)
    xP, yP = zip(*rendPoints)
    ppp.set_xdata([])
    ppp.set_ydata([])

    # print(dirPoints)
    # print(len(dirPoints))

    # for i in range(0, len(dirPoints), 2):
    #     # print(i)
    #     xx = [dirPoints[i][0], dirPoints[i+1][0]]
    #     yy = [dirPoints[i][1], dirPoints[i+1][1]]
    #     ax.plot(xx,yy, 'go-', markersize = 1)

    lineP.set_xdata(xs)
    lineP.set_ydata(ys)

    tLeft.set_xdata([])
    tLeft.set_ydata([])

    tRight.set_xdata([])
    tRight.set_ydata([])

    bLeft.set_xdata([])
    bLeft.set_ydata([])

    bRight.set_xdata([])
    bRight.set_ydata([])

    res = generateStepsSlow(dirPoints)


def press(event):
    global pickedUp, pickedPoint
    if event.key == 'x' and pickedUp:
        del points[pickedPoint]
        if pickedPoint == 0:
            firstPoint.set_xdata(points[0][0])
            firstPoint.set_ydata(points[0][1])
            points.append(points[0])
        pickedUp = False
        pickedPoint = -1

        render(points)

    fig.canvas.draw()
    fig.canvas.flush_events()


if __name__ == '__main__':
    fig.canvas.mpl_connect('button_press_event', onclick)
    fig.canvas.mpl_connect('motion_notify_event', onmove)
    fig.canvas.mpl_connect('key_press_event', press)

    text_box = mpw.TextBox(scaleBox, 'Scale (m):', initial="100")
    text_box.on_submit(update)

    rad_box = mpw.TextBox(radiusBox, 'Radius (m):', initial="2")
    rad_box.on_submit(renderSlow)

    plt.show()
