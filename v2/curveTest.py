import matplotlib.pyplot as plt
import matplotlib.widgets as mpw
import numpy as np
import math

#fig,ax = plt.subplots()
fig = plt.figure()
ax = plt.axes([0.05, 0.2, 0.9, 0.7])
scaleBox = plt.axes([0.2, 0.02, 0.6, 0.0375])
radiusBox = plt.axes([0.2, 0.0575, 0.6, 0.0375])

ax.grid(color='#aaa', linestyle='-', linewidth=1)

ax.set_xlim(0, 10)
ax.set_ylim(0, 10)

cursor, = ax.plot(5,5,'rx')

points = []
lineP = None
areaP = None
areaT = None
firstPoint = None
ppp = None
selected = None

pickedUp = False
pickedPoint = -1

fig.canvas.draw()


def update(text):
    ax.set_xlim(0, float(text))
    ax.set_ylim(0, float(text))

def onclick(event):

    global lineP, areaP, areaT, cursor, pickedUp, pickedPoint, ppp, firstPoint, selected

    if(event.y >= 150):

        if (pickedUp):
            pickedUp = False
            pickedPoint = -1
            selected.set_xdata(-1)
            selected.set_ydata(-1)

        elif(str(event.button) == "MouseButton.RIGHT"):
            if( not pickedUp):
                for i in range(0,len(points)):
                    curPoint = points[i]
                    distance = math.sqrt( ((curPoint[0]-event.xdata)**2)+((curPoint[1]-event.ydata)**2) )
                    if(distance < (abs(ax.get_xlim()[0]) + abs(ax.get_xlim()[1])) / 20):
                        pickedUp = True
                        pickedPoint = i
                        selected.set_xdata(curPoint[0])
                        selected.set_ydata(curPoint[1])
                        print(i)
                        break;

        else:
            print(event.button)

            points.insert(len(points),[event.xdata,event.ydata])

            if (len(points) == 1):
                # points.append([event.xdata,event.ydata])
                xs, ys = zip(*points)

                lineP, = ax.plot(xs,ys, 'bo', markersize=2)

                ppp, = ax.plot(xs,ys,'bo')

                #print(PolyArea(xs,ys))

                firstPoint, = ax.plot(points[0][0],points[0][1],'ro')

                selected, = ax.plot(-1,-1, markersize = 8, marker = 'o', markerfacecolor = "#7a00fc", markeredgecolor = "#7a00fc")

            #print(points)
            tmppp = generate(points)

            xs, ys = zip(*tmppp)
            xP, yP = zip(*points)
            ppp.set_xdata(xP)
            ppp.set_ydata(yP)

            lineP.set_xdata(xs)
            lineP.set_ydata(ys)

            fig.canvas.draw()
            fig.canvas.flush_events()

def onmove(event):
    global pickedUp, pickedPoint, firstPoint, selected

    if(event.y >= 150):

        cursor.set_ydata(event.ydata)
        cursor.set_xdata(event.xdata)

        if(pickedUp):
            if(pickedPoint == 0):

                firstPoint.set_xdata(event.xdata)
                firstPoint.set_ydata(event.ydata)

            # elif (pickedPoint == len(points)-1):
            #     points[0][0] = event.xdata
            #     points[0][1] = event.ydata

            points[pickedPoint][0] = event.xdata
            points[pickedPoint][1] = event.ydata

            tmppp = generate(points)

            xs, ys = zip(*tmppp)
            xP, yP = zip(*points)
            ppp.set_xdata(xP)
            ppp.set_ydata(yP)

            selected.set_xdata(event.xdata)
            selected.set_ydata(event.ydata)

            lineP.set_xdata(xs)
            lineP.set_ydata(ys)

        fig.canvas.draw()
        fig.canvas.flush_events()

def generateOld (points):
    tmppp = []

    for tt in range(0, 100):
        xTmp = 0
        yTmp = 0
        t = tt/100

        xTmp = xTmp + pow((1-t), len(points) - 1) * points[0][0]
        yTmp = yTmp + pow((1-t), len(points) - 1) * points[0][1]

        if(len(points) > 1):
            for pInd in range(1,len(points)-1):
                xTmp = xTmp + pow((1-t), len(points) - 1 - pInd) * pow(t, pInd) * points[pInd][0] * (len(points)-1)
                yTmp = yTmp + pow((1-t), len(points) - 1 - pInd) * pow(t, pInd) * points[pInd][1] * (len(points)-1)

            xTmp = xTmp + pow(t, len(points) - 1) * points[len(points)-1][0]
            yTmp = yTmp + pow(t, len(points) - 1) * points[len(points)-1][1]

        tmppp.append([xTmp, yTmp])

    return tmppp

def generateOld (points):
    tmppp = []

    for tt in range(0, 100):
        xTmp = 0
        yTmp = 0
        t = tt/100

        xTmp = xTmp + pow((1-t), len(points) - 1) * points[0][0]
        yTmp = yTmp + pow((1-t), len(points) - 1) * points[0][1]

        if(len(points) > 1):
            for pInd in range(1,len(points)-1):
                xTmp = xTmp + pow((1-t), len(points) - 1 - pInd) * pow(t, pInd) * points[pInd][0] * (len(points)-1)
                yTmp = yTmp + pow((1-t), len(points) - 1 - pInd) * pow(t, pInd) * points[pInd][1] * (len(points)-1)

            xTmp = xTmp + pow(t, len(points) - 1) * points[len(points)-1][0]
            yTmp = yTmp + pow(t, len(points) - 1) * points[len(points)-1][1]

        tmppp.append([xTmp, yTmp])

    return tmppp

def generate (points):
    tmppp = []

    for pInd in range(1,len(points)-2):

        # points[pInd] to (pInd, pInd+2) intersecting (pInd-1, pInd+1) to pInd+1

        #-1 to +1
        slope1 = (points[pInd+1][1]-points[pInd-1][1])/(points[pInd+1][0]-points[pInd-1][0])
        yInt1 = points[pInd][1] - slope1 * points[pInd][0]

        #0 to 2
        slope2 = (points[pInd+2][1]-points[pInd][1])/(points[pInd+2][0]-points[pInd][0])
        yInt2 = points[pInd+1][1] - slope2 * points[pInd+1][0]

        pointX = (yInt1 - yInt2) / (slope2 - slope1)
        pointY = pointX * slope1 + yInt1

        for tt in range(0, 100):
            t = tt/100

            xTmp = (1-t)**2*points[pInd][0] + 2*(1-t)*t*pointX + t**2*points[pInd+1][0]
            yTmp = (1-t)**2*points[pInd][1] + 2*(1-t)*t*pointY + t**2*points[pInd+1][1]

            tmppp.append([xTmp, yTmp])

    if(len(tmppp) < 2):
        tmppp.append([points[0][0], points[0][1]])
        tmppp.append([points[1][0], points[1][1]])


    return tmppp

def press(event):
    global pickedUp, pickedPoint
    if event.key == 'x' and pickedUp:
        del points[pickedPoint]
        if(pickedPoint == 0):
            firstPoint.set_xdata(points[0][0])
            firstPoint.set_ydata(points[0][1])
            points.append(points[0])
        pickedUp = False
        pickedPoint = -1

        xs, ys = zip(*points)
        ppp.set_xdata(xs)
        ppp.set_ydata(ys)

        selected.set_xdata(-1)
        selected.set_ydata(-1)

        lineP.set_xdata(xs)
        lineP.set_ydata(ys)

    fig.canvas.draw()
    fig.canvas.flush_events()


fig.canvas.mpl_connect('button_press_event', onclick)
fig.canvas.mpl_connect('motion_notify_event', onmove)
fig.canvas.mpl_connect('key_press_event', press)

text_box = mpw.TextBox(scaleBox, 'Scale (m):', initial="10")
text_box.on_submit(update)

rad_box = mpw.TextBox(radiusBox, 'Radius (m):', initial="2")

plt.show()
