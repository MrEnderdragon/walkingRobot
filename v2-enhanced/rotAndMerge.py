run = True

import cv2
import depthai as dai
import numpy as np
import UVdisp
import obstacleDetect
import time
import aStar
import curves

if run:
    import RPi.GPIO as GPIO

    servoPIN = 17
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(servoPIN, GPIO.OUT)

    p = GPIO.PWM(servoPIN, 50)  # GPIO 17 for PWM with 50Hz
    p.start(0)


def setAngle(angle):
    duty = angle / 18 + 2
    p.ChangeDutyCycle(duty)
    time.sleep(1)
    p.ChangeDutyCycle(0)


# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
extended_disparity = False  # doesn't work
# Better accuracy for longer distance, fractional disparity 32-levels:
subpixel = True
# Better handling for occlusions:
lr_check = True

imgid = 'ROT-' + ('SP' if subpixel else '') + ('LR' if lr_check else '')

focalLen = 441.25*31.35
baseline = 7.5*10
robotWidth = 112.6/2+120
# focalLen = 2000

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()

disp = pipeline.createStereoDepth()

outDisp = pipeline.createXLinkOut()
outDisp.setStreamName("dispOut")

outL = pipeline.createXLinkOut()
outL.setStreamName("outL")
outR = pipeline.createXLinkOut()
outR.setStreamName("outR")

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
disp.initialConfig.setConfidenceThreshold(200)
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
disp.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
disp.setLeftRightCheck(subpixel)
disp.setExtendedDisparity(extended_disparity)
disp.setSubpixel(lr_check)

# Linking
monoLeft.out.link(disp.left)
monoRight.out.link(disp.right)

monoLeft.out.link(outL.input)
monoRight.out.link(outR.input)

disp.depth.link(outDisp.input)


def mapVal(inSt, inEn, outSt, outEn, val):
    return (val-inSt)/(inEn-inSt) * (outEn-outSt) + outSt


def renderImgCoord(inGrid):
    # return np.flip(inGrid, axis=0)
    return inGrid


amRot = 3


if __name__ == "__main__":
    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:

        # Output queue will be used to get the disparity frames from the outputs defined above
        q = device.getOutputQueue(name="dispOut", maxSize=4, blocking=False)
        qL = device.getOutputQueue(name="outL", maxSize=4, blocking=False)
        qR = device.getOutputQueue(name="outR", maxSize=4, blocking=False)

        while True:

            vDisps = []
            deps = []
            disps = []
            rots = []

            q.get()  # blocking call, will wait until a new data has arrived
            qL.get()
            qR.get()

            time.sleep(1)

            q.get()  # blocking call, will wait until a new data has arrived
            qL.get()
            qR.get()

            time.sleep(1)

            curTime = int(time.time())
            f = open("images/latestTime.txt", "w")
            f.write(str(curTime))
            f.close()

            for i in range(1, amRot+1):

                print("rot start")

                rot = mapVal(1, amRot, -45, 45, i)

                if run:
                    setAngle(rot+90)

                time.sleep(1)

                inDisp = q.get()  # blocking call, will wait until a new data has arrived

                inL = qL.get()
                inR = qR.get()

                frame = inDisp.getFrame()

                frameL = inL.getCvFrame()
                frameR = inR.getCvFrame()

                # Normalization for better visualization
                frameDep = frame

                curTime = int(time.time())
                f = open("images/latestTime.txt", "w")
                f.write(str(curTime))
                f.close()

                depCol = cv2.applyColorMap(cv2.convertScaleAbs(frameDep * 10, alpha=(255.0 / 65535.0)),
                                           cv2.COLORMAP_JET)
                cv2.imwrite("images/depth-" + str(curTime) + ".png", depCol)
                cv2.imwrite("images/depth16-" + str(curTime) + ".png", frame.astype(np.uint16))
                cv2.imwrite("images/col-" + str(curTime) + ".png", frameR)

                frameDispCalc = np.divide(focalLen * baseline, frameDep)

                cv2.imwrite("images/dispCalc-" + str(curTime) + ".png",
                            cv2.applyColorMap(cv2.convertScaleAbs(frameDispCalc * 10, alpha=(255.0 / 65535.0)),
                                              cv2.COLORMAP_JET))

                cv2.imwrite("images/disp16-" + str(curTime) + ".png", frameDispCalc.astype(np.uint16))

                vDisp, uDisp = UVdisp.uvDisp(frameDispCalc.astype(np.uint16))

                cv2.imwrite("images/vDisp-" + str(curTime) + ".png", vDisp*10)
                cv2.imwrite("images/vDisp16-" + str(curTime) + ".png", vDisp)

                # cv2.imshow("depth", frameDep)
                # cv2.imshow("dispCalc", cv2.applyColorMap(cv2.convertScaleAbs(frameDispCalc*10, alpha=(255.0/65535.0)), cv2.COLORMAP_TWILIGHT))
                # cv2.imshow("v", vDisp*10)

                vDisps.append(vDisp)
                disps.append(frameDispCalc)
                deps.append(frame)
                rots.append(np.deg2rad(rot))

            shellFlat, obsFlat, walkFlat = obstacleDetect.detectMult(vDisps, disps, deps, rots, True, False)

            startCoords = (int(shellFlat.shape[0]/2), int(shellFlat.shape[1]/2))

            onPath, path, closestNode, voro, walkmap = \
                aStar.aStar(shellFlat, obsFlat, walkFlat, (shellFlat.shape[0] - 1, shellFlat.shape[1] - 1), verbose=True,
                            distFunc=aStar.euclid, goalFunc=aStar.euclid, voroFunc=aStar.euclid, robotWidth=robotWidth,
                            ignoreDia=True, diaWeight=100, start=startCoords)

            print("a* done")

            lastlast = None
            last = None

            points = [path[0]]
            curvedpath = np.zeros(onPath.shape, dtype=np.bool_)

            for ind, cur in enumerate(path):
                curX = (cur[1] - shellFlat.shape[1] / 2) * 50
                curY = -(cur[0] - shellFlat.shape[0] / 2) * 50
                if lastlast is not None and last is not None:
                    if last[0] != (curX + lastlast[0]) / 2 or last[1] != (curY + lastlast[1]) / 2:
                        points.append(last)

                lastlast = last
                last = (curX, curY)

            print("points mapping done")

            newCurves = []

            if len(points) > 2:
                enddX = (points[0][0] + points[1][0]) / 2
                enddY = (points[0][1] + points[1][1]) / 2
                curv = curves.quadBezier(points[0], ((enddX + points[0][0]) / 2, (enddY + points[0][1]) / 2), (enddX, enddY))
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
                        curvedpath[int(shellFlat.shape[0] / 2 - i[1] / 50), int(i[0] / 50 - shellFlat.shape[1] / 2)] = 1
                    newCurves.append(curv)

                sttX = (points[len(points) - 2][0] + points[len(points) - 1][0]) / 2
                sttY = (points[len(points) - 2][1] + points[len(points) - 1][1]) / 2
                curv = curves.quadBezier((sttX, sttY),
                                         ((sttX + points[len(points) - 1][0]) / 2, (sttY + points[len(points) - 1][1]) / 2),
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
                # break

            print("curve points done")

            cv2.imwrite("images/onpath-" + str(curTime) + ".png", renderImgCoord(onPath) * 255)
            cv2.imwrite("images/curvePath-" + str(curTime) + ".png", renderImgCoord(curvedpath) * 255)
            cv2.imwrite("images/voro-" + str(curTime) + ".png", renderImgCoord(voro) * 50)
            cv2.imwrite("images/walkMap-" + str(curTime) + ".png", renderImgCoord(walkmap) * 255)
            cv2.imwrite("images/shell-" + str(curTime) + ".png", renderImgCoord(shellFlat.astype(np.uint8)) * 255)
            cv2.imwrite("images/obs-" + str(curTime) + ".png", renderImgCoord(obsFlat.astype(np.uint8)) * 255)

            if input() == "q":
                if run:
                    GPIO.cleanup()
                break

