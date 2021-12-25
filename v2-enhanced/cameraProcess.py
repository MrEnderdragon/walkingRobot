import cv2
import numpy as np
import depthai as dai
import curves
import time
import UVdisp
import obstacleDetect
import aStar


def takeImage(q, lock, pipeline, camSleepTime, **args):
    """
    :param q: queue for images
    :param lock: lock for camera process
    :param pipeline: pipeline for camera
    :param camSleepTime:  time to sleep for camera
    :param args: flagWaitTime, focalLen, baseline, robotWidth
    :return:
    """

    focalLen = args["focalLen"] if "focalLen" in args else 441.25*31.35
    baseline = args["baseline"] if "baseline" in args else 7.5*10
    robotWidth = args["robotWidth"] if "robotWidth" in args else 112.6/2+120
    flagWaitTime = args["flagWaitTime"] if "flagWaitTime" in args else 1

    with dai.Device(pipeline) as device:
        # Output queue will be used to get the disparity frames from the outputs defined above
        depQ = device.getOutputQueue(name="depOut", maxSize=4, blocking=False)
        rQ = device.getOutputQueue(name="outR", maxSize=4, blocking=False)

        while True:
            lock.acquire()

            try:
                inDisp = depQ.get()  # blocking call, will wait until a new data has arrived
                inR = rQ.get()

                frameDep = inDisp.getFrame()
                frameR = inR.getCvFrame()

                curTime = int(time.time())
                f = open("images/latestTime.txt", "w")
                f.write(str(curTime))
                f.close()

                depCol = cv2.applyColorMap(cv2.convertScaleAbs(frameDep * 10, alpha=(255.0 / 65535.0)),
                                           cv2.COLORMAP_JET)
                cv2.imwrite("images/depth-" + str(curTime) + ".png", depCol)
                cv2.imwrite("images/col-" + str(curTime) + ".png", frameR)

                dispCalc = np.divide(focalLen*baseline, frameDep)

                cv2.imwrite("images/dispCalc-" + str(curTime) + ".png",
                            cv2.applyColorMap(cv2.convertScaleAbs(dispCalc * 10, alpha=(255.0 / 65535.0)), cv2.COLORMAP_JET))

                vDisp, uDisp = UVdisp.uvDisp(dispCalc)
                shellFlat, unknFlat, walkFlat = obstacleDetect.detect(vDisp, dispCalc, frameDep, verbose=True)

                onPath, path, closestNode, voro, walkmap = \
                    aStar.aStar(shellFlat, unknFlat, walkFlat, (0, shellFlat.shape[1] - 1), verbose=True,
                                distFunc=aStar.euclid, goalFunc=aStar.euclid, voroFunc=aStar.euclid, robotWidth=robotWidth)

                print("a* done")

                lastlast = None
                last = None

                points = [path[0]]
                curvedpath = np.zeros(onPath.shape, dtype=np.bool_)

                for ind, cur in enumerate(path):
                    curX = cur[1]*50
                    curY = -(cur[0]-shellFlat.shape[0]/2)*50
                    if lastlast is not None and last is not None:
                        if last[0] != (curX+lastlast[0])/2 or last[1] != (curY+lastlast[1])/2:
                            points.append(last)

                    lastlast = last
                    last = (curX, curY)

                print("points mapping done")

                newCurves = []

                if len(points) > 2:
                    enddX = (points[0][0] + points[1][0])/2
                    enddY = (points[0][1] + points[1][1])/2
                    curv = curves.quadBezier(points[0], ((enddX+points[0][0])/2, (enddY+points[0][1])/2), (enddX, enddY))
                    newCurves.append(curv)
                    for i in curv.renderPoints():
                        curvedpath[int(shellFlat.shape[0]/2-i[1]/50), int(i[0]/50)] = 1

                    for ind in range(1, len(points)-1):
                        sttX = (points[ind-1][0] + points[ind][0])/2
                        sttY = (points[ind-1][1] + points[ind][1])/2
                        enddX = (points[ind+1][0] + points[ind][0])/2
                        enddY = (points[ind+1][1] + points[ind][1])/2
                        curv = curves.quadBezier((sttX, sttY), points[ind], (enddX, enddY))
                        for i in curv.renderPoints():
                            curvedpath[int(shellFlat.shape[0]/2-i[1]/50), int(i[0]/50)] = 1
                        newCurves.append(curv)

                    sttX = (points[len(points)-2][0] + points[len(points)-1][0]) / 2
                    sttY = (points[len(points)-2][1] + points[len(points)-1][1]) / 2
                    curv = curves.quadBezier((sttX, sttY), ((sttX + points[len(points)-1][0]) / 2, (sttY + points[len(points)-1][1]) / 2),
                                             points[len(points)-1])
                    newCurves.append(curv)

                    for i in curv.renderPoints():
                        curvedpath[int(shellFlat.shape[0] / 2 - i[1] / 50), int(i[0] / 50)] = 1

                elif len(points) > 1:
                    curv = curves.quadBezier(points[0], ((points[1][0] + points[0][0]) / 2,
                                                         (points[1][1] + points[0][1]) / 2), points[1])
                    newCurves.append(curv)
                    for i in curv.renderPoints():
                        curvedpath[int(shellFlat.shape[0] / 2 - i[1] / 50), int(i[0] / 50)] = 1
                else:
                    pass
                    # break

                print("curve points done")

                cv2.imwrite("images/onpath-" + str(curTime) + ".png", onPath * 255)
                cv2.imwrite("images/curvePath-" + str(curTime) + ".png", curvedpath * 255)
                cv2.imwrite("images/voro-" + str(curTime) + ".png", voro * 50)
                cv2.imwrite("images/walkMap-" + str(curTime) + ".png", walkmap * 255)

                q.put(newCurves)

            except ValueError:
                print("ERRORED")

            lock.release()

            time.sleep(camSleepTime)
