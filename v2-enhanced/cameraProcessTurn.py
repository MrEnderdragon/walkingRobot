import cv2
import numpy as np
import depthai as dai
import curves
import time
import UVdisp
import obstacleDetect
import aStar
import genPath
import RPi.GPIO as GPIO
import traceback


def mapVal(inSt, inEn, outSt, outEn, val):
    return (val-inSt)/(inEn-inSt) * (outEn-outSt) + outSt


def renderImgCoord(inGrid):
    # return np.flip(inGrid, axis=0)
    return inGrid


def setAngle(angle, p):
    duty = angle / 18 + 2
    p.ChangeDutyCycle(duty)
    time.sleep(1)
    p.ChangeDutyCycle(0)


def takeImage(q, lock, camLock, pipeline, camSleep, **args):
    """
    :param q: queue for images
    :param lock: lock for camera process
    :param camLock: lock for starting camera
    :param pipeline: pipeline for camera
    :param camSleep: time to sleep between camera lock checks
    :param args: flagWaitTime, focalLen, baseline, robotWidth, amRot
    :return:
    """

    focalLen = args["focalLen"] if "focalLen" in args else 441.25*31.35
    baseline = args["baseline"] if "baseline" in args else 7.5*10
    robotWidth = args["robotWidth"] if "robotWidth" in args else 112.6/2+120
    # flagWaitTime = args["flagWaitTime"] if "flagWaitTime" in args else 1
    amRot = args["amRot"] if "amRot" in args else 3

    with dai.Device(pipeline) as device:
        # Output queue will be used to get the disparity frames from the outputs defined above
        depQ = device.getOutputQueue(name="depOut", maxSize=4, blocking=False)
        rQ = device.getOutputQueue(name="outR", maxSize=4, blocking=False)

        servoPIN = 17
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(servoPIN, GPIO.OUT)

        p = GPIO.PWM(servoPIN, 50)  # GPIO 17 for PWM with 50Hz
        p.start(0)

        while True:
            camLock.acquire()
            lock.acquire()
            print("cam start")

            try:
                vDisps = []
                deps = []
                disps = []
                rots = []

                depQ.get()  # blocking call, will wait until a new data has arrived
                rQ.get()

                time.sleep(1)

                depQ.get()  # blocking call, will wait until a new data has arrived
                rQ.get()

                time.sleep(1)

                curTime = int(time.time())
                f = open("images/latestTime.txt", "w")
                f.write(str(curTime))
                f.close()

                for i in range(1, amRot + 1):

                    print("rot start")

                    rot = mapVal(1, amRot, -45, 45, i)

                    setAngle(rot + 90, p)

                    time.sleep(1)

                    inDisp = depQ.get()  # blocking call, will wait until a new data has arrived

                    inR = rQ.get()

                    frame = inDisp.getFrame()

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

                    cv2.imwrite("images/vDisp-" + str(curTime) + ".png", vDisp * 10)
                    cv2.imwrite("images/vDisp16-" + str(curTime) + ".png", vDisp)

                    # cv2.imshow("depth", frameDep)
                    # cv2.imshow("dispCalc", cv2.applyColorMap(cv2.convertScaleAbs(frameDispCalc*10, alpha=(255.0/65535.0)), cv2.COLORMAP_TWILIGHT))
                    # cv2.imshow("v", vDisp*10)

                    vDisps.append(vDisp)
                    disps.append(frameDispCalc)
                    deps.append(frame)
                    rots.append(np.deg2rad(rot))

                setAngle(90, p)

                shellFlat, obsFlat, walkFlat, unwalkCoords, valid = obstacleDetect.detectMult(vDisps, disps, deps, rots, True, False)

                startCoords = (int(shellFlat.shape[0] / 2), int(shellFlat.shape[1] / 2))

                onPath, path, closestNode, voro, walkmap = \
                    aStar.aStar(shellFlat, obsFlat, walkFlat, unwalkCoords, None,
                                verbose=True,
                                distFunc=aStar.euclid, goalFunc=aStar.euclid, voroFunc=aStar.euclid,
                                robotWidth=robotWidth,
                                ignoreDia=False, diaWeight=100, start=startCoords)

                # onPath, path, closestNode, voro, walkmap = \
                #     aStar.aStar(shellFlat, obsFlat, walkFlat, unwalkCoords, (shellFlat.shape[0] - 1, shellFlat.shape[1] - 1),
                #                 verbose=True,
                #                 distFunc=aStar.euclid, goalFunc=aStar.euclid, voroFunc=aStar.euclid,
                #                 robotWidth=robotWidth,
                #                 ignoreDia=False, diaWeight=100, start=startCoords)

                # aStar.aStar(shellFlat, obsFlat, walkFlat, (0, shellFlat.shape[1] - 1),

                print("a* done")

                # newCurves, curvedpath = genPath.gen_path((onPath * 255).astype(np.uint8))
                newCurves, curvedpath = genPath.gen_path(path)

                print("curve points done")

                cv2.imwrite("images/onpath-" + str(curTime) + ".png", renderImgCoord(onPath) * 255)
                cv2.imwrite("images/curvePath-" + str(curTime) + ".png", renderImgCoord(curvedpath) * 255)
                cv2.imwrite("images/voro-" + str(curTime) + ".png", renderImgCoord(voro) * 50)
                cv2.imwrite("images/walkMap-" + str(curTime) + ".png", renderImgCoord(walkmap) * 255)
                cv2.imwrite("images/shell-" + str(curTime) + ".png", renderImgCoord(shellFlat.astype(np.uint8)) * 255)
                cv2.imwrite("images/obs-" + str(curTime) + ".png", renderImgCoord(obsFlat.astype(np.uint8)) * 255)

                if valid:
                    q.put(newCurves)
                else:
                    q.put(None)

            except ValueError as e:
                print("ERRORED")
                print(traceback.format_exc())

            print("cam end")
            lock.release()
            camLock.release()

            time.sleep(camSleep)

