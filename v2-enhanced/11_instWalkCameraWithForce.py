import os
runRobot = int(os.environ.get('RUN_ROBOT', '0')) > 0
runCamera = int(os.environ.get('RUN_CAMERA', '0')) > 0
drawRobot = int(os.environ.get('DRAW_ROBOT', '0')) > 0

import math
import time
from multiprocessing import Process, Queue, Lock
if runRobot:
    from pyax12.connection import Connection
    import regMove
if runCamera:
    import depthai as dai
    import cameraProcessTurn
else:
    import obstacleDetectMult
if drawRobot:
    import matplotlib.pyplot as plt

from enum import Enum
import copy
import curves
import numpy as np
import genPath
import log

# import cv2
# import UVdisp
# import obstacleDetect
# import aStar
PI = 3.1415

# AX-12A motor connection
if runRobot:
    serial_connection = Connection(port="/dev/ttyAMA0", rpi_gpio=True, baudrate=1000000, waiting_time=0.08)

lenA = 18.5  # length of shoulder motor1 to shoulder motor 2
lenB = 83  # length of upper arm
lenC = 120  # length of lower arm

width = 112.6  # width of robot
height = 183.8  # length of robot

# amount of moves for every step
stepsPerCycle = 1
# amount of time taken for every move (seconds)
timePerCycle = 0.2
timePerCycleTurn = 0.15
# timePerCycle = 1

stdDelay = 0.001

# height of walk line
walkHeight = 90
# distance of steps from body
lineDist = 120
# how much to lift leg to step
# liftHeight = 40
liftHeight = 80
# amount to tilt before stepping
tiltHeight = 10

# height of turn line
turnWalkHeight = 90
# distance of steps from body
turnLineDist = 90
turnOffsetDist = 40
# how much to lift leg to step
turnLiftHeight = 40
# amount to tilt before stepping
turnTiltHeight = 10

turnStepDegs = 10

# leg max outwards reach
outX = 120
turnOutX = 100
# turnOutX = 70
outDist = math.sqrt(outX**2 + lineDist**2)
outAng = math.atan2(outX, lineDist)
# leg max inwards reach
inX = 50
turnInX = 50
inDist = math.sqrt(inX**2 + lineDist**2)
inAng = math.atan2(inX, lineDist)

order = [0, 3, 1, 2]  # order to take steps
turnOrderPos = [0, 2, 3, 1]  # order to take steps
turnOrderNeg = [0, 1, 3, 2]  # order to take steps

# order = [0, 3, 1, 2]  # order to take steps
# order = [0, 3, 1, 2]  # order to take steps


# program variables V V V
opposites = [3, 2, 1, 0]  # opposites of every leg

# relative coordinates of legs
coords = [
    [outX, lineDist, -walkHeight],
    [outX, lineDist, -walkHeight],
    [inX, lineDist, -walkHeight],
    [inX, lineDist, -walkHeight]
]

# global coordinates of legs
# globCoords = [
#     [height / 2 - inX, width / 2 + lineDist],
#     [height / 2 + outX - (outX + inX) * 1 / 3, -width / 2 - lineDist],
#     [-height / 2 + inX, width / 2 + lineDist],
#     [-height / 2 - outX + (outX + inX) * 1 / 3, -width / 2 - lineDist]
# ]

# motor ids
legId = [
    [1, 2, 3],
    [10, 11, 12],
    [7, 8, 9],
    [4, 5, 6]
]

# movement limits of leg motors
limits = [
    [-40, 20],
    [-40, 50],
    [-75, 105]
]

# rotation offsets for leg motors
rotOffs = [0, 0, -20]

driveAcc = 10  # accuracy of driving for curves (mm)
# radius = 40*10  # millimeters * 10 = centimeters
# driveCurves = [curves.quadBezier((0, 0), (radius, 0), (radius, -radius)),
#                curves.quadBezier((radius, -radius), (radius, -radius*2), (0, -radius*2)),
#                curves.quadBezier((0, -radius*2), (-radius, -radius*2), (-radius, -radius*3)),
#                curves.quadBezier((-radius, -radius*3), (-radius, -radius*4), (0, -radius*4))]

# radius = 40*10  # millimeters * 10 = centimeters
# driveCurves = [curves.quadBezier((0, 0), (radius, 0), (radius, -radius)),
#                curves.quadBezier((radius, -radius), (radius, -radius*2), (0, -radius*2)),
#                curves.quadBezier((0, -radius*2), (-radius, -radius*2), (-radius, -radius)),
#                curves.quadBezier((-radius, -radius), (-radius, 0), (0, 0))]

# radius = 70 * 10
# driveCurves = [curves.quadBezier((0, 0), (radius/2, 0), (radius, 0)), curves.quadBezier((radius, 0), (radius+radius, 0), (radius+radius, -radius))]

# radius = 300 * 10
# driveCurves = [curves.quadBezier((0, 0), (radius/2, 0), (radius, 0))]
driveCurves = []

legPos = [[height / 2 - inX, width / 2 + lineDist],
          [height / 2 + outX - (outX + inX) * 1 / 3, -width / 2 - lineDist],
          [-height / 2 + inX, width / 2 + lineDist],
          [-height / 2 - outX + (outX + inX) * 1 / 3, -width / 2 - lineDist]]  # initial positions of legs (global)

relStPos = [[- inX, lineDist],
            [+ outX - (outX + inX) * 1 / 3, lineDist],
            [+ inX, lineDist],
            [- outX + (outX + inX) * 1 / 3, lineDist]]

turnStPos = [[-turnInX, turnLineDist+turnOffsetDist],
             [turnOutX, turnLineDist-turnOffsetDist],
             [-turnOutX, turnLineDist-turnOffsetDist],
             [turnInX, turnLineDist+turnOffsetDist]]

turnStNeg = [[turnOutX, turnLineDist-turnOffsetDist],
             [-turnInX, turnLineDist+turnOffsetDist],
             [turnInX, turnLineDist+turnOffsetDist],
             [-turnOutX, turnLineDist-turnOffsetDist]]


cornerPoint = [[height / 2, width / 2],
               [height / 2, -width / 2],
               [-height / 2, width / 2],
               [-height / 2, -width / 2]]


lastRelPos = legPos.copy()

# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
extended_disparity = False  # doesn't work
# Better accuracy for longer distance, fractional disparity 32-levels:
subpixel = True
# Better handling for occlusions:
lr_check = True

focalLen = 441.25*31.35
baseline = 7.5*10

camSleepTime = 5
waitTime = 5

refLeg = order[0]
refFlag = True

lastMoved = False

moveCountdown = [-1, -1, -1, -1]

lastFoundP = [0, 0, 0, 0]

pipeline = None

if runCamera:
    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    monoLeft = pipeline.createMonoCamera()
    monoRight = pipeline.createMonoCamera()

    depth = pipeline.createStereoDepth()

    outDisp = pipeline.createXLinkOut()
    outDisp.setStreamName("depOut")

    outR = pipeline.createXLinkOut()
    outR.setStreamName("outR")

    # Properties
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    # Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
    depth.initialConfig.setConfidenceThreshold(200)
    # Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
    depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
    depth.setLeftRightCheck(subpixel)
    depth.setExtendedDisparity(extended_disparity)
    depth.setSubpixel(lr_check)

    # Linking
    monoLeft.out.link(depth.left)
    monoRight.out.link(depth.right)

    monoRight.out.link(outR.input)

    depth.depth.link(outDisp.input)


# Instruction types
class inst_type(Enum):
    nul = 0  # do nothing
    abs = 1  # go to absolute position
    rel = 2  # go to position relative to current position
    gnd = 3  # find the ground
    cal = 4  # calibrate legs to lift body to desired height
    gyr = 5  # calibrate rotation


# Instruction class
class inst:
    type = inst_type.nul
    args = []

    # args for each type:
    # 0: none
    # 1: x, y, z
    # 2: +x, +y, +z
    # 3: none
    # 4: none
    # 5: none

    def __init__(self, typeIn, argsIn):
        self.type = typeIn
        self.args = argsIn


def initVariables():
    global refLeg, refFlag, lastMoved, moveCountdown, lastFoundP

    refLeg = order[0]
    refFlag = True

    # lastMoved = False

    # moveCountdown = [-1, -1, -1, -1]

    lastFoundP = [0, 0, 0, 0]


# Main loop
def mainLoop(q, lock, **args):
    global lastMoved, cornerPoint, refFlag, lastFoundP, lastRelPos, refLeg

    dPoints = genPath.generate(driveCurves, driveAcc=driveAcc)

    initVariables()

    bodyCorners = [[], [], [], []]  # topLeft, topRight, botLeft, botRight

    for leg in range(4):
        bodyCorners[leg] = locToGlob(cornerPoint[leg], [0, 0], 0)

    lock.acquire()
    for j in range(4):
        execInst(inst(inst_type.abs, globToLoc(legPos[j], bodyCorners[j], 0, j, -walkHeight)), j)
    lock.release()

    time.sleep(stdDelay)

    if runRobot:
        lock.acquire()
        regMove.actAll(serial_connection)
        lock.release()

    if len(dPoints) > 1 and abs(dPoints[1]) > np.deg2rad(5):
        # tmpRelPos = [[], [], [], []]
        # for j in range(4):
        #     tmpRelPos[j] = globToLoc(legPos[j], bodyCorners[j], 0, j, -walkHeight)

        destPos = turnStPos if dPoints[1] > 0 else turnStNeg

        if drawRobot:
            pltRobot(dPoints, bodyCorners, 0, legPos)

        lock.acquire()
        # stepLegs(tmpRelPos, destPos, walkHeight, turnWalkHeight)
        stepLegs(coords, destPos, walkHeight, turnWalkHeight)
        lock.release()

        if drawRobot:
            pltRobot(dPoints, bodyCorners, 0, legPos)

        for j in range(4):
            # legPos[j] = locToGlob((destPos[j][0], destPos[j][1] * ((-1) ** j)), bodyCorners[j], 0)
            legPos[j] = locToGlob((coords[j][0], coords[j][1] * ((-1) ** j)), bodyCorners[j], 0)

        lock.acquire()
        spotTurn(np.rad2deg(dPoints[1]*1.4))
        lock.release()

        for leg in range(4):
            bodyCorners[leg] = locToGlob(cornerPoint[leg], [0, 0], dPoints[1])

        # tmpRelPos = [[], [], [], []]
        # for j in range(4):
        #     tmpRelPos[j] = globToLoc(legPos[j], bodyCorners[j], dPoints[1], j, -walkHeight)

        lock.acquire()
        # stepLegs(tmpRelPos, relStPos, turnWalkHeight, walkHeight)
        stepLegs(coords, relStPos, turnWalkHeight, walkHeight)
        lock.release()

        for j in range(4):
            legPos[j] = locToGlob((coords[j][0], coords[j][1] * ((-1) ** j)), bodyCorners[j], dPoints[1])

        # log.log(legPos)

        input() if ("inp" in args and args["inp"]) else None

    for posInd in range(0, min(len(dPoints)-30, 200), 2):  # min(int(len(dPoints)*3/3), int(150000/driveAcc)), 2):

        # log.log(dPoints[posInd])
        # input()

        if not q.empty():
            break

        bodyX = dPoints[posInd][0]
        bodyY = dPoints[posInd][1]

        bodyAng = dPoints[posInd + 1]

        bodyCorners = [[], [], [], []]  # topLeft, topRight, botLeft, botRight

        for leg in range(4):
            bodyCorners[leg] = locToGlob(cornerPoint[leg], [bodyX, bodyY], bodyAng)

        legMoved = -1

        foundPos = None

        # if you are moving and not the current refleg
        # check amtill of your next step, if you'd suddenly become the refleg
        # check amtill of next refleg step, if nothing happened and refleg stayed as refleg
        # choose the smaller one, set as refleg
        # if refleg doesn't change, don't do anything
        # ~~~~~~~~~~~~~~~~~~~~~~ REFLEG ~~~~~~~~~~~~~~~~~~~~~~
        for curLeg in range(0, 4):
            if moveCountdown[curLeg] == 0:
                foundPos = findNext(dPoints, bodyCorners[curLeg], bodyAng, curLeg)
                legMoved = curLeg

                if (curLeg == 0 or curLeg == 1) and refLeg != curLeg:

                    log.log("trying leg " + str(curLeg) + " as refleg")

                    otherLeg = 1-curLeg

                    # see what is amtill if you suddently become the ref leg
                    amTillNew = 0
                    # foundNew = False
                    findFlagNew = False
                    for posIndNext in range(0, len(dPoints), 2):

                        tmpPosInd = (posIndNext + posInd) % len(dPoints)

                        findBodyX = dPoints[tmpPosInd][0]
                        findBodyY = dPoints[tmpPosInd][1]
                        findBodyAng = dPoints[tmpPosInd + 1]

                        refCorner = locToGlob(cornerPoint[curLeg], [findBodyX, findBodyY], findBodyAng)
                        findDist = dist(foundPos, refCorner)

                        if (not findFlagNew) and findDist < inDist and findDist < outDist:
                            findFlagNew = True

                        if findDist > (inDist if 0 <= 1 else outDist) and findFlagNew:
                            # foundNew = True
                            break

                        amTillNew += 1

                    # find next refleg step position

                    foundPos2 = findNext(dPoints, bodyCorners[otherLeg], bodyAng, otherLeg)

                    amTillRef = 0
                    # foundRef = False
                    findFlagRef = False
                    for posIndNext in range(0, len(dPoints), 2):

                        tmpPosInd = (posIndNext + posInd) % len(dPoints)

                        findBodyX = dPoints[tmpPosInd][0]
                        findBodyY = dPoints[tmpPosInd][1]
                        findBodyAng = dPoints[tmpPosInd + 1]

                        refCorner = locToGlob(cornerPoint[otherLeg], [findBodyX, findBodyY], findBodyAng)
                        findDist = dist(foundPos2, refCorner)

                        if (not findFlagRef) and findDist < inDist and findDist < outDist:
                            findFlagRef = True

                        if findDist > (inDist if otherLeg <= 1 else outDist) and findFlagRef:
                            # foundRef = True
                            break

                        amTillRef += 1

                    bodyXtmpRef = dPoints[(posInd + amTillRef*2) % len(dPoints)][0]
                    bodyYtmpRef = dPoints[(posInd + amTillRef*2) % len(dPoints)][1]

                    bodyAngtmpRef = dPoints[(posInd + amTillRef*2 + 1) % len(dPoints)]
                    tmpCorner = locToGlob(cornerPoint[otherLeg], [bodyXtmpRef, bodyYtmpRef], bodyAngtmpRef)

                    foundPosTmp1 = findNext(dPoints, tmpCorner, bodyAngtmpRef, otherLeg)

                    amTillRefFind = 0
                    # foundRef = False
                    findFlagRef = False
                    for posIndNext in range(0, len(dPoints), 2):

                        tmpPosInd = (posIndNext + posInd + amTillRef*2) % len(dPoints)

                        findBodyX = dPoints[tmpPosInd][0]
                        findBodyY = dPoints[tmpPosInd][1]
                        findBodyAng = dPoints[tmpPosInd + 1]

                        refCorner = locToGlob(cornerPoint[otherLeg], [findBodyX, findBodyY], findBodyAng)
                        findDist = dist(foundPosTmp1, refCorner)

                        if (not findFlagRef) and findDist < inDist and findDist < outDist:
                            findFlagRef = True

                        if findDist > (inDist if otherLeg <= 1 else outDist) and findFlagRef:
                            # foundRef = True
                            break

                        amTillRefFind += 1

                    log.log("old: " + str(amTillRefFind) + "       new: " + str(amTillNew))

                    if amTillNew < amTillRefFind:
                        # set new leg as ref leg
                        refLeg = curLeg
                        log.log("set leg " + str(curLeg) + " as refleg")
                        for moveTimerLeg in range(0, 4):
                            moveCountdown[order[(moveTimerLeg+order.index(refLeg)) % 4]] = int(amTillNew * moveTimerLeg / 4)

                    log.log("")

            if moveCountdown[curLeg] >= 0:
                moveCountdown[curLeg] -= 1
        # ~~~~~~~~~~~~~~~~~~~~~~ REFLEG END ~~~~~~~~~~~~~~~~~~~~~~

        # ~~~~~~~~~~~~~~~~~~~~~~ CHECK REFLEG DIST ~~~~~~~~~~~~~~~~~~~~~~

        tmpDist = dist(legPos[refLeg], bodyCorners[refLeg])

        if (not refFlag) and tmpDist < inDist and tmpDist < outDist:
            refFlag = True

        if tmpDist > (inDist if refLeg <= 1 else outDist) and refFlag:  # test if reference leg is outside of its maximum range
            log.log("out of range")
            foundPos = findNext(dPoints, bodyCorners[refLeg], bodyAng, refLeg)

            amTill = 0
            found = False
            refFlag = False

            findFlag = False

            # search for next step of reference leg, and count amount
            for posIndNext in range(0, len(dPoints), 2):

                tmpPosInd = (posIndNext + posInd) % len(dPoints)

                findBodyX = dPoints[tmpPosInd][0]
                findBodyY = dPoints[tmpPosInd][1]
                findBodyAng = dPoints[tmpPosInd + 1]

                refCorner = locToGlob(cornerPoint[refLeg], [findBodyX, findBodyY], findBodyAng)
                findDist = dist(foundPos, refCorner)

                if (not findFlag) and findDist < inDist and findDist < outDist:
                    findFlag = True

                if findDist > (inDist if refLeg <= 1 else outDist) and findFlag:
                    found = True
                    break

                amTill += 1

            if found:
                for moveTimerLeg in range(0, 4):
                    moveCountdown[order[(moveTimerLeg+order.index(refLeg)) % 4]] = int(amTill * moveTimerLeg / 4)
                moveCountdown[refLeg] = -1

            legMoved = refLeg

        relLegPos = [[], [], [], []]

        for j in range(4):  # calculate leg positions relative to body
            relLegPos[j] = globToLoc(legPos[j], bodyCorners[j], bodyAng, j, -walkHeight)

        if lastMoved and legMoved == -1:  # un-tilt
            lastMoved = False

            lock.acquire()

            for leg in range(4):
                execInst(inst(inst_type.abs, relLegPos[leg]), leg)

            time.sleep(stdDelay)

            if runRobot:
                regMove.actAll(serial_connection)

            lock.release()

            time.sleep(stdDelay)
            time.sleep(timePerCycle)

        if legMoved != -1:  # a leg has reached its time, move it

            lastMoved = True
            lock.acquire()
            for leg in range(4):  # move back and tilt
                if leg == legMoved:
                    execInst(inst(inst_type.abs, [relLegPos[leg][0], relLegPos[leg][1], -walkHeight - tiltHeight]), leg)
                elif leg == opposites[legMoved]:
                    execInst(inst(inst_type.abs, [relLegPos[leg][0], relLegPos[leg][1], -walkHeight + tiltHeight]), leg)
                else:
                    execInst(inst(inst_type.abs, relLegPos[leg]), leg)

            time.sleep(stdDelay)

            if runRobot:
                regMove.actAll(serial_connection)

            time.sleep(stdDelay)
            time.sleep(timePerCycle)

            for leg in range(4):  # stay and move lifting leg forwards
                if leg == legMoved:
                    cornerCoords = bodyCorners[legMoved]

                    # update relLegPos with forward pos
                    relLegPos[legMoved] = globToLoc(foundPos, cornerCoords, bodyAng, legMoved % 2, -walkHeight)

                    # update legPos with forward pos
                    legPos[legMoved] = foundPos

                    execInst(inst(inst_type.abs, [relLegPos[leg][0], relLegPos[leg][1], -walkHeight + liftHeight]), leg)
                elif leg == opposites[legMoved]:
                    execInst(inst(inst_type.abs, [relLegPos[leg][0], relLegPos[leg][1], -walkHeight + tiltHeight]), leg)
                else:
                    execInst(inst(inst_type.abs, relLegPos[leg]), leg)

            lastRelPos = copy.deepcopy(relLegPos)

            time.sleep(stdDelay)

            if runRobot:
                regMove.actAll(serial_connection)

            lock.release()

            time.sleep(stdDelay)
            time.sleep(timePerCycle)

            if drawRobot:
                pltRobot(dPoints, bodyCorners, bodyAng, legPos)

    if lastMoved:  # un-tilt
        lock.acquire()
        lastMoved = False

        for leg in range(4):
            execInst(inst(inst_type.abs, lastRelPos[leg]), leg)

        time.sleep(stdDelay)

        if runRobot:
            regMove.actAll(serial_connection)

        lock.release()

        time.sleep(stdDelay)
        time.sleep(timePerCycle)

    # if lock.locked():
    #     lock.release()


def stepLegs(curRelPos, newRelPos, curHeight, newHeight):

    curRelPos = copy.deepcopy(curRelPos)
    newRelPos = copy.deepcopy(newRelPos)

    for leg in range(4):
        execInst(inst(inst_type.abs, [curRelPos[leg][0], curRelPos[leg][1], -curHeight]), leg)

    time.sleep(stdDelay)

    if runRobot:
        regMove.actAll(serial_connection)

    time.sleep(stdDelay)
    time.sleep(timePerCycle)

    output = copy.deepcopy(turnOrderPos)
    for index, x in enumerate(sorted(range(len(newRelPos)), key=lambda y: -newRelPos[y][0] * (-1 if y > 1 else 1))):
        output[index] = x
    # for index, x in enumerate(sorted(range(len(curRelPos)), key=lambda y: curRelPos[y][0] * (-1 if y > 1 else 1))):
    #     output[index] = x

    for moveLeg in output:

        for leg in range(4):  # move back and tilt
            if leg == moveLeg:
                execInst(inst(inst_type.abs, [curRelPos[leg][0], curRelPos[leg][1], -curHeight - turnTiltHeight]), leg)
            elif leg == opposites[moveLeg]:
                execInst(inst(inst_type.abs, [curRelPos[leg][0], curRelPos[leg][1], -curHeight + turnTiltHeight]), leg)
            else:
                execInst(inst(inst_type.abs, [curRelPos[leg][0], curRelPos[leg][1], -curHeight]), leg)

        time.sleep(stdDelay)

        if runRobot:
            regMove.actAll(serial_connection)

        time.sleep(stdDelay)
        time.sleep(timePerCycle)
        time.sleep(timePerCycle)

        for leg in range(4):  # move back and tilt
            if leg == moveLeg:
                execInst(inst(inst_type.abs, [curRelPos[leg][0], curRelPos[leg][1], -curHeight + turnLiftHeight]), leg)
            elif leg == opposites[moveLeg]:
                execInst(inst(inst_type.abs, [curRelPos[leg][0], curRelPos[leg][1], -curHeight + turnTiltHeight]), leg)
            else:
                execInst(inst(inst_type.abs, [curRelPos[leg][0], curRelPos[leg][1], -curHeight]), leg)

        time.sleep(stdDelay)

        if runRobot:
            regMove.actAll(serial_connection)

        time.sleep(stdDelay)
        time.sleep(timePerCycle)
        time.sleep(timePerCycle)

        for leg in range(4):  # move leg
            if leg == moveLeg:
                curRelPos[leg] = copy.deepcopy(newRelPos[leg])

                execInst(inst(inst_type.abs, [curRelPos[leg][0], curRelPos[leg][1], -curHeight + turnLiftHeight]), leg)
            elif leg == opposites[moveLeg]:
                execInst(inst(inst_type.abs, [curRelPos[leg][0], curRelPos[leg][1], -curHeight + turnTiltHeight]), leg)
            else:
                execInst(inst(inst_type.abs, [curRelPos[leg][0], curRelPos[leg][1], -curHeight]), leg)

        time.sleep(stdDelay)

        if runRobot:
            regMove.actAll(serial_connection)

        time.sleep(stdDelay)
        time.sleep(timePerCycle)

    for leg in range(4):
        execInst(inst(inst_type.abs, [curRelPos[leg][0], curRelPos[leg][1], -newHeight]), leg)

    time.sleep(stdDelay)

    if runRobot:
        regMove.actAll(serial_connection)

    time.sleep(stdDelay)
    time.sleep(timePerCycle)


def spotTurn(degs):
    counter = 0
    turnSt = turnStPos if degs > 0 else turnStNeg
    turnOrder = turnOrderPos if degs > 0 else turnOrderNeg
    log.log("order: " + str(turnOrder))

    for _ in np.arange(0, int(degs/turnStepDegs)+(1 if degs > 0 else -1), 1 if degs > 0 else -1):
        moveLeg = turnOrder[counter % len(turnOrder)]
        log.log("turning " + str(moveLeg))
        counter = (counter + 1)
        rotDegs = (counter * turnStepDegs * (1 if degs > 0 else -1)) if abs(counter * turnStepDegs) < abs(degs) else degs
        rot = np.deg2rad(rotDegs)

        log.log(rotDegs)

        bodyCorners = [[], [], [], []]  # topLeft, topRight, botLeft, botRight

        for leg in range(4):
            bodyCorners[leg] = locToGlob(cornerPoint[leg], [0, 0], rot)

        for leg in range(4):  # move back and tilt
            if leg == moveLeg:
                execInst(inst(inst_type.abs, globToLoc(legPos[leg], bodyCorners[leg], rot, leg, -turnWalkHeight - turnTiltHeight)), leg)
            elif leg == opposites[moveLeg]:
                execInst(inst(inst_type.abs, globToLoc(legPos[leg], bodyCorners[leg], rot, leg, -turnWalkHeight + turnTiltHeight)), leg)
            else:
                execInst(inst(inst_type.abs, globToLoc(legPos[leg], bodyCorners[leg], rot, leg, -turnWalkHeight)), leg)

        time.sleep(stdDelay)

        if runRobot:
            regMove.actAll(serial_connection)

        time.sleep(stdDelay)
        time.sleep(timePerCycleTurn)

        for leg in range(4):  # move leg forward
            if leg == moveLeg:
                execInst(inst(inst_type.abs, (turnSt[leg][0], turnSt[leg][1], -turnWalkHeight + turnLiftHeight)), leg)
                legPos[leg] = locToGlob((turnSt[leg][0], turnSt[leg][1] * ((-1) ** leg)), bodyCorners[leg], rot)
            elif leg == opposites[moveLeg]:
                execInst(inst(inst_type.abs, globToLoc(legPos[leg], bodyCorners[leg], rot, leg, -turnWalkHeight + turnTiltHeight)), leg)
            else:
                execInst(inst(inst_type.abs, globToLoc(legPos[leg], bodyCorners[leg], rot, leg, -turnWalkHeight)), leg)

        time.sleep(stdDelay)

        if runRobot:
            regMove.actAll(serial_connection)

        time.sleep(stdDelay)
        time.sleep(timePerCycleTurn)

        for j in range(4):
            execInst(inst(inst_type.abs, globToLoc(legPos[j], bodyCorners[j], rot, j, -turnWalkHeight)), j)

        # log.log(legPos)

        time.sleep(stdDelay)

        if runRobot:
            regMove.actAll(serial_connection)

        time.sleep(stdDelay)
        time.sleep(timePerCycleTurn)


def findNext(dPoints, cornerCoord, bodyAng, leg):
    searchDist = (width / 2 + lineDist)
    relPos = [0, searchDist * ((-1) ** leg)]  # flip to right side (legs 1 and 3)
    refAng = (bodyAng + (0.5*PI if leg % 2 == 0 else 1.5*PI)) % (2*PI)

    back = inDist if leg <= 1 else outDist
    front = outDist if leg <= 1 else inDist

    backAng = inAng if leg <= 1 else outAng
    frontAng = outAng if leg <= 1 else inAng

    foundInd = lastFoundP[leg]

    flag = False

    for j in range(0, len(dPoints)*2, 2):
        toGo = (j + lastFoundP[leg]) % len(dPoints)

        testX = dPoints[toGo][0]
        testY = dPoints[toGo][1]
        testAng = dPoints[toGo + 1]

        globLegTestPos = locToGlob(relPos, [testX, testY], testAng)
        testAng = (math.atan2(globLegTestPos[1]-cornerCoord[1], globLegTestPos[0]-cornerCoord[0]) + (2*PI)) % (2*PI)
        diffAng = (testAng-refAng) * ((-1) ** (leg+1))  # flip on legs 0 and 2

        testDist = dist(globLegTestPos, cornerCoord)

        # if (diffAng > frontAng)  or (diffAng > PI) or testDist > max(front, back):
        if flag and (diffAng > frontAng or testDist > max(front, back)):
            foundInd = ((toGo - 2) + len(dPoints)) % len(dPoints)
            log.log("found for leg " + str(leg) + " " + str(lastFoundP[leg]) + " " + str(foundInd))
            break

        if (not flag) and abs(diffAng) < min(backAng, frontAng) and testDist < max(front, back):
            flag = True
            log.log("flag set for leg " + str(leg) + " " + str(lastFoundP[leg]) + " " + str(toGo))

    foundX = dPoints[foundInd][0]
    foundY = dPoints[foundInd][1]
    foundAng = dPoints[foundInd + 1]

    lastFoundP[leg] = foundInd

    return locToGlob(relPos, [foundX, foundY], foundAng)


def dist(xy1, xy2):
    return math.sqrt((xy2[0] - xy1[0]) ** 2 + (xy2[1] - xy1[1]) ** 2)


def calcRots(xyz, leg):
    x = xyz[0] * ((-1) ** leg)  # invert x of legs 1 and 3 (right side)
    y = xyz[1]
    z = xyz[2]

    xyAng = math.degrees(math.atan(x / y)) if y != 0 else (90 if x >= 0 else -90)  # z-rot of entire leg, from center (90) (shoulder side to side)
    xyLen = math.sqrt((x ** 2) + (y ** 2)) - lenA  # length of leg vector projected from Z to ground
    trueLen = math.sqrt((xyLen ** 2) + (z ** 2))  # actual length of leg vector

    zXYAng = math.degrees(math.atan(xyLen / -z)) if z != 0 else 0  # angle of leg vector (shoulder part 1)

    if trueLen >= lenB + lenC:
        angA = 0
        angB = 180
    else:
        angA = math.degrees(math.acos(((trueLen ** 2) + (lenB ** 2) - (lenC ** 2)) / (2 * trueLen * lenB)))  # angle of top arm from vector (shoulder part 2)
        angB = math.degrees(math.acos(((lenB ** 2) + (lenC ** 2) - (trueLen ** 2)) / (2 * lenB * lenC)))  # angle of bottom arm from top arm (elbow)

    return [(-xyAng) + rotOffs[0], (90 - zXYAng - angA) + rotOffs[1], (angB - 90) + rotOffs[2]]  # angle at elbow has to be inverted


# Drive all motors of a leg
def moveLegs(rots, leg):
    driveLeg(leg, 0, rots[0])

    time.sleep(stdDelay)

    if leg == 0 or leg == 3:
        driveLeg(leg, 1, rots[1])
    else:
        driveLeg(leg, 1, -rots[1])

    time.sleep(stdDelay)

    if leg == 0 or leg == 3:
        driveLeg(leg, 2, rots[2])
    else:
        driveLeg(leg, 2, -rots[2])


# Executes instruction provided on the leg provided
def execInst(ins, leg):
    global coords

    for tryy in range(3):
        try:
            if ins.type == inst_type.abs:
                for j in range(0, 3):
                    if ins.args[j] is not None:
                        coords[leg][j] = ins.args[j] * (0.80 if j == 0 and leg % 2 == 1 else 1)
                moveLegs(calcRots(coords[leg], leg), leg)

            elif ins.type == inst_type.rel:
                for j in range(0, 3):
                    if ins.args[j] is not None:
                        coords[leg][j] += ins.args[j]
                moveLegs(calcRots(coords[leg], leg), leg)

            return

        except ValueError:
            log.log("incomplete packet! try #" + str(tryy+1))

    raise ValueError("INCOMPLETE PACKET 3x")


# drives single motor of a leg
def driveLeg(leg, motor, rot):
    toDrive = clamp(limits[motor][0], limits[motor][1], rot * (-1 if motor == 0 and leg == 1 or leg == 2 else 1)) * (
        -1 if motor == 0 and leg == 1 or leg == 2 else 1)

    if runRobot:
        regMove.regMove(serial_connection, legId[leg][motor], toDrive, speed=1023, degrees=True)


def clamp(inmin, inmax, num):
    return max(inmin, min(inmax, num))


def locToGlob(loc_p, orig_p, rad):
    cosAm = math.cos(rad)
    sinAm = math.sin(rad)
    x = loc_p[0]
    y = loc_p[1]
    return [(cosAm * x) - (sinAm * y) + orig_p[0], (sinAm * x) + (cosAm * y) + orig_p[1]]


def globToLoc(glob_p, orig_p, orig_rot, flipY=0, zHeight=0):
    # atan2(posY - carY, posX - carX) to get global angle
    # (global car angle) - (global leg angle) to get local angle
    # sin and cos to go back to local coordinates
    # invert Y coord on right side of robot

    rel_x = glob_p[0] - orig_p[0]
    rel_y = glob_p[1] - orig_p[1]
    cosAm = math.cos(-orig_rot)
    sinAm = math.sin(-orig_rot)
    return [cosAm * rel_x - sinAm * rel_y, (sinAm * rel_x + cosAm * rel_y) * ((-1) ** flipY), zHeight]


def pltRobot(dPoints, corners, angle, legs):
    xPoints = []
    yPoints = []
    for index in range(0, len(dPoints), 2):
        xPoints.append(dPoints[index][0])
        yPoints.append(dPoints[index][1])

    graph1.clear()
    graph1.axis([-2000, 2000, -2000, 2000])
    graph1.plot(xPoints, yPoints, 'r-')

    xPoints = []
    yPoints = []

    indList = [0, 1, 3, 2, 0]

    for index in indList:
        xPoints.append(corners[index][0])
        yPoints.append(corners[index][1])

    graph1.plot(xPoints, yPoints, 'b-')

    outPoints = [[120, 120], [120, -120], [-120, 120], [-120, -120]]
    for index in range(4):
        gp = locToGlob(outPoints[index], corners[index],  angle)
        graph1.plot([gp[0], corners[index][0]], [gp[1], corners[index][1]], 'y-')

    inPoints = [[-50, 120], [-50, -120], [50, 120], [50, -120]]
    for index in range(4):
        gp = locToGlob(inPoints[index], corners[index],  angle)
        graph1.plot([gp[0], corners[index][0]], [gp[1], corners[index][1]], 'y-')

    for index in range(4):
        graph1.plot([legs[index][0], corners[index][0]], [legs[index][1], corners[index][1]], 'b-')

    tmp = ""

    for index in range(4):
        tmp += "(" + str(int(legs[index][0])) + "," + str(int(legs[index][1])) + ") "

    log.log(tmp)

    fig.canvas.draw()
    # plt.waitforbuttonpress()
    plt.pause(0.1)


def driveLine(rad): return [curves.quadBezier((0, 0), (rad/2, 0), (rad, 0))]


def driveTurnLine(turn, rad): return [curves.quadBezier((0, 0), (rad*math.cos(math.radians(turn))/2, rad*math.sin(math.radians(turn))/2), (rad*math.cos(math.radians(turn)), rad*math.sin(math.radians(turn))))]


def driveTurnLineSmooth(rad1, turn, rad2): return [
    curves.quadBezier((0, 0),
                      (rad1/4, 0),
                      (rad1/2, 0)),

    curves.quadBezier((rad1/2, 0),
                      (rad1, 0),
                      (rad1 + rad2*math.cos(math.radians(turn))/2, rad2*math.sin(math.radians(turn))/2)),

    curves.quadBezier((rad1 + rad2*math.cos(math.radians(turn))/2, rad2*math.sin(math.radians(turn))/2),
                      (rad1 + rad2*math.cos(math.radians(turn))*3/4, rad2*math.sin(math.radians(turn))*3/4),
                      (rad1 + rad2 * math.cos(math.radians(turn)), rad2 * math.sin(math.radians(turn))))
]


def driveQuatCircleCW(rad): return [curves.quadBezier((0, 0), (rad, 0), (rad, -rad))]


# forceToDrive = driveLine(300*10)
forceToDrive = None


# program start
if __name__ == "__main__":
    import atexit

    def exit_handler():
        log.log("\n\n~~~ program end: " + str(time.asctime(time.localtime())) + " ~~~\n\n")

    atexit.register(exit_handler)

    qu = Queue()
    ll = Lock()
    cl = Lock()

    mainLoop(qu, ll)

    if forceToDrive is not None:
        while True:
            input()
            driveCurves = forceToDrive

            mainLoop(qu, ll, inp=True)

            log.log("loop over")

            if lastRelPos is None:
                break

            bodyCornersTmp = [[], [], [], []]
            for ind in range(4):
                bodyCornersTmp[ind] = locToGlob(cornerPoint[ind], [0, 0], 0)

            for i in range(4):
                legPos[i] = locToGlob((coords[i][0], coords[i][1] * ((-1) ** i)), bodyCornersTmp[i], 0)

    else:
        if runCamera:
            p = Process(target=cameraProcessTurn.takeImageLidar, args=(qu, ll, cl, pipeline, camSleepTime),
                        kwargs={"flagWaitTime": 1, "focalLen": focalLen, "baseline": baseline, "robotWidth": width/2+lineDist},
                        daemon=True)
        else:
            p = Process(target=obstacleDetectMult.takeImage, args=(qu, ll, None, camSleepTime),
                        kwargs={"flagWaitTime": 1, "focalLen": focalLen, "baseline": baseline, "robotWidth": width/2+lineDist},
                        daemon=True)

        p.start()

        if drawRobot:
            fig = plt.figure()
            graph1 = fig.add_subplot(1, 1, 1)

            graph1.axis([-500, 2000, -2000, 500])

        while True:
            while qu.empty():
                log.log("path check... empty")
                time.sleep(waitTime)

            while not qu.empty():
                log.log("path got")
                driveCurves = qu.get()

            # radius = 70 * 10
            # driveCurves = [curves.quadBezier((0, 0), (radius/2, 0), (radius, 0)), curves.quadBezier((radius, 0), (radius+radius, 0), (radius+radius, -radius))]

            # radius = 70 * 10
            # driveCurves = [curves.quadBezier((0, 0), (radius/2, radius/2), (radius, radius))]

            cl.acquire()

            if driveCurves is None:
                log.log("invalid images, turning 90")

                rotRad = np.deg2rad(-90)

                bodyCornerss = [[], [], [], []]  # topLeft, topRight, botLeft, botRight

                for legg in range(4):
                    bodyCornerss[legg] = locToGlob(cornerPoint[legg], [0, 0], 0)

                # tmpRelPoss = [[], [], [], []]
                # for k in range(4):
                #     tmpRelPoss[k] = globToLoc(legPos[k], bodyCornerss[k], 0, k, -walkHeight)

                destPoss = turnStPos if rotRad > 0 else turnStNeg

                stepLegs(coords, destPoss, walkHeight, turnWalkHeight)

                for k in range(4):
                    # legPos[k] = locToGlob((destPoss[k][0], destPoss[k][1] * ((-1) ** k)), bodyCornerss[k], 0)
                    legPos[k] = locToGlob((coords[k][0], coords[k][1] * ((-1) ** k)), bodyCornerss[k], 0)

                spotTurn(-90)

                for legg in range(4):
                    bodyCornerss[legg] = locToGlob(cornerPoint[legg], [0, 0], rotRad)

                # tmpRelPoss = [[], [], [], []]
                # for k in range(4):
                #     tmpRelPoss[k] = globToLoc(legPos[k], bodyCornerss[k], rotRad, k, -walkHeight)

                # destPoss = relStPos

                stepLegs(coords, relStPos, turnWalkHeight, walkHeight)

                for k in range(4):
                    # legPos[k] = locToGlob((destPoss[k][0], destPoss[k][1] * ((-1) ** k)), bodyCornerss[k], 0)
                    legPos[k] = locToGlob((coords[k][0], coords[k][1] * ((-1) ** k)), bodyCornerss[k], 0)
            else:
                mainLoop(qu, ll)

            cl.release()

            log.log("loop over")

            if lastRelPos is None:
                break

            bodyCornersTmp = [[], [], [], []]
            for ind in range(4):
                bodyCornersTmp[ind] = locToGlob(cornerPoint[ind], [0, 0], 0)

            for i in range(4):
                # legPos[i] = locToGlob((lastRelPos[i][0], lastRelPos[i][1] * ((-1) ** i)), bodyCornersTmp[i], 0)
                legPos[i] = locToGlob((coords[i][0], coords[i][1] * ((-1) ** i)), bodyCornersTmp[i], 0)

            # break
            # input()
