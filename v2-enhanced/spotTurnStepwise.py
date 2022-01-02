import os

import numpy as np

runRobot = int(os.environ['RUN_ROBOT']) > 0

import math
import time
from multiprocessing import Process, Queue, Lock
if runRobot:
    from pyax12.connection import Connection
    import regMove
from enum import Enum
import copy
# import curves
# import numpy as np
# import cv2
# import UVdisp
# import obstacleDetect
# import aStar
PI = 3.1415

# AX-12A motor connection
if runRobot:
    serial_connection = Connection(port="/dev/ttyAMA0", rpi_gpio=True, baudrate=1000000)

lenA = 18.5  # length of shoulder motor1 to shoulder motor 2
lenB = 83  # length of upper arm
lenC = 120  # length of lower arm

width = 112.6  # width of robot
height = 183.8  # length of robot

# amount of moves for every step
stepsPerCycle = 1
# amount of time taken for every move (seconds)
timePerCycle = 0.15
# timePerCycle = 1

stdDelay = 0.001

# height of walk line
walkHeight = 90
# distance of steps from body
lineDist = 120
# how much to lift leg to step
liftHeight = 40
# amount to tilt before stepping
tiltHeight = 10

# height of turn line
turnWalkHeight = 90
turnHeight = 90
# distance of steps from body
turnLineDist = 90
turnOffsetDist = 40
# how much to lift leg to step
turnLiftHeight = 40
# amount to tilt before stepping
turnTiltHeight = 10

turnStepDegs = 10

# leg max outwards reach
# outX = 120
# leg max outwards reach
outX = 120
turnOutX = 70
outDist = math.sqrt(outX**2 + lineDist**2)
outAng = math.atan2(outX, lineDist)
# leg max inwards reach
inX = 50
turnInX = 50
inDist = math.sqrt(inX**2 + lineDist**2)
inAng = math.atan2(inX, lineDist)

# order = [0, 2, 3, 1]  # order to take steps
# order = [0, 1, 3, 2]  # order to take steps
order = [0, 2, 3, 1]  # order to take steps
turnOrderPos = [0, 2, 3, 1]  # order to take steps
turnOrderNeg = [0, 1, 3, 2]  # order to take steps

# program variables V V V
opposites = [3, 2, 1, 0]  # opposites of every leg

# initial starting coordinates
coords = [
    [outX, turnLineDist, -walkHeight],
    [outX, turnLineDist, -walkHeight],
    [inX, turnLineDist, -walkHeight],
    [inX, turnLineDist, -walkHeight]
]

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

# radius = 70 * 10
# driveCurves = [curves.quadBezier((0, 0), (radius/2, 0), (radius, 0)), curves.quadBezier((radius, 0), (radius+radius, 0), (radius+radius, -radius))]

# radius = 300 * 10
# driveCurves = [curves.quadBezier((0, 0), (radius/2, 0), (radius, 0))]
driveCurves = []

# legPos = [[height / 2 - inX, width / 2 + lineDist],
#           [height / 2 + outX - (outX + inX) * 1 / 3, -width / 2 - lineDist],
#           [-height / 2 + inX, width / 2 + lineDist],
#           [-height / 2 - outX + (outX + inX) * 1 / 3, -width / 2 - lineDist]]  # initial positions of legs (global)

legPos = [[height / 2 + outX, width / 2 + turnLineDist],
          [height / 2 + outX, -width / 2 - turnLineDist],
          [-height / 2 - outX, width / 2 + turnLineDist],
          [-height / 2 + inX, -width / 2 - turnLineDist]]  # initial positions of legs (global)

# legPos = [[height / 2, width / 2 + turnLineDist],
#           [height / 2, -width / 2 - turnLineDist],
#           [-height / 2, width / 2 + turnLineDist],
#           [-height / 2, -width / 2 - turnLineDist]]  # initial positions of legs (global)


# turnRelLeg = [[-inX, turnLineDist+lineDist, -turnHeight],
#               [outX, turnLineDist-lineDist, -turnHeight],
#               [-outX, turnLineDist-lineDist, -turnHeight],
#               [inX, turnLineDist+lineDist, -turnHeight]]

turnRelLeg = [[outX, turnLineDist-lineDist, -turnHeight],
              [-inX, turnLineDist+lineDist, -turnHeight],
              [inX, turnLineDist+lineDist, -turnHeight],
              [-outX, turnLineDist-lineDist, -turnHeight]]

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


refLeg = order[0]
refFlag = True

lastMoved = False

cornerPoint = [[height / 2, width / 2],
               [height / 2, -width / 2],
               [-height / 2, width / 2],
               [-height / 2, -width / 2]]

moveCountdown = [-1, -1, -1, -1]

lastFoundP = [0, 0, 0, 0]

lastRelPos = legPos.copy()

# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
extended_disparity = False  # doesn't work
# Better accuracy for longer distance, fractional disparity 32-levels:
subpixel = True
# Better handling for occlusions:
lr_check = True

focalLen = 441.25*31.35
baseline = 7.5*10

camSleepTime = 30
waitTime = 2


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


# Main loop
def mainLoop():
    global lastMoved, cornerPoint, refFlag, lastFoundP, lastRelPos, refLeg

    step = 5
    stepRad = np.deg2rad(step)
    amTurn = 45

    counter = 0

    while True:
        for ii in np.arange(0, amTurn, step):
            moveLeg = order[counter % len(order)]
            counter = (counter+1)
            rot = np.deg2rad(counter*step)
            print(rot)

            bodyCorners = [[], [], [], []]  # topLeft, topRight, botLeft, botRight

            for ind in range(4):
                bodyCorners[ind] = locToGlob(cornerPoint[ind], [0, 0], rot)

            for leg in range(4):  # move back and tilt
                if leg == moveLeg:
                    execInst(inst(inst_type.abs, globToLoc(legPos[leg], bodyCorners[leg], rot, leg, -turnHeight - tiltHeight)), leg)
                elif leg == opposites[moveLeg]:
                    execInst(inst(inst_type.abs, globToLoc(legPos[leg], bodyCorners[leg], rot, leg, -turnHeight + tiltHeight)), leg)
                else:
                    execInst(inst(inst_type.abs, globToLoc(legPos[leg], bodyCorners[leg], rot, leg, -turnHeight)), leg)

            time.sleep(stdDelay)

            if runRobot:
                regMove.actAll(serial_connection)

            time.sleep(stdDelay)
            time.sleep(timePerCycle)

            for leg in range(4):  # move leg forward
                if leg == moveLeg:
                    execInst(inst(inst_type.abs, (turnRelLeg[leg][0], turnRelLeg[leg][1], -turnHeight + liftHeight)), leg)
                    legPos[leg] = locToGlob((turnRelLeg[leg][0], turnRelLeg[leg][1] * ((-1) ** leg), turnRelLeg[leg][2]), bodyCorners[leg], rot)
                elif leg == opposites[moveLeg]:
                    execInst(inst(inst_type.abs, globToLoc(legPos[leg], bodyCorners[leg], rot, leg, -turnHeight + tiltHeight)), leg)
                else:
                    execInst(inst(inst_type.abs, globToLoc(legPos[leg], bodyCorners[leg], rot, leg, -turnHeight)), leg)

            time.sleep(stdDelay)

            if runRobot:
                regMove.actAll(serial_connection)

            time.sleep(stdDelay)
            time.sleep(timePerCycle)

            for j in range(4):
                execInst(inst(inst_type.abs, globToLoc(legPos[j], bodyCorners[j], rot, j, -turnHeight)), j)

            time.sleep(stdDelay)

            if runRobot:
                regMove.actAll(serial_connection)

            time.sleep(stdDelay)
            time.sleep(timePerCycle)

        # stepLegs(coords, turnRelLeg)
        input()

def spotTurn(degs):
    counter = 0
    turnSt = turnStPos if degs > 0 else turnStNeg
    turnOrder = turnOrderPos if degs > 0 else turnOrderNeg

    for _ in np.arange(0, int(degs/turnStepDegs)+(1 if degs > 0 else -1), 1 if degs > 0 else -1):
        moveLeg = turnOrder[counter % len(order)]
        counter = (counter + 1)
        rotDegs = (counter * turnStepDegs * (1 if degs > 0 else -1)) if abs(counter * turnStepDegs) < abs(degs) else degs
        rot = np.deg2rad(rotDegs)

        print(rotDegs)

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
        time.sleep(timePerCycle)

        for leg in range(4):  # move leg forward
            if leg == moveLeg:
                execInst(inst(inst_type.abs, (turnSt[leg][0], turnSt[leg][1], -turnWalkHeight + liftHeight)), leg)
                legPos[leg] = locToGlob((turnSt[leg][0], turnSt[leg][1] * ((-1) ** leg)), bodyCorners[leg], rot)
            elif leg == opposites[moveLeg]:
                execInst(inst(inst_type.abs, globToLoc(legPos[leg], bodyCorners[leg], rot, leg, -turnWalkHeight + turnTiltHeight)), leg)
            else:
                execInst(inst(inst_type.abs, globToLoc(legPos[leg], bodyCorners[leg], rot, leg, -turnWalkHeight)), leg)

        time.sleep(stdDelay)

        if runRobot:
            regMove.actAll(serial_connection)

        time.sleep(stdDelay)
        time.sleep(timePerCycle)

        for j in range(4):
            execInst(inst(inst_type.abs, globToLoc(legPos[j], bodyCorners[j], rot, j, -turnWalkHeight)), j)

        # print(legPos)

        time.sleep(stdDelay)

        if runRobot:
            regMove.actAll(serial_connection)

        time.sleep(stdDelay)
        time.sleep(timePerCycle)


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

    if ins.type == inst_type.abs:
        for j in range(0, 3):
            if ins.args[j] is not None:
                coords[leg][j] = ins.args[j]
        moveLegs(calcRots(coords[leg], leg), leg)

    elif ins.type == inst_type.rel:
        for j in range(0, 3):
            if ins.args[j] is not None:
                coords[leg][j] += ins.args[j]
        moveLegs(calcRots(coords[leg], leg), leg)


# drives single motor of a leg
def driveLeg(leg, motor, rot):
    toDrive = clamp(limits[motor][0], limits[motor][1], rot * (-1 if motor == 0 and leg == 1 or leg == 2 else 1)) * (
        -1 if motor == 0 and leg == 1 or leg == 2 else 1)

    if runRobot:
        regMove.regMove(serial_connection, legId[leg][motor], toDrive, speed=1023, degrees=True)


def clamp(inmin, inmax, num):
    return max(inmin, min(inmax, num))


def locToGlob(loc_p, orig_p, rad):
    """
    :param loc_p: rel coordinates to point
    :param orig_p: body origin point in global coords
    :param rad: rotation of the body in radians
    :return:
    """

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


# program start
if __name__ == "__main__":
    while True:
        spotTurn(360)
        input()
        # mainLoop()
