runRobot = True

import math
import time
if runRobot:
    from pyax12.connection import Connection
    import regMove
# import sys
from enum import Enum
# import threading
# import multiprocessing
import curves

# import smbus

# AX-12A motor connection
if runRobot:
    serial_connection = Connection(port="/dev/ttyAMA0", rpi_gpio=True, baudrate=1000000)

lenA = 18.5  # length of shoulder motor1 to shoulder motor 2
lenB = 83  # length of upper arm
lenC = 120  # length of lower arm

width = 110  # width of robot
height = 183  # length of robot

# amount of moves for every step
stepsPerCycle = 1
# amount of time taken for every move (seconds)
timePerCycle = 0.2
# timePerCycle = 1

stdDelay = 0.001

# height of walk line
walkHeight = 90
# distance of steps from body
lineDist = 120
# how much to lift leg to step
liftHeight = 40
# amount to tilt before stepping
tiltHeight = 20

# leg max outwards reach
outX = 120
outDist = math.sqrt(outX**2 + lineDist**2)
# leg max inwards reach
inX = 50
inDist = math.sqrt(inX**2 + lineDist**2)

order = [0, 3, 1, 2]  # order to take steps

# program variables V V V
opposites = [3, 2, 1, 0]  # opposites of every leg

# empty array to store instructions (old)
moves = [
    [] for it in opposites
]

# initial starting coordinates (old)
coords = [
    [outX, lineDist, -walkHeight],
    [outX, lineDist, -walkHeight],
    [inX, lineDist, -walkHeight],
    [inX, lineDist, -walkHeight]
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
# radius = 30*10  # millimeters * 10 = centimeters
# driveCurves = [curves.quadBezier((0, 0), (radius, 0), (radius, -radius)),
#                curves.quadBezier((radius, -radius), (radius, -radius*2), (0, -radius*2)),
#                curves.quadBezier((0, -radius*2), (-radius, -radius*2), (-radius, -radius)),
#                curves.quadBezier((-radius, -radius), (-radius, 0), (0, 0))]

radius = 100 * 10
driveCurves = [curves.quadBezier((0, 0), (radius, 0), (radius * 2, 0))]

# legPos = [[height / 2 + outX, width / 2 + lineDist],
#           [height / 2 - inX + (outX + inX) * 1 / 3, -width / 2 - lineDist],
#           [-height / 2 + inX - (outX + inX) * 1 / 3, width / 2 + lineDist],
#           [-height / 2 - outX, -width / 2 - lineDist]]  # initial positions of legs

legPos = [[height / 2 - inX, width / 2 + lineDist],
          [height / 2 + outX - (outX + inX) * 1 / 3, -width / 2 - lineDist],
          [-height / 2 + inX, width / 2 + lineDist],
          [-height / 2 - outX + (outX + inX) * 1 / 3, -width / 2 - lineDist]]  # initial positions of legs

maxLegReach = 170

refLeg = order[0]
refFlag = True

lastMoved = False
# cornerAng = [math.atan2(width / 2, height / 2),
#              math.atan2(-width / 2, height / 2),
#              math.atan2(width / 2, -height / 2),
#              math.atan2(-width / 2, -height / 2)]

cornerPoint = [[height / 2, width / 2],
               [height / 2, -width / 2],
               [-height / 2, width / 2],
               [-height / 2, -width / 2]]

moveCountdown = [-1, -1, -1, -1]


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
    global lastMoved, cornerPoint, refFlag

    for i in range(len(coords)):
        moveLegs(calcRots(coords[i], i), i)

    time.sleep(stdDelay)
    if runRobot:
        regMove.actAll(serial_connection)

    dPoints = generate()

    while True:
        for posInd in range(0, len(dPoints), 2):
            # print(i)

            bodyX = dPoints[posInd][0]
            bodyY = dPoints[posInd][1]
            bodyAng = dPoints[posInd + 1]

            bodyCorners = [[], [], [], []]  # topLeft, topRight, botLeft, botRight

            for ind in range(4):
                bodyCorners[ind] = locToGlob(cornerPoint[ind], [bodyX, bodyY], bodyAng)

            legMoved = -1

            foundPos = None

            for curLeg in range(0, 4):
                if moveCountdown[curLeg] == 0:
                    foundPos = findNext(dPoints, bodyCorners[curLeg], posInd, curLeg)
                    legMoved = curLeg
                if moveCountdown[curLeg] >= 0:
                    moveCountdown[curLeg] -= 1

            tmpDist = dist(legPos[refLeg], bodyCorners[refLeg])

            if (not refFlag) and tmpDist < inDist and tmpDist < outDist:
                refFlag = True

            if tmpDist > (inDist if refLeg <= 1 else outDist) and refFlag:  # test if reference leg is outside of its maximum range
                print("out of range")
                foundPos = findNext(dPoints, bodyCorners[refLeg], posInd, refLeg)

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
                        moveCountdown[order[moveTimerLeg]] = int(amTill * moveTimerLeg / 4)
                    moveCountdown[refLeg] = -1

                legMoved = refLeg

            relLegPos = [[], [], [], []]

            for i in range(4):  # calculate leg positions relative to body
                relLegPos[i] = globToLoc(legPos[i], bodyCorners[i], bodyAng, i, -walkHeight)

            if lastMoved and legMoved == -1:  # un-tilt
                lastMoved = False
                for leg in range(4):
                    execInst(inst(inst_type.abs, relLegPos[leg]), leg)

                time.sleep(stdDelay)
                if runRobot:
                    regMove.actAll(serial_connection)
                time.sleep(stdDelay)

                time.sleep(timePerCycle)

            if legMoved != -1:  # a leg has reached its maximum, move it
                print("leg moved: " + str(legMoved))
                print("timers: " + str(moveCountdown))
                lastMoved = True
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

                time.sleep(stdDelay)
                if runRobot:
                    regMove.actAll(serial_connection)
                time.sleep(stdDelay)

                time.sleep(timePerCycle)


def findNext(dPoints, cornerCoord, start, leg):
    searchDist = (width / 2 + lineDist)
    relPos = [0, searchDist * ((-1) ** leg)]  # flip to right side (legs 1 and 3)

    back = inDist if leg <= 1 else outDist
    front = outDist if leg <= 1 else inDist

    foundInd = len(dPoints) - 2

    flag = False

    for j in range(start, len(dPoints), 2):
        testX = dPoints[j][0]
        testY = dPoints[j][1]
        testAng = dPoints[j + 1]

        globLegTestPos = locToGlob(relPos, [testX, testY], testAng)
        # locLegTestPos = globToLoc([testX, testY], relPos, testAng)  # find leg (test) pos relative to body corner

        testDist = dist(globLegTestPos, cornerCoord)

        if flag and testDist > front:
            foundInd = j - 2
            break

        if (not flag) and testDist < back and testDist < front:
            flag = True

    foundX = dPoints[foundInd][0]
    foundY = dPoints[foundInd][1]
    foundAng = dPoints[foundInd + 1]

    return locToGlob(relPos, [foundX, foundY], foundAng)


def generate():
    dirTmp = []
    curOver = 0

    for pInd in range(len(driveCurves)):

        curve = driveCurves[pInd]
        pos, ang = curve.getPosDir(driveAcc, curOver)
        curOver = (curOver + curve.getLength()) % driveAcc

        for i in range(len(pos)):
            dirTmp.append(pos[i])
            dirTmp.append(ang[i])

    return dirTmp


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
        for i in range(0, 3):
            if ins.args[i] is not None:
                coords[leg][i] = ins.args[i]
        moveLegs(calcRots(coords[leg], leg), leg)

    elif ins.type == inst_type.rel:
        for i in range(0, 3):
            if ins.args[i] is not None:
                coords[leg][i] += ins.args[i]
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
    mainLoop()
