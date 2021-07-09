import math
import time
from pyax12.connection import Connection
import regMove
import sys
from enum import Enum
import threading
import multiprocessing
import curves

# import smbus

# AX-12A motor connection
serial_connection = Connection(port="/dev/ttyAMA0", rpi_gpio=True, baudrate=1000000)

lenA = 18.5  # lenth of shoulder motor1 to shoulder motor 2
lenB = 83  # lenth of upper arm
lenC = 120  # length of lower arm

width = 110  # width of robot
height = 183  # length of robot

# amount of moves for every step
stepsPerCycle = 1
# amount of time taken for every move (seconds)
timePerCycle = 0.01 * 6 * 2

stdDelay = 0.001

# height of walk line
walkHeight = 90
# distance of steps from body
lineDist = 150
# how much to lift leg to step
liftHeight = 40
# amount to tilt before stepping
tiltHeight = 10

# leg max outwards reach
outX = 120
# leg max inwards reach
inX = 50

order = [0, 3, 1, 2]  # order to take steps (old)

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

# rotation offsets for leg motos
rotOffs = [0, 0, -20]

driveAcc = 1  # accuracy of driving for curves (mm)
radius = 100*10  # millimeters * 10 = centimeters
driveCurves = [curves.quadBezier((0, 0), (radius, 0), (radius, radius)),
               curves.quadBezier((radius, radius), (radius, radius*2), (0, radius*2)),
               curves.quadBezier((0, radius*2), (-radius, radius*2), (-radius, radius)),
               curves.quadBezier((-radius, radius), (-radius, 0), (0, 0))]

legPos = [[height/2+outX, width/2+lineDist], [height/2-inX+(outX+inX)*1/4, -width/2-lineDist], [-height/2+inX-(outX+inX)*1/4, width/2+lineDist], [-height/2-outX, -width/2-lineDist]]  # initial positions of legs
maxLegReach = 120


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

    for i in range(len(coords)):
        moveLegs(calcRots(coords[i], i), i)

    time.sleep(stdDelay)
    regMove.actAll(serial_connection)

    dPoints = generate()

    while True:
        for i in range(0, len(dPoints), 2):
            bodyX = dPoints[i][0]
            bodyY = dPoints[i][1]
            bodyAng = dPoints[i + 1]

            r = math.sqrt((width / 2) ** 2 + (height / 2) ** 2)

            tmpAng = math.atan2(width / 2, height / 2)
            topLeft = [r * math.cos(tmpAng + bodyAng) + bodyX, r * math.sin(tmpAng + bodyAng) + bodyY]

            tmpAng = math.atan2(-width / 2, height / 2)
            topRight = [r * math.cos(tmpAng + bodyAng) + bodyX, r * math.sin(tmpAng + bodyAng) + bodyY]

            tmpAng = math.atan2(width / 2, -height / 2)
            botLeft = [r * math.cos(tmpAng + bodyAng) + bodyX, r * math.sin(tmpAng + bodyAng) + bodyY]

            tmpAng = math.atan2(-width / 2, -height / 2)
            botRight = [r * math.cos(tmpAng + bodyAng) + bodyX, r * math.sin(tmpAng + bodyAng) + bodyY]

            # print(str(dist(legPos[0], topLeft)) + "    (" + str(legPos[0]) + " and " + str(topLeft) + ")")
            # print(str(topLeft) + " " + str(topRight) + " " + str(botLeft) + " " + str(botRight))

            legMoved = -1

            foundPos = None

            if dist(legPos[0], topLeft) > maxLegReach:
                print("too long")
                for j in range(i, len(dPoints), 2):
                    testX = dPoints[j][0]
                    testY = dPoints[j][1]
                    testAng = dPoints[j + 1]

                    testPos = [testX - (width / 2 + lineDist) * math.sin(testAng),
                               testY + (width / 2 + lineDist) * math.cos(testAng)]

                    if dist(testPos, topLeft) > maxLegReach:
                        foundX = dPoints[j - 2][0]
                        foundY = dPoints[j - 2][1]
                        foundAng = dPoints[j - 2 + 1]

                        foundPos = [foundX - (width / 2 + lineDist) * math.sin(foundAng),
                                    foundY + (width / 2 + lineDist) * math.cos(foundAng)]

                        legMoved = 0

                        break

            elif dist(legPos[1], topRight) > maxLegReach:
                print("too long")
                for j in range(i, len(dPoints), 2):
                    testX = dPoints[j][0]
                    testY = dPoints[j][1]
                    testAng = dPoints[j + 1]

                    testPos = [testX + (width / 2 + lineDist) * math.sin(testAng),
                               testY - (width / 2 + lineDist) * math.cos(testAng)]

                    if dist(testPos, topRight) > maxLegReach:
                        foundX = dPoints[j - 2][0]
                        foundY = dPoints[j - 2][1]
                        foundAng = dPoints[j - 2 + 1]

                        foundPos = [foundX + (width / 2 + lineDist) * math.sin(foundAng),
                                    foundY - (width / 2 + lineDist) * math.cos(foundAng)]

                        legMoved = 1

                        break

            elif dist(legPos[2], botLeft) > maxLegReach:
                print("too long")
                for j in range(i, len(dPoints), 2):
                    testX = dPoints[j][0]
                    testY = dPoints[j][1]
                    testAng = dPoints[j + 1]

                    testPos = [testX - (width / 2 + lineDist) * math.sin(testAng),
                               testY + (width / 2 + lineDist) * math.cos(testAng)]

                    if dist(testPos, botLeft) > maxLegReach:
                        foundX = dPoints[j - 2][0]
                        foundY = dPoints[j - 2][1]
                        foundAng = dPoints[j - 2 + 1]

                        foundPos = [foundX - (width / 2 + lineDist) * math.sin(foundAng),
                                    foundY + (width / 2 + lineDist) * math.cos(foundAng)]

                        legMoved = 2

                        break

            elif dist(legPos[3], botRight) > maxLegReach:
                print("too long")
                for j in range(i, len(dPoints), 2):
                    testX = dPoints[j][0]
                    testY = dPoints[j][1]
                    testAng = dPoints[j + 1]

                    testPos = [testX + (width / 2 + lineDist) * math.sin(testAng),
                               testY - (width / 2 + lineDist) * math.cos(testAng)]

                    if dist(testPos, botRight) > maxLegReach:
                        foundX = dPoints[j - 2][0]
                        foundY = dPoints[j - 2][1]
                        foundAng = dPoints[j - 2 + 1]

                        foundPos = [foundX + (width / 2 + lineDist) * math.sin(foundAng),
                                    foundY - (width / 2 + lineDist) * math.cos(foundAng)]

                        legMoved = 3

                        break

            relLegPos = [[], [], [], []]
            
            tmpAng = math.atan2(legPos[0][1]-topLeft[1], legPos[0][0]-topLeft[0])
            relDist = dist(legPos[0], topLeft)
            relLegPos[0] = [relDist*math.cos(bodyAng-tmpAng), relDist*math.sin(bodyAng-tmpAng), -walkHeight]

            tmpAng = math.atan2(legPos[1][1] - topRight[1], legPos[1][0] - topRight[0])
            relDist = dist(legPos[1], topRight)
            relLegPos[1] = [relDist * math.cos(bodyAng - tmpAng), - relDist * math.sin(bodyAng - tmpAng), -walkHeight]  # invert y on right side

            tmpAng = math.atan2(legPos[2][1] - botLeft[1], legPos[2][0] - botLeft[0])
            relDist = dist(legPos[2], botLeft)
            relLegPos[2] = [relDist * math.cos(bodyAng - tmpAng), relDist * math.sin(bodyAng - tmpAng), -walkHeight]

            tmpAng = math.atan2(legPos[3][1] - botRight[1], legPos[3][0] - botRight[0])
            relDist = dist(legPos[3], botRight)
            relLegPos[3] = [relDist * math.cos(bodyAng - tmpAng), - relDist * math.sin(bodyAng - tmpAng), -walkHeight]  # invert y on right side

            # atan2 posx-x, posy-y to get angle
            # car angle - angle
            # sin and cos to go back to coords
            # invert Y coord on right side

            if legMoved == -1:
                # move none
                for leg in range(4):
                    execInst(inst(inst_type.abs, relLegPos[leg]), leg)

            else:
                for leg in range(4): # move back and tilt
                    if leg == legMoved:
                        execInst(inst(inst_type.abs, [relLegPos[leg][0], relLegPos[leg][1], -walkHeight-tiltHeight]), leg)
                    elif leg == opposites[legMoved]:
                        execInst(inst(inst_type.abs, [relLegPos[leg][0], relLegPos[leg][1], -walkHeight+tiltHeight]), leg)
                    else:
                        execInst(inst(inst_type.abs, relLegPos[leg]), leg)

                time.sleep(stdDelay)
                regMove.actAll(serial_connection)
                time.sleep(stdDelay)

                time.sleep(timePerCycle / stepsPerCycle)

                for leg in range(4):  # stay and move lifting leg forwards
                    if leg == legMoved:

                        cornerCoords = None

                        if legMoved == 0:
                            cornerCoords = topLeft
                        elif legMoved == 1:
                            cornerCoords = topRight
                        elif legMoved == 2:
                            cornerCoords = botLeft
                        elif legMoved == 3:
                            cornerCoords = botRight

                        # update relLegPos with forward pos
                        tmpAng = math.atan2(foundPos[1] - cornerCoords[1], foundPos[0] - cornerCoords[0])
                        relDist = dist(foundPos, cornerCoords)
                        relLegPos[legMoved] = [relDist * math.cos(bodyAng - tmpAng), - relDist * math.sin(bodyAng - tmpAng), (-1)**leg * walkHeight]

                        # update legPos with forward pos
                        legPos[legMoved] = foundPos

                        execInst(inst(inst_type.abs, [relLegPos[leg][0], relLegPos[leg][1], -walkHeight+liftHeight]), leg)
                    elif leg == opposites[legMoved]:
                        execInst(inst(inst_type.abs, [relLegPos[leg][0], relLegPos[leg][1], -walkHeight+tiltHeight]), leg)
                    else:
                        execInst(inst(inst_type.abs, relLegPos[leg]), leg)

            time.sleep(stdDelay)
            regMove.actAll(serial_connection)
            time.sleep(stdDelay)

            time.sleep(timePerCycle / stepsPerCycle)


def generate():
    dirTmppp = []

    curOver = 0

    for pInd in range(len(driveCurves)):

        curv = driveCurves[pInd]

        pos, ang = curv.getPosDir(driveAcc, curOver)

        curOver = (curOver + curv.getLength()) % driveAcc

        for i in range(len(pos)):
            dirTmppp.append(pos[i])
            dirTmppp.append(ang[i])

    return dirTmppp


def dist(xy1, xy2):
    return math.sqrt((xy2[0] - xy1[0]) ** 2 + (xy2[1] - xy1[1]) ** 2)


# def generateSteps(dPoints):
#     maxDist = 10
#     legPos = [[0, 0], [0, 0], [0, 0], [0, 0]]
#     width = 2
#     height = 5
#     legDist = 5
#
#     res = [[], [], [], []]
#
#     for i in range(0, len(dPoints), 2):
#         # print(i)
#         x = dPoints[i][0]
#         y = dPoints[i][1]
#         ang = dPoints[i + 1]
#
#         r = math.sqrt((width / 2) ** 2 + (height / 2) ** 2)
#
#         tmpAng = math.atan2(width / 2, height / 2)
#         topLeft = [r * math.cos(tmpAng + ang) + x, r * math.sin(tmpAng + ang) + y]
#
#         tmpAng = math.atan2(-width / 2, height / 2)
#         topRight = [r * math.cos(tmpAng + ang) + x, r * math.sin(tmpAng + ang) + y]
#
#         tmpAng = math.atan2(width / 2, -height / 2)
#         botLeft = [r * math.cos(tmpAng + ang) + x, r * math.sin(tmpAng + ang) + y]
#
#         tmpAng = math.atan2(-width / 2, -height / 2)
#         botRight = [r * math.cos(tmpAng + ang) + x, r * math.sin(tmpAng + ang) + y]
#
#         # print(str(dist(legPos[0], topLeft)) + "    (" + str(legPos[0]) + " and " + str(topLeft) + ")")
#         # print(str(topLeft) + " " + str(topRight) + " " + str(botLeft) + " " + str(botRight))
#
#         if dist(legPos[0], topLeft) > maxDist:
#             print("too long")
#             for j in range(i, len(dPoints), 2):
#                 xx = dPoints[j][0]
#                 yy = dPoints[j][1]
#                 angg = dPoints[j + 1]
#
#                 poss = [xx - (width / 2 + legDist) * math.sin(angg),
#                         yy + (width / 2 + legDist) * math.cos(angg)]
#
#                 # print("testing " + str(dist(poss, topLeft)))
#
#                 if dist(poss, topLeft) > maxDist:
#                     xxx = dPoints[j - 2][0]
#                     yyy = dPoints[j - 2][1]
#                     anggg = dPoints[j - 2 + 1]
#
#                     posss = [xxx - (width / 2 + legDist) * math.sin(anggg),
#                              yyy + (width / 2 + legDist) * math.cos(anggg)]
#
#                     legPos[0] = posss
#                     res[0].append(posss)
#
#                     print("found, setting leg pos to: " + str(posss))
#
#                     break
#
#         elif dist(legPos[1], topRight) > maxDist:
#             print("too long")
#             for j in range(i, len(dPoints), 2):
#                 xx = dPoints[j][0]
#                 yy = dPoints[j][1]
#                 angg = dPoints[j + 1]
#
#                 poss = [xx + (width / 2 + legDist) * math.sin(angg),
#                         yy - (width / 2 + legDist) * math.cos(angg)]
#
#                 # print("testing " + str(dist(poss, topRight)))
#
#                 if dist(poss, topRight) > maxDist:
#                     xxx = dPoints[j - 2][0]
#                     yyy = dPoints[j - 2][1]
#                     anggg = dPoints[j - 2 + 1]
#
#                     posss = [xxx + (width / 2 + legDist) * math.sin(anggg),
#                              yyy - (width / 2 + legDist) * math.cos(anggg)]
#
#                     legPos[1] = posss
#                     res[1].append(posss)
#
#                     print("found, setting leg pos to: " + str(posss))
#
#                     break
#
#         elif dist(legPos[2], botLeft) > maxDist:
#             print("too long")
#             for j in range(i, len(dPoints), 2):
#                 xx = dPoints[j][0]
#                 yy = dPoints[j][1]
#                 angg = dPoints[j + 1]
#
#                 poss = [xx - (width / 2 + legDist) * math.sin(angg),
#                         yy + (width / 2 + legDist) * math.cos(angg)]
#
#                 # print("testing " + str(dist(poss, topRight)))
#
#                 if dist(poss, botLeft) > maxDist:
#                     xxx = dPoints[j - 2][0]
#                     yyy = dPoints[j - 2][1]
#                     anggg = dPoints[j - 2 + 1]
#
#                     posss = [xxx - (width / 2 + legDist) * math.sin(anggg),
#                              yyy + (width / 2 + legDist) * math.cos(anggg)]
#
#                     legPos[2] = posss
#                     res[2].append(posss)
#
#                     print("found, setting leg pos to: " + str(posss))
#
#                     break
#
#         elif dist(legPos[3], botRight) > maxDist:
#             print("too long")
#             for j in range(i, len(dPoints), 2):
#                 xx = dPoints[j][0]
#                 yy = dPoints[j][1]
#                 angg = dPoints[j + 1]
#
#                 poss = [xx + (width / 2 + legDist) * math.sin(angg), yy - (width / 2 + legDist) * math.cos(angg)]
#
#                 # print("testing " + str(dist(poss, topRight)))
#
#                 if dist(poss, botRight) > maxDist:
#                     xxx = dPoints[j - 2][0]
#                     yyy = dPoints[j - 2][1]
#                     anggg = dPoints[j - 2 + 1]
#
#                     posss = [xxx + (width / 2 + legDist) * math.sin(anggg),
#                              yyy - (width / 2 + legDist) * math.cos(anggg)]
#
#                     legPos[3] = posss
#                     res[3].append(posss)
#
#                     print("found, setting leg pos to: " + str(posss))
#
#                     break
#
#     return res


# def render(points):
#     tmppp, rendPoints, dirPoints = generate(points)
#     res = generateSteps(dirPoints)


# # Generate list of instructions
# def generate():
#     dists = getDists()
#
#     for i in range(len(order)):
#         for j in range(len(moves)):
#
#             curMovePos = (outX if j <= 1 else inX) - dists[j][i] * ((outX + inX) / (len(order)))
#             amMove = ((outX + inX) / (len(order))) / 2
#
#             # move into place, tilt the robot, op does nothing, middle legs lift, lifting leg lifts more
#             # lift the lifting leg, move other legs back
#             # move lifting leg, others back
#             # lower lifting leg, set ground level
#             # calibrate other legs
#
#             if j == order[i]:
#                 moves[j].append(
#                     inst(inst_type.abs, [curMovePos + amMove * 1, lineDist, -walkHeight - tiltHeight]))  # move + tilt
#                 moves[j].append(inst(inst_type.abs,
#                                      [(outX if j <= 1 else inX), lineDist, -walkHeight + liftHeight]))  # lift and fwd
#
#             elif j == opposites[order[i]]:
#                 moves[j].append(
#                     inst(inst_type.abs, [curMovePos + amMove * 1, lineDist, -walkHeight + tiltHeight]))  # move + tilt
#                 moves[j].append(
#                     inst(inst_type.abs, [curMovePos + amMove * 0, lineDist, -walkHeight + tiltHeight]))  # lift and fwd
#             else:
#                 moves[j].append(inst(inst_type.abs, [curMovePos + amMove * 1, lineDist, -walkHeight]))  # move + tilt
#                 moves[j].append(inst(inst_type.abs, [curMovePos + amMove * 0, lineDist, -walkHeight]))  # lift and fwd


# def getDists():
#     dists = [[[] for it in order] for it2 in opposites]
#
#     for i in range(len(order)):
#         j = (i + 1) % len(order)
#         counter = 1
#         for k in range(len(order)):
#             dists[order[i]][j] = counter
#             if order[j] == order[i]:
#                 counter = 0
#             j = (j + 1) % len(order)
#             counter += 1
#
#     return dists


# Inverse Kinematics
def calcRots(xyz, leg):
    x = xyz[0] * ((-1) ** leg)  # invert x of legs 2 and 4 (int 1 and 3) (right side)
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

    regMove.regMove(serial_connection, legId[leg][motor], toDrive, speed=1023, degrees=True)


# get amount of digits in a number
# def getDigs(inn):
#     return int(math.log10(abs(int(inn))) if inn != 0 else 0) + 1 + (1 if inn < 0 else 0) + 2


# prints all essential information
# def printInterface():
#     global coords
#
#     buffer = 10
#
#     print("-" * ((1 + 4 + buffer) * 2 + 1))
#     # digsone = int(math.log10(int(coords[0][0])))+1
#     # digstwo = int(math.log10(int(coords[1][0])))+1
#     print("|", (" " * (4 + (buffer - getDigs(coords[0][0])))), "X: %.2f" % coords[0][0], " | ", (" " * (4 + (buffer - getDigs(coords[1][0])))), "X: %.2f" % coords[1][0], " |")
#     # digsone = int(math.log10(int(coords[0][1])))+1
#     # digstwo = int(math.log10(int(coords[1][1])))+1
#     print("|", "0: ", (" " * (buffer - getDigs(coords[0][1]))), "Y: %.2f" % coords[0][1], " | ", " 1: ", (" " * (buffer - getDigs(coords[1][1]))), "Y: %.2f" % coords[1][1], " |")
#     # digsone = int(math.log10(int(coords[0][2])))+1
#     # digstwo = int(math.log10(int(coords[1][2])))+1
#     print("|", (" " * (4 + (buffer - getDigs(coords[0][2])))), "Z: %.2f" % coords[0][2], " | ", (" " * (4 + (buffer - getDigs(coords[1][2])))), "Z: %.2f" % coords[1][2], " |")
#
#     print("-" * ((1 + 4 + buffer) * 2 + 1))
#     # digsone = int(math.log10(int(coords[2][0])))+1
#     # digstwo = int(math.log10(int(coords[3][0])))+1
#     print("|", (" " * (4 + (buffer - getDigs(coords[2][0])))), "X: %.2f" % coords[2][0], " | ", (" " * (4 + (buffer - getDigs(coords[3][0])))), "X: %.2f" % coords[3][0], " |")
#     # digsone = int(math.log10(int(coords[2][1])))+1
#     # digstwo = int(math.log10(int(coords[3][1])))+1
#     print("|", "2: ", (" " * (buffer - getDigs(coords[2][1]))), "Y: %.2f" % coords[2][1], " | ", " 3: ", (" " * (buffer - getDigs(coords[3][1]))), "Y: %.2f" % coords[3][1], " |")
#     # digsone = int(math.log10(int(coords[2][2])))+1
#     # digstwo = int(math.log10(int(coords[3][2])))+1
#     print("|", (" " * (4 + (buffer - getDigs(coords[2][2])))), "Z: %.2f" % coords[2][2], " | ", (" " * (4 + (buffer - getDigs(coords[3][2])))), "Z: %.2f" % coords[3][2], " |")
#
#     # print("-" * ((1 + 4 + buffer) * 2 + 1))
#     # print("  X: %.2f" % rotation[0], "----Y: %.2f" % rotation[1], "----Z: %.2f" % rotation[2])
#     # print("-" * ((1 + 4 + buffer) * 2 + 1))


# clamps num to be in between inmin and inmax
def clamp(inmin, inmax, num):
    return max(inmin, min(inmax, num))


# program start
generate()
mainLoop()
