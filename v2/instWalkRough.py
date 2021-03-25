import math
import time
from pyax12.connection import Connection
import regMove
import sys
from enum import Enum
import threading

class inst_type (Enum):
    nul = 0 # do nothing
    abs = 1 # go to absolute position
    rel = 2 # go to position relative to current position
    gnd = 3 # find the ground
    cal = 4 # calibrate legs to be more reasonable

class inst:
    type = inst_type.nul
    args = []
    # args for each type:
    # 0:
    # 1: x, y, z
    # 2: +x, +y, +z
    # 3:
    # 4:


    def __init__(self, typeIn, argsIn):
        self.type = typeIn
        self.args = argsIn



serial_connection = Connection(port="/dev/ttyAMA0", rpi_gpio=True, baudrate=1000000)

lenA = 18.5 # lenth of shoulder motor1 to shoulder motor 2, projected onto the ground plane
lenB = 83 # lenth of upper arm
lenC = 120 # length of lower arm

stepsPerCycle = 1 # amount of steps for every cycle
timePerCycle = 0.1 # amount of time taken for every cycle (seconds)
gndStepSize = 4 # amount to move every step of findground
gndThresh = 7 #

walkHeight = 90 # height of walk line
backLine = 120 # dist from walk line
frontLine = 120 # dist from walk line
liftHeight = 80

tiltHeight = 20 # amount to drop to tilt

slope = 0
robLen = 185

outX = 100
inX = 30

amTasks = 0
driving = False

order = [0,3,1,2]

opposites = [3,2,1,0]

moves = [
[] for it in opposites
]

tmpAmMove = (outX+inX) / (len(order))

coords = [
[outX, frontLine, -walkHeight],
[outX, frontLine, -walkHeight],
[inX, backLine, -walkHeight],
[inX, backLine, -walkHeight]
]

legId = [
[1,2,3],
[10,11,12],
[7,8,9],
[4,5,6]
]

limits = [
[-40,20],
[-40,50],
[-75,105]
]

rotOffs = [0,0,-20]



def getDists ():
    dists = [[[] for it in order] for it2 in opposites]

    for i in range(len(order)):
        j = (i+1)% len(order)
        counter = 1
        for k in range(len(order)):
            dists[order[i]][j] = counter
            if order[j] == order[i]:
                counter = 0
            j = (j + 1) % len(order)
            counter += 1

    return dists

def generate ():
    dists = getDists()

    print(dists)

    for i in range(len(order)):
        for j in range(len(moves)):

            curMovePos = (outX if j <= 1 else inX) - dists[j][i] * ((outX+inX) / (len(order)))
            amMove = ((outX+inX) / (len(order))) / 4

            # move into place, tilt the robot, op does nothing, middle legs lift, lifting leg lifts more
            # lift the lifting leg, move other legs back
            # move lifting leg, others back
            # lower lifting leg, set ground level
            # calibrate other legs

            if j == order[i]: #lift
                moves[j].append(inst(inst_type.rel, [-amMove, 0, -tiltHeight*2])) # move + tilt
                moves[j].append(inst(inst_type.rel, [0, 0, liftHeight])) # lift
                moves[j].append(inst(inst_type.abs, [(outX if j <= 1 else inX), None, None])) # fwd
                moves[j].append(inst(inst_type.gnd, [])) # ground
                moves[j].append(inst(inst_type.cal, [])) # calibration

            elif j == opposites[order[i]]: #drop
                moves[j].append(inst(inst_type.rel, [-amMove, 0, tiltHeight])) # move + tilt
                moves[j].append(inst(inst_type.rel, [-amMove, 0, 0])) # lift
                moves[j].append(inst(inst_type.rel, [-amMove, 0, 0])) # fwd
                moves[j].append(inst(inst_type.rel, [-amMove, 0, -tiltHeight])) # ground
                moves[j].append(inst(inst_type.cal, [])) # calibration
            else: #do
                moves[j].append(inst(inst_type.rel, [-amMove, 0, -tiltHeight])) # move + tilt
                moves[j].append(inst(inst_type.rel, [-amMove, 0, 0])) # lift
                moves[j].append(inst(inst_type.rel, [-amMove, 0, 0])) # fwd
                moves[j].append(inst(inst_type.rel, [-amMove, 0, tiltHeight])) # ground
                moves[j].append(inst(inst_type.cal, [])) # calibration



def calcRots (xyz, leg):
    x = xyz[0] * ((-1)**leg) # invert legs 2 and 4 (int 1 and 3)
    y = xyz[1]
    z = xyz[2]

    xyAng = math.degrees(math.atan(x/y)) if y!=0 else (90 if x >= 0 else -90) # z-rot of entire leg, from center (90) (shoulder side to side)
    xyLen = math.sqrt((x**2) + (y**2)) - lenA # length of leg vector projected from Z to ground
    trueLen = math.sqrt((xyLen**2) + (z**2)) # actual length of leg vector

    zXYAng = math.degrees(math.atan(xyLen/abs(z))) if z!=0 else 0 # angle of leg vector (shoulder part 1)

    if(trueLen >= lenB + lenC):
        angA = 0
        angB = 180
    else:
        angA = math.degrees(math.acos(((trueLen**2) + (lenB**2) - (lenC**2)) / (2*trueLen*lenB))) # angle of top arm from vector (shoulder part 2)
        angB = math.degrees(math.acos(((lenB**2) + (lenC**2) - (trueLen**2)) / (2*lenB*lenC))) # angle of bottom arm from top arm (elbow)

    return [(-xyAng) + rotOffs[0], (90-zXYAng-angA) + rotOffs[1], (angB-90) + rotOffs[2]] # angle at elbow has to be inverted


def findGround (leg) :
    global amTasks, driving, coords

    while driving == True:
        time.sleep(0.01)

    print("g start")
    while True:
        pres1 = serial_connection.get_present_position(legId[leg][1])
        goal1 = serial_connection.get_goal_position(legId[leg][1])

        pres2 = serial_connection.get_present_position(legId[leg][2])
        goal2 = serial_connection.get_goal_position(legId[leg][2])

        err1 = pres1-goal1
        err2 = pres2-goal2

        print(str(err1) + " 1 <==> 2 " + str(err2))

        if abs(err1) > gndThresh or abs(err2) > gndThresh:
            break

        coords[leg][2] -= gndStepSize

        moveLegs(calcRots(coords[leg], leg), leg)
        time.sleep(0.01)
        regMove.actAll(serial_connection)
        time.sleep(0.1)

    amTasks -= 1


def calibrateLegs ():
    avg = 0
    for i in coords:
        avg -= i[2]

    diff = (walkHeight*4 - avg)/4.0

    print("diff " + str(diff))

    for i in range(4):
        coords[i][2] -= diff
        moveLegs(calcRots(coords[i], i), i)

    regMove.actAll(serial_connection)


def execInst (ins, leg):
    global amTasks, driving

    if(ins.type == inst_type.abs):
        for i in range(0, 3):
            if(ins.args[i] != None):
                coords[leg][i] = ins.args[i]
        moveLegs(calcRots(coords[leg], leg), leg)

    elif (ins.type == inst_type.rel):
        for i in range(0, 3):
            if(ins.args[i] != None):
                coords[leg][i] += ins.args[i]
        moveLegs(calcRots(coords[leg], leg), leg)

    elif (ins.type == inst_type.gnd):
        amTasks += 1
        t1 = threading.Thread(target = findGround, args = (leg,))
        t1.start()

def clamp (inmin, inmax, num):
    return max(inmin, min(inmax, num))

def driveLeg (leg, motor, rot):
    # if(rot*(-1 if motor == 0 and leg == 1 or leg == 2 else 1) < limits[motor][0] or rot*(-1 if motor == 0 and leg == 1 or leg == 2 else 1) > limits[motor][1]):
    #     print("Leg " + str(leg) + "-" + str(motor) + " angle out of bounds, rot= " + str(rot))
    #     raise ValueError

    toDrive = clamp(limits[motor][0], limits[motor][1], rot*(-1 if motor == 0 and leg == 1 or leg == 2 else 1))*(-1 if motor == 0 and leg == 1 or leg == 2 else 1)

    regMove.regMove(serial_connection, legId[leg][motor], toDrive, speed=512, degrees=True)

def moveLegs (rots,leg): # one leg per group of  (4 ports)

    driveLeg(leg, 0, rots[0])

    time.sleep(0.01)

    if(leg == 0 or leg == 3):
        driveLeg(leg, 1, rots[1])
    else:
        driveLeg(leg, 1, -rots[1])
    time.sleep(0.01)

    if(leg == 0 or leg == 3):
        driveLeg(leg, 2, rots[2])
    else:
        driveLeg(leg, 2, -rots[2])


def mainLoop():
    global amTasks, driving

    counter = 0
    # moveLegs(steps[0][0], 0)
    # moveLegs(steps[1][0], 1)
    # moveLegs(steps[2][0], 2)
    # moveLegs(steps[3][0], 3)

    for i in range(len(coords)):
        moveLegs(calcRots(coords[i], i), i)

    time.sleep(0.01)
    regMove.actAll(serial_connection)

    time.sleep(1)
    while True:
        driving = True

        cal = False

        for i in range(4):
            if(moves[i][counter].type == inst_type.cal):
                cal = True
                break

        if not cal:
            execInst(moves[0][counter], 0)
            execInst(moves[1][counter], 1)
            execInst(moves[2][counter], 2)
            execInst(moves[3][counter], 3)

            time.sleep(0.01)
            regMove.actAll(serial_connection)
            time.sleep(0.01)
        else:
            # print("calibration not yet implemented")
            calibrateLegs()

        driving = False
        print("drive end")

        time.sleep(timePerCycle / stepsPerCycle)

        # print(str(amTasks) + " tasks")

        while amTasks > 0:
            time.sleep(0.01)

        print("waiting end")

        counter = (counter+1)%len(moves[0])

generate()
mainLoop()
