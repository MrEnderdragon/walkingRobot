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

class inst:
    type = inst_type.nul
    args = []

    def __init__(self, typeIn, argsIn):
        self.type = typeIn
        self.args = argsIn



serial_connection = Connection(port="/dev/ttyAMA0", rpi_gpio=True, baudrate=1000000)

lenA = 18.5 # lenth of shoulder motor1 to shoulder motor 2, projected onto the ground plane
lenB = 83 # lenth of upper arm
lenC = 120 # length of lower arm

stepsPerCycle = 1 # amount of steps for every cycle
timePerCycle = 1 # amount of time taken for every cycle (seconds)
gndStepSize = 2 # amount to move every step of findground
gndThresh = 5 #

walkHeight = 90 # height of walk line
backLine = 150 # dist from walk line
frontLine = 150 # dist from walk line
liftHeight = 40

tiltHeight = 10 # amount to drop to tilt

slope = 0
robLen = 185

outX = 120
inX = 50

amTasks = 0
driving = False

order = [0,3,1,2]

opposites = [3,2,1,0]

moves = [
[] for it in opposites
]

coords = [
[0, frontLine, -walkHeight],
[0, frontLine, -walkHeight],
[0, backLine, -walkHeight],
[0, backLine, -walkHeight]
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
            amMove = ((outX+inX) / (len(order))) / 3

            yesOff = 0 if j <= 1 else 1

            if j == order[i]: #lift
                moves[j].append(inst(inst_type.abs, [curMovePos + amMove*2, frontLine if j <= 1 else backLine, -walkHeight-tiltHeight - yesOff*robLen*slope + (curMovePos + amMove*1)*slope ])) # drop
                moves[j].append(inst(inst_type.abs, [(outX if j <= 1 else inX), frontLine if j <= 1 else backLine, -walkHeight+liftHeight - yesOff*robLen*slope + (inX)*slope])) # lift+move
                moves[j].append(inst(inst_type.gnd,[]))# move

            elif j == opposites[order[i]]: #drop
                moves[j].append(inst(inst_type.abs, [curMovePos + amMove*2, frontLine if j <= 1 else backLine, -walkHeight+tiltHeight - yesOff*robLen*slope + (curMovePos + amMove*1)*slope])) # drop
                moves[j].append(inst(inst_type.abs, [curMovePos + amMove*1, frontLine if j <= 1 else backLine, -walkHeight+tiltHeight - yesOff*robLen*slope + (curMovePos + amMove*0)*slope])) # move
                moves[j].append(inst(inst_type.abs, [curMovePos + amMove*0, frontLine if j <= 1 else backLine, -walkHeight - yesOff*robLen*slope + (curMovePos + amMove*0)*slope])) # lower
            else: #do
                moves[j].append(inst(inst_type.abs, [curMovePos + amMove*2, frontLine if j <= 1 else backLine, -walkHeight - yesOff*robLen*slope + (curMovePos + amMove*1)*slope])) # drop
                moves[j].append(inst(inst_type.abs, [curMovePos + amMove*1, frontLine if j <= 1 else backLine, -walkHeight - yesOff*robLen*slope + (curMovePos + amMove*0)*slope]))# move
                moves[j].append(inst(inst_type.abs, [curMovePos + amMove*0, frontLine if j <= 1 else backLine, -walkHeight - yesOff*robLen*slope + (curMovePos + amMove*0)*slope]))# move


def calcRots (xyz, leg):
    x = xyz[0] * ((-1)**leg) # invert legs 2 and 4 (int 1 and 3)
    y = xyz[1]
    z = xyz[2]

    xyAng = math.degrees(math.atan(x/y)) if y!=0 else (90 if x >= 0 else -90) # z-rot of entire leg, from center (90) (shoulder side to side)
    xyLen = math.sqrt((x**2) + (y**2)) - lenA # length of leg vector projected from Z to ground
    trueLen = math.sqrt((xyLen**2d) + (z**2)) # actual length of leg vector

    zXYAng = math.degrees(math.atan(xyLen/abs(z))) # angle of leg vector (shoulder part 1)

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


def execInst (ins, leg):
    global amTasks, driving

    if(ins.type == inst_type.abs):
        for i in range(0, 3):
            coords[leg][i] = ins.args[i]
        moveLegs(calcRots(coords[leg], leg), leg)

    elif (ins.type == inst_type.rel):
        for i in range(0, 3):
            coords[leg][i] += ins.args[i]
        moveLegs(calcRots(coords[leg], leg), leg)

    elif (ins.type == inst_type.gnd):
        amTasks += 1
        t1 = threading.Thread(target = findGround, args = (leg,))
        t1.start()



def driveLeg (leg, motor, rot):
    if(rot*(-1 if motor == 0 and leg == 1 or leg == 2 else 1) < limits[motor][0] or rot*(-1 if motor == 0 and leg == 1 or leg == 2 else 1) > limits[motor][1]):
        print("Leg " + str(leg) + "-" + str(motor) + " angle out of bounds, rot= " + str(rot))
        raise ValueError
    regMove.regMove(serial_connection, legId[leg][motor], rot, speed=512, degrees=True)



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

    execInst(moves[0][0], 0)
    execInst(moves[1][0], 1)
    execInst(moves[2][0], 2)
    execInst(moves[3][0], 3)

    time.sleep(0.01)
    regMove.actAll(serial_connection)

    time.sleep(1)
    while True:
        driving = True
        execInst(moves[0][counter], 0)
        execInst(moves[1][counter], 1)
        execInst(moves[2][counter], 2)
        execInst(moves[3][counter], 3)

        time.sleep(0.01)
        regMove.actAll(serial_connection)
        time.sleep(0.01)

        driving = False
        print("drive end")

        time.sleep(timePerCycle / stepsPerCycle)

        print(str(amTasks))

        while amTasks > 0:
            time.sleep(0.01)

        print("wait end")

        counter = (counter+1)%len(moves[0])

generate()
mainLoop()
