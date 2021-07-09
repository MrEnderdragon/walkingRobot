import math
import time
from pyax12.connection import Connection
import regMove
import sys
from enum import Enum
import threading
import multiprocessing
import smbus

# AX-12A motor connection
serial_connection = Connection(port="/dev/ttyAMA0", rpi_gpio=True, baudrate=1000000)

lenA = 18.5 # lenth of shoulder motor1 to shoulder motor 2
lenB = 83 # lenth of upper arm
lenC = 120 # length of lower arm

# amount of moves for every step
stepsPerCycle = 1
# amount of time taken for every move (seconds)
timePerCycle = 0.01 *6 *2

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

order = [0,3,1,2] # order to take steps


# program variables V V V
opposites = [3,2,1,0] # opposites of every leg

# empty array to store instructions
moves = [
[] for it in opposites
]

# initial starting coordinates
coords = [
[outX, lineDist, -walkHeight],
[outX, lineDist, -walkHeight],
[inX, lineDist, -walkHeight],
[inX, lineDist, -walkHeight]
]

# motor ids
legId = [
[1,2,3],
[10,11,12],
[7,8,9],
[4,5,6]
]

# movement limits of leg motors
limits = [
[-40,20],
[-40,50],
[-75,105]
]

# rotation offsets for leg motos
rotOffs = [0,0,-20]

# Instruction types
class inst_type (Enum):
    nul = 0 # do nothing
    abs = 1 # go to absolute position
    rel = 2 # go to position relative to current position
    gnd = 3 # find the ground
    cal = 4 # calibrate legs to lift body to desired height
    gyr = 5 # calibrate rotation

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
    global amTasks, driving

    counter = 0

    for i in range(len(coords)):
        moveLegs(calcRots(coords[i], i), i)

    time.sleep(stdDelay)
    regMove.actAll(serial_connection)

    while True:
        driving = True

        execInst(moves[0][counter], 0)
        execInst(moves[1][counter], 1)
        execInst(moves[2][counter], 2)
        execInst(moves[3][counter], 3)

        time.sleep(stdDelay)
        regMove.actAll(serial_connection)
        time.sleep(stdDelay)

        time.sleep(timePerCycle / stepsPerCycle)

        counter = (counter+1)%len(moves[0])

# Generate list of instructions
def generate ():

    dists = getDists()

    for i in range(len(order)):
        for j in range(len(moves)):

            curMovePos = (outX if j <= 1 else inX) - dists[j][i] * ((outX+inX) / (len(order)))
            amMove = ((outX+inX) / (len(order))) / 2

            # move into place, tilt the robot, op does nothing, middle legs lift, lifting leg lifts more
            # lift the lifting leg, move other legs back
            # move lifting leg, others back
            # lower lifting leg, set ground level
            # calibrate other legs

            if j == order[i]:
                moves[j].append(inst(inst_type.abs, [curMovePos + amMove*1, lineDist, -walkHeight-tiltHeight])) # move + tilt
                moves[j].append(inst(inst_type.abs, [(outX if j <= 1 else inX), lineDist, -walkHeight+liftHeight])) # lift and fwd

            elif j == opposites[order[i]]:
                moves[j].append(inst(inst_type.abs, [curMovePos + amMove*1, lineDist, -walkHeight+tiltHeight])) # move + tilt
                moves[j].append(inst(inst_type.abs, [curMovePos + amMove*0, lineDist, -walkHeight+tiltHeight])) # lift and fwd
            else:
                moves[j].append(inst(inst_type.abs, [curMovePos + amMove*1, lineDist, -walkHeight])) # move + tilt
                moves[j].append(inst(inst_type.abs, [curMovePos + amMove*0, lineDist, -walkHeight])) # lift and fwd

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

# Inverse Kinematics
def calcRots (xyz, leg):
    x = xyz[0] * ((-1)**leg) # invert legs 2 and 4 (int 1 and 3)
    y = xyz[1]
    z = xyz[2]

    xyAng = math.degrees(math.atan(x/y)) if y!=0 else (90 if x >= 0 else -90) # z-rot of entire leg, from center (90) (shoulder side to side)
    xyLen = math.sqrt((x**2) + (y**2)) - lenA # length of leg vector projected from Z to ground
    trueLen = math.sqrt((xyLen**2) + (z**2)) # actual length of leg vector

    zXYAng = math.degrees(math.atan(xyLen/-z)) if z!=0 else 0 # angle of leg vector (shoulder part 1)

    if(trueLen >= lenB + lenC):
        angA = 0
        angB = 180
    else:
        angA = math.degrees(math.acos(((trueLen**2) + (lenB**2) - (lenC**2)) / (2*trueLen*lenB))) # angle of top arm from vector (shoulder part 2)
        angB = math.degrees(math.acos(((lenB**2) + (lenC**2) - (trueLen**2)) / (2*lenB*lenC))) # angle of bottom arm from top arm (elbow)

    return [(-xyAng) + rotOffs[0], (90-zXYAng-angA) + rotOffs[1], (angB-90) + rotOffs[2]] # angle at elbow has to be inverted

# Drive all motors of a leg
def moveLegs (rots,leg):

    driveLeg(leg, 0, rots[0])

    time.sleep(stdDelay)

    if(leg == 0 or leg == 3):
        driveLeg(leg, 1, rots[1])
    else:
        driveLeg(leg, 1, -rots[1])

    time.sleep(stdDelay)

    if(leg == 0 or leg == 3):
        driveLeg(leg, 2, rots[2])
    else:
        driveLeg(leg, 2, -rots[2])

# Executes instruction provided on the leg provided
def execInst (ins, leg):
    global amTasks, driving, coords

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


# drives single motor of a leg
def driveLeg (leg, motor, rot):
    toDrive = clamp(limits[motor][0], limits[motor][1], rot*(-1 if motor == 0 and leg == 1 or leg == 2 else 1))*(-1 if motor == 0 and leg == 1 or leg == 2 else 1)

    regMove.regMove(serial_connection, legId[leg][motor], toDrive, speed=1023, degrees=True)

# get amount of digits in a number
def getDigs (inn):
    return int(math.log10(abs(int(inn))) if inn != 0 else 0) +1 + (1 if inn<0 else 0) + 2

# prints all essential information
def printInterface():
    global coords

    buffer = 10

    print("-" * ((1+4+buffer)*2 + 1))
    # digsone = int(math.log10(int(coords[0][0])))+1
    # digstwo = int(math.log10(int(coords[1][0])))+1
    print("|", (" " * (4+(buffer-getDigs(coords[0][0])))),  "X: %.2f" %coords[0][0]," | ", (" " * (4+(buffer-getDigs(coords[1][0])))),  "X: %.2f" %coords[1][0]," |")
    # digsone = int(math.log10(int(coords[0][1])))+1
    # digstwo = int(math.log10(int(coords[1][1])))+1
    print("|", "0: ", (" " * (buffer-getDigs(coords[0][1]))),  "Y: %.2f" %coords[0][1]," | ", " 1: ", (" " * (buffer-getDigs(coords[1][1]))),  "Y: %.2f" %coords[1][1]," |")
    # digsone = int(math.log10(int(coords[0][2])))+1
    # digstwo = int(math.log10(int(coords[1][2])))+1
    print("|", (" " * (4+(buffer-getDigs(coords[0][2])))),  "Z: %.2f" %coords[0][2]," | ", (" " * (4+(buffer-getDigs(coords[1][2])))),  "Z: %.2f" %coords[1][2]," |")

    print("-" * ((1+4+buffer)*2 + 1))
    # digsone = int(math.log10(int(coords[2][0])))+1
    # digstwo = int(math.log10(int(coords[3][0])))+1
    print("|", (" " * (4+(buffer-getDigs(coords[2][0])))),  "X: %.2f" %coords[2][0], " | ", (" " * (4+(buffer-getDigs(coords[3][0])))),  "X: %.2f" %coords[3][0]," |")
    # digsone = int(math.log10(int(coords[2][1])))+1
    # digstwo = int(math.log10(int(coords[3][1])))+1
    print("|", "2: ", (" " * (buffer-getDigs(coords[2][1]))),  "Y: %.2f" %coords[2][1]," | ", " 3: ", (" " * (buffer-getDigs(coords[3][1]))),  "Y: %.2f" %coords[3][1]," |")
    # digsone = int(math.log10(int(coords[2][2])))+1
    # digstwo = int(math.log10(int(coords[3][2])))+1
    print("|", (" " * (4+(buffer-getDigs(coords[2][2])))),  "Z: %.2f" %coords[2][2]," | ", (" " * (4+(buffer-getDigs(coords[3][2])))),  "Z: %.2f" %coords[3][2]," |")

    print("-" * ((1+4+buffer)*2 + 1))
    print("  X: %.2f" %rotation[0], "----Y: %.2f" %rotation[1], "----Z: %.2f" %rotation[2])
    print("-" * ((1+4+buffer)*2 + 1))

# clamps num to be in between inmin and inmax
def clamp (inmin, inmax, num):
    return max(inmin, min(inmax, num))

# program start
generate()
mainLoop()
