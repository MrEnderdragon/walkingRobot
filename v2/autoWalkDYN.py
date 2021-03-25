import math
import time
from pyax12.connection import Connection
import regMove
import sys

serial_connection = Connection(port="/dev/ttyAMA0", rpi_gpio=True, baudrate=1000000)

lenA = 18.5 # lenth of shoulder motor1 to shoulder motor 2, projected onto the ground plane
lenB = 83 # lenth of upper arm
lenC = 120 # length of lower arm

stepsPerCycle = 1 # amount of steps for every cycle
timePerCycle = 0.8 # amount of time taken for every cycle (seconds)

walkHeight = 120 # height of walk line
lineDist = 80 # dist from walk line
liftHeight = 60
# buffer = 60 # dist limit from max reach

tiltHeight = 20 # amount to drop to tilt

# bcGroundLen = math.sqrt((lenB + lenC)**2 - walkHeight**2) # length of leg vector projected from Z to the ground
# maxX = math.sqrt((bcGroundLen + lenA)**2 - lineDist**2)-buffer # maximum forwards X reach of leg
outX = 30
inX = 30

order = [0,3,1,2]

opposites = [3,2,1,0]

moves = [
[] for it in opposites
]

steps = [
[] for it in opposites
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
            amMove = ((outX+inX) / (len(order))) / 5

            if j == order[i]: #lift
                moves[j].append([curMovePos + amMove*4, lineDist, -walkHeight]) # position move
                moves[j].append([curMovePos + amMove*4, lineDist, -walkHeight-tiltHeight]) # drop
                moves[j].append([curMovePos + amMove*4, lineDist, -walkHeight+liftHeight]) # lift
                moves[j].append([(outX if j <= 1 else inX), lineDist, -walkHeight+liftHeight]) # lifted move
                moves[j].append([(outX if j <= 1 else inX), lineDist, -walkHeight]) # undrop
            elif j == opposites[order[i]]: #drop
                moves[j].append([curMovePos + amMove*4, lineDist, -walkHeight]) # position move
                moves[j].append([curMovePos + amMove*3, lineDist, -walkHeight+tiltHeight]) # drop
                moves[j].append([curMovePos + amMove*2, lineDist, -walkHeight+tiltHeight]) # move
                moves[j].append([curMovePos + amMove*1, lineDist, -walkHeight+tiltHeight])# move
                moves[j].append([curMovePos + amMove*0, lineDist, -walkHeight]) # undrop
            else: #do
                moves[j].append([curMovePos + amMove*4, lineDist, -walkHeight]) # position move
                moves[j].append([curMovePos + amMove*3, lineDist, -walkHeight]) # drop
                moves[j].append([curMovePos + amMove*2, lineDist, -walkHeight]) # move
                moves[j].append([curMovePos + amMove*1, lineDist, -walkHeight])# move
                moves[j].append([curMovePos + amMove*0, lineDist, -walkHeight]) # undrop

def getSteps (cycleStart, cycleEnd, stepsIn, legIn):
    moveX = (cycleEnd[0]-cycleStart[0])/stepsIn
    moveY = (cycleEnd[1]-cycleStart[1])/stepsIn
    moveZ = (cycleEnd[2]-cycleStart[2])/stepsIn

    stepsTmp = []

    for i in range(stepsIn):
        stepsTmp.append(calcRots([cycleStart[0] + moveX*i,cycleStart[1] + moveY*i,cycleStart[2] + moveZ*i], legIn))

    return stepsTmp

def genSteps () :
    for leg in range(len(moves)):
        for i in range(len(moves[leg])):
            steps[leg].extend(getSteps(moves[leg][i], moves[leg][(i+1)%len(moves[leg])], stepsPerCycle, leg))


def calcRots (xyz, leg):
    x = xyz[0] * ((-1)**leg) # invert legs 2 and 4 (int 1 and 3)
    y = xyz[1]
    z = xyz[2]

    #print(str(leg)+"---"+str(int(x))+" "+str(int(y))+" "+str(int(z)))

    xyAng = math.degrees(math.atan(x/y)) if y!=0 else (90 if x >= 0 else -90) # z-rot of entire leg, from center (90) (shoulder side to side)
    xyLen = math.sqrt((x**2) + (y**2)) - lenA # length of leg vector projected from Z to ground
    trueLen = math.sqrt((xyLen**2) + (z**2)) # actual length of leg vector

    zXYAng = math.degrees(math.atan(xyLen/abs(z))) # angle of leg vector (shoulder part 1)
    angA = math.degrees(math.acos(((trueLen**2) + (lenB**2) - (lenC**2)) / (2*trueLen*lenB))) # angle of top arm from vector (shoulder part 2)
    angB = math.degrees(math.acos(((lenB**2) + (lenC**2) - (trueLen**2)) / (2*lenB*lenC))) # angle of bottom arm from top arm (elbow)

    return [(-xyAng) + rotOffs[0], (90-zXYAng-angA) + rotOffs[1], (angB-90) + rotOffs[2]] # angle at elbow has to be inverted



def driveLeg (leg, motor, rot):
    if(rot*(-1 if motor == 0 and leg == 1 or leg == 2 else 1) < limits[motor][0] or rot*(-1 if motor == 0 and leg == 1 or leg == 2 else 1) > limits[motor][1]):
        print("Leg " + str(leg) + "-" + str(motor) + " angle out of bounds, rot= " + str(rot))
        raise ValueError
    regMove.regMove(serial_connection, legId[leg][motor], rot, speed=512, degrees=True)



def moveLegs (rots,leg): # one leg per group of  (4 ports)

    # for i in range(0, 3):
    #     if(rots[i] < limits[i][0] or rots[i] > limits[i][1]):
    #         print("Leg " + str(leg) + "-" + str(i) + " angle out of bounds, rot= " + str(rots[i]))
    #         raise ValueError


    driveLeg(leg, 0, rots[0])

    # regMove.regMove(serial_connection, legId[leg][0], rots[0], speed=512, degrees=True)

    time.sleep(0.01)

    if(leg == 0 or leg == 3):
        # kit.servo[leg*4+1].angle = rots[1]
        # regMove.regMove(serial_connection, legId[leg][1], rots[1], speed=512, degrees=True)
        driveLeg(leg, 1, rots[1])
    else:
        # regMove.regMove(serial_connection, legId[leg][1], -rots[1], speed=512, degrees=True)
        driveLeg(leg, 1, -rots[1])
        # kit.servo[leg*4+1].angle = 180-rots[1]
    time.sleep(0.01)

    if(leg == 0 or leg == 3):
        # regMove.regMove(serial_connection, legId[leg][2], rots[2], speed=512, degrees=True)
        driveLeg(leg, 2, rots[2])
        # kit.servo[leg*4+2].angle = rots[2]
    else:
        # regMove.regMove(serial_connection, legId[leg][2], -rots[2], speed=512, degrees=True)
        driveLeg(leg, 2, -rots[2])
        # kit.servo[leg*4+2].angle = 180-rots[2]


def mainLoop():
    counter = 0
    moveLegs(steps[0][0], 0)
    moveLegs(steps[1][0], 1)
    moveLegs(steps[2][0], 2)
    moveLegs(steps[3][0], 3)

    time.sleep(0.01)
    regMove.actAll(serial_connection)

    time.sleep(1)
    while True:
        moveLegs(steps[0][counter], 0)
        moveLegs(steps[1][counter], 1)
        moveLegs(steps[2][counter], 2)
        moveLegs(steps[3][counter], 3)

        time.sleep(0.01)
        regMove.actAll(serial_connection)

        time.sleep(timePerCycle / stepsPerCycle)

        counter = (counter+1)%len(steps[0])

generate()
genSteps()
mainLoop()
