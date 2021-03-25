import math
import time
import offsets as ofs
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

lenA = 10.875 # lenth of shoulder motor1 to shoulder motor 2, projected onto the ground plane
lenB = 46.95 # lenth of upper arm
lenC = 68.5 # length of lower arm

stepsPerCycle = 10 # amount of steps for every cycle
timePerCycle = 0.1 # amount of time taken for every cycle (seconds)

walkHeight = 68.5 # height of walk line
lineDist = 46.95 # dist from walk line
buffer = 60 # dist limit from max reach

liftHeight = 20 # amount to drop to tilt

# bcGroundLen = math.sqrt((lenB + lenC)**2 - walkHeight**2) # length of leg vector projected from Z to the ground
# maxX = math.sqrt((bcGroundLen + lenA)**2 - lineDist**2)-buffer # maximum forwards X reach of leg
maxX = 30
minX = 10

order = [0,1]

opposites = [3,2,1,0]

moves = [
[] for it in opposites
]

steps = [
[] for it in opposites
]

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

    for i in range(len(order)):
        for j in range(len(moves)):
            if j == order[i]: #lift
                moves[j].append([minX if j <= 1 else -maxX, lineDist, -walkHeight]) # move
                moves[j].append([minX if j <= 1 else -maxX, lineDist, -walkHeight+liftHeight]) # lift
            elif j == opposites[order[i]]: #drop
                moves[j].append([minX if j <= 1 else -maxX, lineDist, -walkHeight]) # move
                moves[j].append([minX if j <= 1 else -maxX, lineDist, -walkHeight+liftHeight]) # lift
            else: #do
                moves[j].append([maxX if j <= 1 else -minX, lineDist, -walkHeight+liftHeight]) # unlift
                moves[j].append([maxX if j <= 1 else -minX, lineDist, -walkHeight]) # move

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

    return [max(min(90+xyAng,180),0), max(min(180-zXYAng-angA,180),0), max(min(angB,180),0)] # angle at elbow has to be inverted



def moveLegs (rots,leg): # one leg per group of  (4 ports)

    kit.servo[leg*4+0].angle = rots[0] + ofs.offsets[leg][0]# invert the legs on one side

    if(leg == 0 or leg == 3):
        kit.servo[leg*4+1].angle = rots[1] + ofs.offsets[leg][1]
    else:
        kit.servo[leg*4+1].angle = 180-rots[1] + ofs.offsets[leg][1]

    if(leg == 0 or leg == 3):
        kit.servo[leg*4+2].angle = rots[2] + ofs.offsets[leg][2]
    else:
        kit.servo[leg*4+2].angle = 180-rots[2] + ofs.offsets[leg][2]


def mainLoop():
    counter = 0
    moveLegs(steps[0][0], 0)
    moveLegs(steps[1][0], 1)
    moveLegs(steps[2][0], 2)
    moveLegs(steps[3][0], 3)
    time.sleep(1)
    while True:
        moveLegs(steps[0][counter], 0)
        moveLegs(steps[1][counter], 1)
        moveLegs(steps[2][counter], 2)
        moveLegs(steps[3][counter], 3)

        time.sleep(timePerCycle / stepsPerCycle)

        counter = (counter+1)%len(steps[0])

generate()
genSteps()
mainLoop()
