import math
import time
# from adafruit_servokit import ServoKit
# kit = ServoKit(channels=16)

lenA = 10.875 # lenth of shoulder motor1 to shoulder motor 2, projected onto the ground plane
lenB = 46.95 # lenth of upper arm
lenC = 68.5 # length of lower arm

stepsPerCycle = 30 # amount of steps for every cycle
timePerCycle = 1 # amount of time taken for every cycle (seconds)

walkHeight = 68.5 # height of walk line
lineDist = 46.95 # dist from walk line
buffer = 60 # dist limit from max reach

tiltHeight = 30 # amount to drop to tilt

bcGroundLen = math.sqrt((lenB + lenC)**2 - walkHeight**2) # length of leg vector projected from Z to the ground
# maxX = math.sqrt((bcGroundLen + lenA)**2 - lineDist**2)-buffer # maximum forwards X reach of leg
maxX = 30

order = [0,3,1,2]

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
    dists = getDists()

    print(dists)

    for i in range(len(order)):
        for j in range(len(moves)):

            curMovePos = maxX - dists[j][i] * (maxX*2 / len(order))
            amMove = (maxX*2 / len(order)) / 3

            if j == order[i]: #lift
                moves[j].append([curMovePos + amMove*2, lineDist, -walkHeight]) # position move
                moves[j].append([curMovePos + amMove*2, lineDist, -walkHeight-tiltHeight]) # drop
                moves[j].append([curMovePos + amMove*2, lineDist, -walkHeight+30]) # lift
                moves[j].append([maxX, lineDist, -walkHeight+30]) # lifted move
                moves[j].append([maxX, lineDist, -walkHeight]) # undrop
            elif j == opposites[order[i]]: #drop
                moves[j].append([curMovePos + amMove*2, lineDist, -walkHeight]) # position move
                moves[j].append([curMovePos + amMove*2, lineDist, -walkHeight+tiltHeight]) # drop
                moves[j].append([curMovePos + amMove, lineDist, -walkHeight+tiltHeight]) # move
                moves[j].append([curMovePos, lineDist, -walkHeight+tiltHeight])# move
                moves[j].append([curMovePos, lineDist, -walkHeight]) # undrop
            else: #do
                moves[j].append([curMovePos + amMove*2, lineDist, -walkHeight]) # position move
                moves[j].append([curMovePos + amMove*2, lineDist, -walkHeight]) # drop
                moves[j].append([curMovePos + amMove, lineDist, -walkHeight]) # move
                moves[j].append([curMovePos, lineDist, -walkHeight])# move
                moves[j].append([curMovePos, lineDist, -walkHeight]) # undrop

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



def moveLegs (rots,leg): # one leg per group of  (4 ports) TODO: REDO!!! ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    # print (rots)

    kit.servo[leg*4+0].angle = rots[0] # invert the legs on one side

    if(leg == 0 or leg == 3):
        kit.servo[leg*4+1].angle = rots[1]
    else:
        kit.servo[leg*4+1].angle = 180-rots[1]

    if(leg == 0 or leg == 3):
        kit.servo[leg*4+2].angle = rots[2]
    else:
        kit.servo[leg*4+2].angle = 180-rots[2]


def mainLoop():
    counter = 0
    while True:
        moveLegs(steps[0][counter], 0)
        moveLegs(steps[1][counter], 1)
        moveLegs(steps[2][counter], 2)
        moveLegs(steps[3][counter], 3)

        time.sleep(timePerCycle / stepsPerCycle)

        counter = (counter+1)%(len(order)*stepsPerCycle*5)

generate()
genSteps()

for itttttt in range(len(moves[0])):
    print(moves[0][itttttt])

print("2: ---------------")

for itttttt in range(len(moves[1])):
    print(moves[1][itttttt])

print("3: ---------------")

for itttttt in range(len(moves[2])):
    print(moves[2][itttttt])


print("4: ---------------")

for itttttt in range(len(moves[3])):
    print(moves[3][itttttt])

# print(len(steps[0]))

mainLoop()
