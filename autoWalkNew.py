import math
import time
import Adafruit_PCA9685

class Servo:
    def __init__(self):
        self.angleMin=18
        self.angleMax=162
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(50)               # Set the cycle frequency of PWM
    #Convert the input angle to the value of pca9685
    def map(self,value,fromLow,fromHigh,toLow,toHigh):
        return (toHigh-toLow)*(value-fromLow) / (fromHigh-fromLow) + toLow
    def setAng(self,channel, angle):
        if angle < self.angleMin:
            angle = self.angleMin
        elif angle >self.angleMax:
            angle=self.angleMax
        date=self.map(angle,0,180,102,512)
        #print(date,date/4096*0.02)
        self.pwm.set_pwm(channel, 0, int(date))

lenA = 10.875 # lenth of shoulder motor1 to shoulder motor 2, projected onto th$
lenB = 46.95 # lenth of upper arm
lenC = 68.5 # length of lower arm

stepsPerCycle = 15 # amount of steps for every cycle
timePerCycle = 0.1 # amount of time taken for every cycle (seconds)

walkHeight = 88.5 # height of walk line
lineDist = 36.95 # dist from walk line
buffer = 60 # dist limit from max reach

tiltHeight = 20 # amount to drop to tilt

bcGroundLen = math.sqrt((lenB + lenC)**2 - walkHeight**2) # length of leg vecto$
#maxX = math.sqrt((bcGroundLen + lenA)**2 - lineDist**2)-buffer # maximum forwa$
maxX = 25

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

def moveLegs (rots,leg): # one leg per group of  (4 ports)
    Se.setAng(leg*4+0,rots[0]) # invert the legs on one side

    if(leg == 0 or leg == 3):
        # kit.servo[leg*4+1].angle = rots[1]
        Se.setAng(leg*4+1,rots[1])
    else:
        # kit.servo[leg*4+1].angle = 180-rots[1]
        Se.setAng(leg*4+1,180-rots[1])

    if(leg == 0 or leg == 3):
        # kit.servo[leg*4+2].angle = rots[2]
        Se.setAng(leg*4+2,rots[2])
    else:
        # kit.servo[leg*4+2].angle = 180-rots[2]
        Se.setAng(leg*4+2,180-rots[2])


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
mainLoop()
