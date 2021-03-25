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

lenA = 10.875 # lenth of shoulder motor1 to shoulder motor 2, projected onto the ground plane
lenB = 46.95 # lenth of upper arm
lenC = 68.5 # length of lower arm

stepsPerCycle = 15 # amount of steps for every cycle
timePerCycle = 0.3 # amount of time taken for every cycle (seconds)

liftHeight = 118.5 # height of walk line
lineDist = 66.95 # dist from walk line
buffer = 30 # dist limit from max reach

Se = Servo()

# bcGroundLen = math.sqrt((lenB + lenC)**2 - liftHeight**2) # length of leg vector projected from Z to the ground
# maxX = math.sqrt((bcGroundLen + lenA)**2 - lineDist**2)-buffer # maximum forwards X reach of leg

moveCycle = [
[20,20,-60], # } DOWN

[20,20,-100], # } UP
]

offsets = [0,0,0,0]

def getSteps (cycleStart, cycleEnd, stepsIn):
    moveX = (cycleEnd[0]-cycleStart[0])/stepsIn
    moveY = (cycleEnd[1]-cycleStart[1])/stepsIn
    moveZ = (cycleEnd[2]-cycleStart[2])/stepsIn

    steps = []

    for i in range(stepsIn):
        steps.append([cycleStart[0] + moveX*i,cycleStart[1] + moveY*i,cycleStart[2] + moveZ*i])

    return steps


def calcRots (xyz, leg):
    x = xyz[0]
    if(leg == 1 or leg == 2):
        x = -xyz[0] # invert legs 2 and 4 (int 1 and 3)
    y = xyz[1]
    z = xyz[2]

    #print(str(leg)+"---"+str(int(x))+" "+str(int(y))+" "+str(int(z)))

    xyAng = math.degrees(math.atan(x/y)) # z-rot of entire leg, from center (90) (shoulder side to side)
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
        stepsOne = getSteps(moveCycle[(counter+offsets[0])%len(moveCycle)], moveCycle[(counter+1+offsets[0])%len(moveCycle)], stepsPerCycle) # generate steps
        stepsTwo = getSteps(moveCycle[(counter+offsets[1])%len(moveCycle)], moveCycle[(counter+1+offsets[1])%len(moveCycle)], stepsPerCycle)
        stepsThree = getSteps(moveCycle[(counter+offsets[2])%len(moveCycle)], moveCycle[(counter+1+offsets[2])%len(moveCycle)], stepsPerCycle)
        stepsFour = getSteps(moveCycle[(counter+offsets[3])%len(moveCycle)], moveCycle[(counter+1+offsets[3])%len(moveCycle)], stepsPerCycle)

        for i in range(stepsPerCycle): # run through steps
            moveLegs(calcRots(stepsOne[i], 0), 0)
            moveLegs(calcRots(stepsTwo[i], 1), 1)
            moveLegs(calcRots(stepsThree[i], 2), 2)
            moveLegs(calcRots(stepsFour[i], 3), 3)

            time.sleep(timePerCycle / stepsPerCycle)

        print(counter)
        counter = (counter+1)%len(moveCycle)

mainLoop()
