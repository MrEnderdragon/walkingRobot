import math
import time
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

lenA = 10.875 # lenth of shoulder motor1 to shoulder motor 2, projected onto the ground plane
lenB = 46.95 # lenth of upper arm
lenC = 68.5 # length of lower arm

legs = [
[70,70,-130], # } one

[70,70,-130], # } two

[70,70,-130], # } three

[70,70,-130], # } four
]


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
    while True:
        moveLegs(calcRots(legs[0], 0), 0)
        moveLegs(calcRots(legs[1], 1), 1)
        moveLegs(calcRots(legs[2], 2), 2)
        moveLegs(calcRots(legs[3], 3), 3)

mainLoop()
