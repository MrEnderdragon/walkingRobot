import math
import time
from pyax12.connection import Connection
import pyax12.utils as utils
import regMove
import sys

serial_connection = Connection(port="/dev/ttyAMA0", rpi_gpio=True, baudrate=1000000)

thresh = 1
step = 1
reset = [0,100,-70]


lenA = 18.5 # lenth of shoulder motor1 to shoulder motor 2, projected onto the ground plane
lenB = 83 # lenth of upper arm
lenC = 120 # length of lower arm


legId = [
[1,2,3],
[10,11,12],
[7,8,9],
[4,5,6]
]

coords = reset.copy()

limits = [
[-40,20],
[-40,50],
[-75,105]
]

rotOffs = [0,0,-20]


legg = int(sys.argv[1])

def calcRots (xyz, leg):
    x = xyz[0] * ((-1)**leg) # invert legs 2 and 4 (int 1 and 3)
    y = xyz[1]
    z = xyz[2]

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


while True:
    while True:
        pres1 = serial_connection.get_present_position(legId[legg][1])
        goal1 = serial_connection.get_goal_position(legId[legg][1])

        pres2 = serial_connection.get_present_position(legId[legg][2])
        goal2 = serial_connection.get_goal_position(legId[legg][2])

        err1 = pres1-goal1
        err2 = pres2-goal2

        if abs(err1) > thresh or abs(err2) > thresh:
            break

        coords[2] -= step

        moveLegs(calcRots(coords, legg), legg)
        time.sleep(0.01)
        regMove.actAll(serial_connection)
        time.sleep(0.05)

    coords = reset.copy()
    moveLegs(calcRots(coords, legg), legg)
    time.sleep(0.01)
    regMove.actAll(serial_connection)
    time.sleep(0.2)
