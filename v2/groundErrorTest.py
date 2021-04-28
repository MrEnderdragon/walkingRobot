import math
import time
from pyax12.connection import Connection
import regMove
import sys
from enum import Enum
import threading
import multiprocessing
import smbus

serial_connection = Connection(port="/dev/ttyAMA0", rpi_gpio=True, baudrate=1000000)
bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

lenA = 18.5 # lenth of shoulder motor1 to shoulder motor 2, projected onto the ground plane
lenB = 83 # lenth of upper arm
lenC = 120 # length of lower arm

# amount of steps for every cycle
stepsPerCycle = 1
# amount of time taken for every cycle (seconds)
timePerCycle = 0.5
# amount to move every step of findground
gndStepSize = 4
# threshold for findGround
gndThresh = 5


# height of walk line
walkHeight = 90
# dist from walk line (back)
backLine = 120
# dist from walk line (front)
frontLine = 120
# how much to lift leg
liftHeight = 80
# amount to tilt
tiltHeight = 30

# leg max outwards reach
outX = 100
# leg max inwards reach
inX = 30


# gyro calibrate amount
amCalibrate = 10
# gyro time per rotation update [0.05]->[0.2]
updateTime = 0.05
# gyro time per move
moveTime = 0.1
# gyro gains up and down
gainsUp = 20
# gyro gains side to side (unused)
gainsSide = 1
# gyro flat threshold
gyroThresh = 1


slope = 0
robLen = 185


# robot variables V V V
amTasks = 0
driving = False

order = [0,3,1,2]

opposites = [3,2,1,0]

moves = [
[] for it in opposites
]

tmpAmMove = (outX+inX) / (len(order))

coords = [
[outX, frontLine, -10],
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

rotation = multiprocessing.Array('d', 3) # [0,0,0]
offsets = [0,0,0]

PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


class inst_type (Enum):
    nul = 0 # do nothing
    abs = 1 # go to absolute position
    rel = 2 # go to position relative to current position
    gnd = 3 # find the ground
    cal = 4 # calibrate legs to be more reasonable
    gyr = 5 # calibrate rotation

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

        print(str(coords[leg][2]) + "," + str(abs(err1)) + "," + str(abs(err2)))

        # if abs(err1) > gndThresh or abs(err2) > gndThresh:
            # break

        coords[leg][2] -= gndStepSize

        moveLegs(calcRots(coords[leg], leg), leg)
        time.sleep(0.01)
        regMove.actAll(serial_connection)
        time.sleep(0.1)

    amTasks -= 1

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

    for i in range(len(coords)):
        moveLegs(calcRots(coords[i], i), i)

    time.sleep(0.01)
    regMove.actAll(serial_connection)

    time.sleep(1)

    findGround(0)

mainLoop()
