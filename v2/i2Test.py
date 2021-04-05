import math
import time
import sys
from enum import Enum
import threading
import smbus

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
gndThresh = 7


# height of walk line
walkHeight = 90
# dist from walk line (back)
backLine = 120
# dist from walk line (front)
frontLine = 120
# how much to lift leg
liftHeight = 80
# amount to tilt
tiltHeight = 20

# leg max outwards reach
outX = 100
# leg max inwards reach
inX = 30


# gyro calibrate amount
amCalibrate = 10
# gyro time per rotation update [0.05]->[0.2]
updateTime = 0.2
# gyro time per move
moveTime = 0.05
# gyro gains up and down
gainsUp = 15
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
[outX, frontLine, -walkHeight],
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

rotation = [0,0,0]
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



def MPU_Init():
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)

    #concatenate higher and lower value
    value = ((high << 8) | low)

    #to get signed value from mpu6050
    if(value > 32768):
        value = value - 65536
    return value

def readGyros():
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)
    Gx = gyro_x/131.0
    Gy = gyro_y/131.0
    Gz = gyro_z/131.0

    return (Gx, Gy, Gz)

def updateRotation ():
    while(True):
        got = readGyros()
        rotation[0] += got[0]*updateTime - offsets[0]*updateTime
        rotation[1] += got[1]*updateTime - offsets[1]*updateTime
        rotation[2] += got[2]*updateTime - offsets[2]*updateTime
        time.sleep(updateTime)


def calibrateGyro () :
    print("calibrating...")
    for i in range(amCalibrate):
        got = readGyros()
        offsets[0] += got[0]*0.2
        offsets[1] += got[1]*0.2
        offsets[2] += got[2]*0.2
        time.sleep(0.2)
        print(i)

    offsets[0] /= (amCalibrate*0.2)
    offsets[1] /= (amCalibrate*0.2)
    offsets[2] /= (amCalibrate*0.2)
    print("Done! " + str(offsets))


def selfLevel():
    while(True):
        print("X: %.2f" %rotation[0], "----Y: %.2f" %rotation[1], "----Z: %.2f" %rotation[2])

        if abs(rotation[0])<=gyroThresh and abs(rotation[1])<=gyroThresh:
            break

        coords[0][2] -= rotation[0]*gainsUp*moveTime
        coords[2][2] -= rotation[0]*gainsUp*moveTime

        coords[1][2] += rotation[0]*gainsUp*moveTime
        coords[3][2] += rotation[0]*gainsUp*moveTime


        coords[2][2] += rotation[1]*gainsUp*moveTime
        coords[3][2] += rotation[1]*gainsUp*moveTime

        coords[0][2] -= rotation[1]*gainsUp*moveTime
        coords[1][2] -= rotation[1]*gainsUp*moveTime

        for i in range(4):
            moveLegs(calcRots(coords[i],i),i)
            time.sleep(0.01)

        regMove.actAll(serial_connection)

        time.sleep(moveTime)


def mainLoop():

    MPU_Init()

    time.sleep(1)

    calibrateGyro()
    updateRotation()

mainLoop()
