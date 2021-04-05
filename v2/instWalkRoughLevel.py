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


def getDigs (inn):
    return int(math.log10(abs(int(inn))))+1 + (1 if inn<0 else 0) + 2

def printInterface():
    global coords

    buffer = 10

    print("-" * ((1+10)*2 + 1))
    # digsone = int(math.log10(int(coords[0][0])))+1
    # digstwo = int(math.log10(int(coords[1][0])))+1
    print("|", (" " * (4+(buffer-getDigs(coords[0][0])))),  "X: %.2f" %coords[0][0]," | ", (" " * (4+(buffer-getDigs(coords[1][0])))),  "X: %.2f" %coords[1][0]," |")
    # digsone = int(math.log10(int(coords[0][1])))+1
    # digstwo = int(math.log10(int(coords[1][1])))+1
    print("|", " 0: ", (" " * (buffer-getDigs(coords[0][1]))),  "Y: %.2f" %coords[0][1]," | ", " 1: ", (" " * (buffer-getDigs(coords[1][1]))),  "Y: %.2f" %coords[1][1]," |")
    # digsone = int(math.log10(int(coords[0][2])))+1
    # digstwo = int(math.log10(int(coords[1][2])))+1
    print("|", (" " * (4+(buffer-getDigs(coords[0][2])))),  "Z: %.2f" %coords[0][2]," | ", (" " * (4+(buffer-getDigs(coords[1][2])))),  "Z: %.2f" %coords[1][2]," |")

    print("-" * ((1+10)*2 + 1))
    # digsone = int(math.log10(int(coords[2][0])))+1
    # digstwo = int(math.log10(int(coords[3][0])))+1
    print("|", (" " * (4+(buffer-getDigs(coords[2][0])))),  "X: %.2f" %coords[2][0], " | ", (" " * (4+(buffer-getDigs(coords[3][0])))),  "X: %.2f" %coords[3][0]," |")
    # digsone = int(math.log10(int(coords[2][1])))+1
    # digstwo = int(math.log10(int(coords[3][1])))+1
    print("|", " 2: ", (" " * (buffer-getDigs(coords[2][1]))),  "Y: %.2f" %coords[2][1]," | ", " 3: ", (" " * (buffer-getDigs(coords[3][1]))),  "Y: %.2f" %coords[3][1]," |")
    # digsone = int(math.log10(int(coords[2][2])))+1
    # digstwo = int(math.log10(int(coords[3][2])))+1
    print("|", (" " * (4+(buffer-getDigs(coords[2][2])))),  "Z: %.2f" %coords[2][2]," | ", (" " * (4+(buffer-getDigs(coords[3][2])))),  "Z: %.2f" %coords[3][2]," |")

    print("-" * ((1+10)*2 + 1))
    print("  X: %.2f" %rotation[0], "----Y: %.2f" %rotation[1], "----Z: %.2f" %rotation[2])
    print("-" * ((1+10)*2 + 1))


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
            amMove = ((outX+inX) / (len(order))) / 4

            # move into place, tilt the robot, op does nothing, middle legs lift, lifting leg lifts more
            # lift the lifting leg, move other legs back
            # move lifting leg, others back
            # lower lifting leg, set ground level
            # calibrate other legs

            if j == order[i]: #lift
                moves[j].append(inst(inst_type.rel, [-amMove, 0, -tiltHeight*2])) # move + tilt
                moves[j].append(inst(inst_type.rel, [0, 0, liftHeight])) # lift
                moves[j].append(inst(inst_type.abs, [(outX if j <= 1 else inX), None, None])) # fwd
                moves[j].append(inst(inst_type.gnd, [])) # ground
                moves[j].append(inst(inst_type.cal, [])) # calibration
                moves[j].append(inst(inst_type.gyr, [])) # calibration

            elif j == opposites[order[i]]: #drop
                moves[j].append(inst(inst_type.rel, [-amMove, 0, tiltHeight])) # move + tilt
                moves[j].append(inst(inst_type.rel, [-amMove, 0, 0])) # lift
                moves[j].append(inst(inst_type.rel, [-amMove, 0, 0])) # fwd
                moves[j].append(inst(inst_type.rel, [-amMove, 0, -tiltHeight])) # ground
                moves[j].append(inst(inst_type.cal, [])) # calibration
                moves[j].append(inst(inst_type.gyr, [])) # calibration
            else: #do
                moves[j].append(inst(inst_type.rel, [-amMove, 0, -tiltHeight])) # move + tilt
                moves[j].append(inst(inst_type.rel, [-amMove, 0, 0])) # lift
                moves[j].append(inst(inst_type.rel, [-amMove, 0, 0])) # fwd
                moves[j].append(inst(inst_type.rel, [-amMove, 0, tiltHeight])) # ground
                moves[j].append(inst(inst_type.cal, [])) # calibration
                moves[j].append(inst(inst_type.gyr, [])) # calibration



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

        print(str(err1) + " 1 <==> 2 " + str(err2))

        if abs(err1) > gndThresh or abs(err2) > gndThresh:
            break

        coords[leg][2] -= gndStepSize

        moveLegs(calcRots(coords[leg], leg), leg)
        time.sleep(0.01)
        regMove.actAll(serial_connection)
        time.sleep(0.1)

    amTasks -= 1


def calibrateLegs ():
    global coords
    avg = 0
    for i in coords:
        avg -= i[2]

    diff = (walkHeight*4 - avg)/4.0

    print("diff " + str(diff))

    for i in range(4):
        coords[i][2] -= diff
        moveLegs(calcRots(coords[i], i), i)

    regMove.actAll(serial_connection)


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

    elif (ins.type == inst_type.gnd):
        amTasks += 1
        t1 = threading.Thread(target = findGround, args = (leg,))
        t1.start()

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

def updateRotation (rotation, offsets):
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

        print("1: %.2f" %coords[0][2], "----2: %.2f" %coords[1][2])
        print("3: %.2f" %coords[2][2], "----4: %.2f" %coords[3][2])


        if abs(rotation[0])<=gyroThresh and abs(rotation[1])<=gyroThresh:
            break

        #     - <  > +       ^ +     v -


        #   0    1

        #   2    3


        coords[0][2] += rotation[0]*gainsUp*moveTime
        coords[2][2] += rotation[0]*gainsUp*moveTime

        coords[1][2] -= rotation[0]*gainsUp*moveTime
        coords[3][2] -= rotation[0]*gainsUp*moveTime


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
    global amTasks, driving

    counter = 0

    for i in range(len(coords)):
        moveLegs(calcRots(coords[i], i), i)

    time.sleep(0.01)
    regMove.actAll(serial_connection)

    time.sleep(1)

    MPU_Init()

    time.sleep(1)

    calibrateGyro()

    updT = multiprocessing.Process(target = updateRotation, args = (rotation,offsets,))
    updT.start()

    while True:
        driving = True

        levelCal = False
        rotCal = False

        for i in range(4):
            if(moves[i][counter].type == inst_type.cal):
                levelCal = True
                break
            if(moves[i][counter].type == inst_type.gyr):
                rotCal = True
                break

        if levelCal:
            # print("calibration not yet implemented")
            calibrateLegs()
        elif rotCal:
            selfLevel()
        else:
            execInst(moves[0][counter], 0)
            execInst(moves[1][counter], 1)
            execInst(moves[2][counter], 2)
            execInst(moves[3][counter], 3)

            time.sleep(0.01)
            regMove.actAll(serial_connection)
            time.sleep(0.01)

        driving = False
        print("drive end")

        if not levelCal and not rotCal:
            time.sleep(timePerCycle / stepsPerCycle)

        # print(str(amTasks) + " tasks")

        while amTasks > 0:
            time.sleep(0.01)

        print("waiting end")

        counter = (counter+1)%len(moves[0])

generate()
mainLoop()
