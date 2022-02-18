import copy
import math
import time
import numpy as np
from pyax12.connection import Connection
# import depthai as dai
import regMove
from enum import Enum
import threading
import multiprocessing
from multiprocessing import Lock

# AX-12A motor connection
serial_connection = Connection(port="/dev/ttyAMA0", rpi_gpio=True, baudrate=1000000)

lenA = 18.5  # lenth of shoulder motor1 to shoulder motor 2
lenB = 83  # lenth of upper arm
lenC = 120  # length of lower arm

# amount of moves for every step
stepsPerCycle = 1
# amount of time taken for every move (seconds)
timePerCycle = 0.25
# timePerCycle = 1
# amount to move every step of findground subsystem
gndStepSize = 4
# threshold for findGround subsystem
gndThresh = 5

# height of walk line
walkHeight = 90
# ensure that all legs are aways at this height or higher
minHeight = 70
# distance of steps from body (back legs)
backLine = 120
# distance of steps from body (front legs)
frontLine = 120
# how much to lift leg to step
liftHeight = 90
# amount to tilt before stepping
tiltHeight = 30

# leg max outwards reach
outX = 130
# leg max inwards reach
inX = 20

# gyro calibrate amount
amCalibrate = 3
# time per gyro read
updateTime = 0.05
# time per leg adjustment for levelling subsystem
moveTime = 0.1
# levelling gains up and down for levelling subsystem
gainsUp = -5
# rotation threshold for levelling subsystem
gyroThresh = 3

order = [0, 3, 1, 2]  # order to take steps

# program variables V V V
amTasks = 0  # how many current tasks running
driving = False  # is robot driving a leg?

opposites = [3, 2, 1, 0]  # opposites of every leg

# empty array to store instructions
moves = [
    [] for it in opposites
]

# initial starting coordinates
coords = [
    [-inX, frontLine, -walkHeight],
    [outX - (outX + inX) * 1 / 3, frontLine, -walkHeight],
    [+inX, backLine, -walkHeight],
    [- outX + (outX + inX) * 1 / 3, backLine, -walkHeight]
]

# motor ids
legId = [
    [1, 2, 3],
    [10, 11, 12],
    [7, 8, 9],
    [4, 5, 6]
]

# movement limits of leg motors
limits = [
    [-40, 20],
    [-40, 50],
    [-75, 105]
]

# rotation offsets for leg motos
rotOffs = [0, 0, -20]

# Arrays for transferring data between processes
rotation = multiprocessing.Array('d', 3)  # [0,0,0]
offsets = multiprocessing.Array('d', 3)  # [0,0,0]


# Instruction types
class inst_type(Enum):
    nul = 0  # do nothing
    abs = 1  # go to absolute position
    rel = 2  # go to position relative to current position
    gnd = 3  # find the ground
    cal = 4  # calibrate legs to lift body to desired height
    gyr = 5  # calibrate rotation


# Instruction class
class inst:
    type = inst_type.nul
    args = []

    # args for each type:
    # 0: none
    # 1: x, y, z
    # 2: +x, +y, +z
    # 3: none
    # 4: none
    # 5: none

    def __init__(self, typeIn, argsIn):
        self.type = typeIn
        self.args = argsIn


# Main loop
def mainLoop():
    global amTasks, driving

    counter = 0

    for i in range(len(coords)):
        moveLegs(calcRots(coords[i], i), i)

    time.sleep(0.01)
    regMove.actAll(serial_connection)

    time.sleep(1)

    ll = Lock()

    updT = multiprocessing.Process(target=updateRotation, args=(rotation, offsets, ll,))
    updT.start()

    time.sleep(0.5)

    ll.acquire()

    while True:
        driving = True

        levelCal = False
        rotCal = False

        for i in range(4):
            if moves[i][counter].type == inst_type.cal:
                levelCal = True
                break
            if moves[i][counter].type == inst_type.gyr:
                rotCal = True
                break

        if levelCal:
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

        if not levelCal and not rotCal:
            time.sleep(timePerCycle / stepsPerCycle)

        # wait for multiprocessing tasks to be finished
        while amTasks > 0:
            time.sleep(0.01)

        counter = (counter + 1) % len(moves[0])


# Generate list of instructions
def generate():
    # height of walk line
    altwalkHeight = 90
    # distance of steps from body
    altlineDist = 120
    # how much to lift leg to step
    altliftHeight = 40
    # amount to tilt before stepping
    alttiltHeight = 10

    # leg max outwards reach
    altoutX = 120
    altinX = 50

    # amWalkStraight = 3
    amWalkStraight = 0
    amWalkRough = 30

    tmpCoords = copy.deepcopy(coords)

    for _ in range(amWalkStraight):
        for i in range(len(order)):
            for j in range(len(moves)):

                altamMove = ((altoutX + altinX) / (len(order)))

                # move into place, tilt the robot, op does nothing, middle legs lift, lifting leg lifts more
                # lift the lifting leg, move other legs back
                # move lifting leg, others back
                # lower lifting leg, set ground level
                # calibrate other legs

                tmpCoords[j][0] -= altamMove

                if j == order[i]:  # lift
                    moves[j].append(inst(inst_type.abs, [tmpCoords[j][0], altlineDist, -altwalkHeight-alttiltHeight]))  # fwd
                    moves[j].append(inst(inst_type.abs, [(altoutX if j <= 1 else altinX), altlineDist, -altwalkHeight+altliftHeight]))  # fwd
                    tmpCoords[j][0] = (altoutX if j <= 1 else altinX)

                elif j == opposites[order[i]]:  # drop
                    moves[j].append(inst(inst_type.abs, [tmpCoords[j][0], altlineDist, -altwalkHeight + alttiltHeight]))  # fwd
                    moves[j].append(inst(inst_type.abs, [None, altlineDist, -altwalkHeight + alttiltHeight]))  # fwd

                else:  # do
                    moves[j].append(inst(inst_type.abs, [tmpCoords[j][0], altlineDist, -altwalkHeight]))  # fwd
                    moves[j].append(inst(inst_type.abs, [None, altlineDist, -altwalkHeight]))  # fwd

    for _ in range(amWalkRough):
        for i in range(len(order)):
            for j in range(len(moves)):

                amMove = ((outX + inX) / (len(order))) / 3

                # move into place, tilt the robot, op does nothing, middle legs lift, lifting leg lifts more
                # lift the lifting leg, move other legs back
                # move lifting leg, others back
                # lower lifting leg, set ground level
                # calibrate other legs

                if j == order[i]:  # lift
                    moves[j].append(inst(inst_type.rel, [-amMove, 0, -tiltHeight * 2]))  # move + tilt
                    moves[j].append(inst(inst_type.rel, [0, 0, liftHeight]))  # lift
                    moves[j].append(inst(inst_type.abs, [(outX if j <= 1 else inX), None, None]))  # fwd
                    moves[j].append(inst(inst_type.gnd, []))  # ground
                    moves[j].append(inst(inst_type.cal, []))  # calibration
                    moves[j].append(inst(inst_type.gyr, []))  # calibration

                elif j == opposites[order[i]]:  # drop
                    moves[j].append(inst(inst_type.rel, [-amMove, 0, tiltHeight]))  # move + tilt
                    moves[j].append(inst(inst_type.rel, [-amMove, 0, 0]))  # lift
                    moves[j].append(inst(inst_type.rel, [-amMove, 0, 0]))  # fwd
                    moves[j].append(inst(inst_type.nul, []))  # ground
                    moves[j].append(inst(inst_type.cal, []))  # calibration
                    moves[j].append(inst(inst_type.gyr, []))  # calibration
                else:  # do
                    moves[j].append(inst(inst_type.rel, [-amMove, 0, -tiltHeight]))  # move + tilt
                    moves[j].append(inst(inst_type.rel, [-amMove, 0, 0]))  # lift
                    moves[j].append(inst(inst_type.rel, [-amMove, 0, 0]))  # fwd
                    moves[j].append(inst(inst_type.nul, []))  # ground
                    moves[j].append(inst(inst_type.cal, []))  # calibration
                    moves[j].append(inst(inst_type.gyr, []))  # calibration


# Inverse Kinematics
def calcRots(xyz, leg):
    x = xyz[0] * ((-1) ** leg)  # invert legs 2 and 4 (int 1 and 3)
    y = xyz[1]
    z = xyz[2]

    xyAng = math.degrees(math.atan(x / y)) if y != 0 else (
        90 if x >= 0 else -90)  # z-rot of entire leg, from center (90) (shoulder side to side)
    xyLen = math.sqrt((x ** 2) + (y ** 2)) - lenA  # length of leg vector projected from Z to ground
    trueLen = math.sqrt((xyLen ** 2) + (z ** 2))  # actual length of leg vector

    zXYAng = math.degrees(math.atan(xyLen / -z)) if z != 0 else 0  # angle of leg vector (shoulder part 1)

    if trueLen >= lenB + lenC:
        angA = 0
        angB = 180
    else:
        angA = math.degrees(math.acos(((trueLen ** 2) + (lenB ** 2) - (lenC ** 2)) / (
                    2 * trueLen * lenB)))  # angle of top arm from vector (shoulder part 2)
        angB = math.degrees(math.acos(((lenB ** 2) + (lenC ** 2) - (trueLen ** 2)) / (
                    2 * lenB * lenC)))  # angle of bottom arm from top arm (elbow)

    return [(-xyAng) + rotOffs[0], (90 - zXYAng - angA) + rotOffs[1],
            (angB - 90) + rotOffs[2]]  # angle at elbow has to be inverted


# Drive all motors of a leg
def moveLegs(rots, leg):
    driveLeg(leg, 0, rots[0])

    time.sleep(0.01)

    if leg == 0 or leg == 3:
        driveLeg(leg, 1, rots[1])
    else:
        driveLeg(leg, 1, -rots[1])
    time.sleep(0.01)

    if leg == 0 or leg == 3:
        driveLeg(leg, 2, rots[2])
    else:
        driveLeg(leg, 2, -rots[2])


# Executes instruction provided on the leg provided
def execInst(ins, leg):
    global amTasks, driving, coords

    for tryy in range(3):
        try:
            if ins.type == inst_type.abs:
                for i in range(0, 3):
                    if ins.args[i] is not None:
                        coords[leg][i] = ins.args[i] * (0.80 if i == 0 and leg % 2 == 1 else 1)
                moveLegs(calcRots(coords[leg], leg), leg)

            elif ins.type == inst_type.rel:
                for i in range(0, 3):
                    if ins.args[i] is not None:
                        coords[leg][i] += ins.args[i] * (0.80 if i == 0 and leg % 2 == 1 else 1)
                moveLegs(calcRots(coords[leg], leg), leg)

            elif ins.type == inst_type.gnd:
                amTasks += 1
                t1 = threading.Thread(target=findGround, args=(leg,))
                t1.start()

            return

        except ValueError:
            print("incomplete packet! try #" + str(tryy + 1))

    raise ValueError("INCOMPLETE PACKET 3x")


# findground subsystem: moves leg down until it touches the floor
def findGround(leg):
    global amTasks, driving, coords

    while driving:
        time.sleep(0.01)

    print("g start")
    while True:

        pres1 = None
        goal1 = None
        pres2 = None
        goal2 = None

        errorr = True

        for tryy in range(3):
            try:
                pres1 = serial_connection.get_present_position(legId[leg][1])
                goal1 = serial_connection.get_goal_position(legId[leg][1])

                pres2 = serial_connection.get_present_position(legId[leg][2])
                goal2 = serial_connection.get_goal_position(legId[leg][2])

                errorr = False

                break

            except (ValueError, TypeError):
                print("incomplete packet! try #" + str(tryy + 1))

        if errorr:
            raise ValueError("INCOMPLETE PACKET 3x")

        err1 = pres1 - goal1
        err2 = pres2 - goal2

        print(str(err1) + " 1 <==> 2 " + str(err2))

        if abs(err1) > gndThresh or abs(err2) > gndThresh:
            break

        for i in range(len(opposites)):
            if i == leg or i == opposites[leg]:
                coords[i][2] -= gndStepSize
            else:
                coords[i][2] += gndStepSize

        for i in range(len(opposites)):
            errorr = True

            for tryy in range(3):
                try:
                    moveLegs(calcRots(coords[i], i), i)

                    errorr = False

                    break

                except (ValueError, TypeError):
                    print("incomplete packet! try #" + str(tryy + 1))

            if errorr:
                raise ValueError("INCOMPLETE PACKET 3x")
            time.sleep(0.01)

        regMove.actAll(serial_connection)
        time.sleep(0.1)

    amTasks -= 1


# calibrate subsystem: moves entire body up or down to a preferred height
# also ensures that every part of the body is at least minHeight distance from the ground
def calibrateLegs():
    global coords
    avg = 0
    minn = 10000
    for i in coords:
        avg -= i[2]
        minn = min(minn, -i[2])

    diff = (walkHeight * 4 - avg) / 4.0
    minMove = minHeight - minn
    diff = max(minMove, diff)

    print("diff " + str(diff))

    for i in range(4):
        coords[i][2] -= diff
        moveLegs(calcRots(coords[i], i), i)

    regMove.actAll(serial_connection)


# levelling subsystem: moves legs up and down until robot is level
def selfLevel():
    while True:
        print("X: %.2f" % rotation[0], "----Y: %.2f" % rotation[1], "----Z: %.2f" % rotation[2])

        print("1: %.2f" % coords[0][2], "----2: %.2f" % coords[1][2])
        print("3: %.2f" % coords[2][2], "----4: %.2f" % coords[3][2])

        if abs(rotation[0]) <= gyroThresh and abs(rotation[1]) <= gyroThresh:
            break

        #     - <  > +       ^ +     v -

        #   0    1

        #   2    3

        coords[0][2] += rotation[0] * gainsUp * moveTime
        coords[2][2] += rotation[0] * gainsUp * moveTime

        coords[1][2] -= rotation[0] * gainsUp * moveTime
        coords[3][2] -= rotation[0] * gainsUp * moveTime

        coords[2][2] += rotation[1] * gainsUp * moveTime
        coords[3][2] += rotation[1] * gainsUp * moveTime

        coords[0][2] -= rotation[1] * gainsUp * moveTime
        coords[1][2] -= rotation[1] * gainsUp * moveTime

        for i in range(4):
            errorr = True
            for tryy in range(3):
                try:
                    moveLegs(calcRots(coords[i], i), i)
                    errorr = False

                    break

                except ValueError:
                    print("incomplete packet! try #" + str(tryy + 1))

            if errorr:
                raise ValueError("INCOMPLETE PACKET 3x")
            time.sleep(0.01)

        regMove.actAll(serial_connection)

        time.sleep(moveTime)
        # time.sleep(0.01)


# drives single motor of a leg
def driveLeg(leg, motor, rot):
    # if(rot*(-1 if motor == 0 and leg == 1 or leg == 2 else 1) < limits[motor][0] or rot*(-1 if motor == 0 and leg == 1 or leg == 2 else 1) > limits[motor][1]):
    #     print("Leg " + str(leg) + "-" + str(motor) + " angle out of bounds, rot= " + str(rot))
    #     raise ValueError

    toDrive = clamp(limits[motor][0], limits[motor][1], rot * (-1 if motor == 0 and leg == 1 or leg == 2 else 1)) * (
        -1 if motor == 0 and leg == 1 or leg == 2 else 1)

    regMove.regMove(serial_connection, legId[leg][motor], toDrive, speed=512, degrees=True)


# reads rotation data from gyroscope and integrates it
def updateRotation(rotation, offsets, ll):
    ll.acquire()

    import depthai as dai

    pipeline = dai.Pipeline()
    imu = pipeline.create(dai.node.IMU)
    xlinkOut = pipeline.create(dai.node.XLinkOut)

    xlinkOut.setStreamName("imu")

    # enable RAW_ACCELEROMETER and RAW_GYROSCOPE at 100 hz rate
    imu.enableIMUSensor([dai.IMUSensor.ROTATION_VECTOR], 100)
    # above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
    imu.setBatchReportThreshold(1)
    # maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    # if lower or equal to batchReportThreshold then the sending is always blocking on device
    # useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
    imu.setMaxBatchReports(10)

    imu.out.link(xlinkOut.input)

    print("init finish")

    with dai.Device(pipeline) as device:

        imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
        print("q got")

        calibrateGyro(imuQueue)
        ll.release()

        while True:
            got = readGyros(imuQueue)
            rotation[0] = got[0] - offsets[0]
            rotation[1] = got[1] - offsets[1]
            rotation[2] = got[2] - offsets[2]
            time.sleep(updateTime)


def euler_from_quaternion(i, j, k, real):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    # t0 = +2.0 * (w * x + y * z)
    # t1 = +1.0 - 2.0 * (x * x + y * y)
    # roll_x = math.atan2(t0, t1)
    #
    # t2 = +2.0 * (w * y - z * x)
    # t2 = +1.0 if t2 > +1.0 else t2
    # t2 = -1.0 if t2 < -1.0 else t2
    # pitch_y = math.asin(t2)
    #
    # t3 = +2.0 * (w * z + x * y)
    # t4 = +1.0 - 2.0 * (y * y + z * z)
    # yaw_z = math.atan2(t3, t4)
    #
    # return np.rad2deg(roll_x), np.rad2deg(pitch_y), np.rad2deg(yaw_z)

    # roll = np.arctan2(2 * y * w - 2 * x * z, 1 - 2 * y * y - 2 * z * z)  # up to down
    # pitch = np.arctan2(2 * x * w - 2 * y * z, 1 - 2 * x * x - 2 * z * z)  # turning
    # yaw = np.arcsin(2 * x * y + 2 * z * w)  # side to side rocking
    #
    # return np.rad2deg(yaw), np.rad2deg(roll), np.rad2deg(pitch)

    # 0 is real, 1 is i, 2 is j, 3 is k

    roll = np.arctan2(j*k + real*i, 1/2 - (i*i+j*j))
    pitch = np.arcsin(-2*(i*k - real*j))
    heading = np.arctan2(i*j + real*k, 1/2 - (j*j+k*k))

    return np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(heading)


# reads rotation data from gyroscope
def readGyros(q):

    imuData = q.get()
    imuPackets = imuData.packets

    rVvalues = imuPackets[len(imuPackets)-1].rotationVector

    res = euler_from_quaternion(rVvalues.i, rVvalues.j, rVvalues.k, rVvalues.real)

    tmp = [0, 0, 0]
    tmp[0] = res[0] - offsets[0]
    tmp[1] = res[1] - offsets[1]
    tmp[2] = res[2] - offsets[2]

    # print(tmp)

    return res


# calibrates gyro offsets
def calibrateGyro(imuQueue):

    print("calibrating...")
    for i in range(amCalibrate):
        got = readGyros(imuQueue)
        offsets[0] += got[0]
        offsets[1] += got[1]
        offsets[2] += got[2]
        time.sleep(0.2)
        print(i)

    offsets[0] /= amCalibrate
    offsets[1] /= amCalibrate
    offsets[2] /= amCalibrate
    print("Done! " + str(offsets[0]) + " " + str(offsets[1]) + " " + str(offsets[2]))


# get amount of digits in a number
def getDigs(inn):
    return int(math.log10(abs(int(inn))) if inn != 0 else 0) + 1 + (1 if inn < 0 else 0) + 2


# prints all essential information
def printInterface():
    global coords

    buffer = 10

    print("-" * ((1 + 4 + buffer) * 2 + 1))
    # digsone = int(math.log10(int(coords[0][0])))+1
    # digstwo = int(math.log10(int(coords[1][0])))+1
    print("|", (" " * (4 + (buffer - getDigs(coords[0][0])))), "X: %.2f" % coords[0][0], " | ",
          (" " * (4 + (buffer - getDigs(coords[1][0])))), "X: %.2f" % coords[1][0], " |")
    # digsone = int(math.log10(int(coords[0][1])))+1
    # digstwo = int(math.log10(int(coords[1][1])))+1
    print("|", "0: ", (" " * (buffer - getDigs(coords[0][1]))), "Y: %.2f" % coords[0][1], " | ", " 1: ",
          (" " * (buffer - getDigs(coords[1][1]))), "Y: %.2f" % coords[1][1], " |")
    # digsone = int(math.log10(int(coords[0][2])))+1
    # digstwo = int(math.log10(int(coords[1][2])))+1
    print("|", (" " * (4 + (buffer - getDigs(coords[0][2])))), "Z: %.2f" % coords[0][2], " | ",
          (" " * (4 + (buffer - getDigs(coords[1][2])))), "Z: %.2f" % coords[1][2], " |")

    print("-" * ((1 + 4 + buffer) * 2 + 1))
    # digsone = int(math.log10(int(coords[2][0])))+1
    # digstwo = int(math.log10(int(coords[3][0])))+1
    print("|", (" " * (4 + (buffer - getDigs(coords[2][0])))), "X: %.2f" % coords[2][0], " | ",
          (" " * (4 + (buffer - getDigs(coords[3][0])))), "X: %.2f" % coords[3][0], " |")
    # digsone = int(math.log10(int(coords[2][1])))+1
    # digstwo = int(math.log10(int(coords[3][1])))+1
    print("|", "2: ", (" " * (buffer - getDigs(coords[2][1]))), "Y: %.2f" % coords[2][1], " | ", " 3: ",
          (" " * (buffer - getDigs(coords[3][1]))), "Y: %.2f" % coords[3][1], " |")
    # digsone = int(math.log10(int(coords[2][2])))+1
    # digstwo = int(math.log10(int(coords[3][2])))+1
    print("|", (" " * (4 + (buffer - getDigs(coords[2][2])))), "Z: %.2f" % coords[2][2], " | ",
          (" " * (4 + (buffer - getDigs(coords[3][2])))), "Z: %.2f" % coords[3][2], " |")

    print("-" * ((1 + 4 + buffer) * 2 + 1))
    print("  X: %.2f" % rotation[0], "----Y: %.2f" % rotation[1], "----Z: %.2f" % rotation[2])
    print("-" * ((1 + 4 + buffer) * 2 + 1))


# clamps num to be in between inmin and inmax
def clamp(inmin, inmax, num):
    return max(inmin, min(inmax, num))


# program start
generate()
mainLoop()
