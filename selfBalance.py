import smbus            #import SMBus module of I2C
from time import sleep          #import
import threading
import offsets as ofs
from adafruit_servokit import ServoKit
import math
kit = ServoKit(channels=16)

#some MPU6050 Registers and their Address
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

lenA = 10.875 # lenth of shoulder motor1 to shoulder motor 2, projected onto the ground plane
lenB = 46.95 # lenth of upper arm
lenC = 68.5 # length of lower arm

offsets = [0,0,0]
rotation = [0,0,0]

pos = [
[50,50,-lenC] for i in range(4)
]

amCalibrate = 10
updateTime = 0.05
moveTime = 0.1
gainsUp = 1
gainsSide = 1

def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    #Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

    #Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)

    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

    #Write to interrupt enable register
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

def update ():
    while(True):
        got = readGyros()
        rotation[0] += got[0]*updateTime - offsets[0]*updateTime
        rotation[1] += got[1]*updateTime - offsets[1]*updateTime
        rotation[2] += got[2]*updateTime - offsets[2]*updateTime
        sleep(updateTime)

def calibrate () :
    print("calibrating...")
    for i in range(amCalibrate):
        got = readGyros()
        offsets[0] += got[0]*0.2
        offsets[1] += got[1]*0.2
        offsets[2] += got[2]*0.2
        sleep(0.2)
        print(i)

    offsets[0] /= (amCalibrate*0.2)
    offsets[1] /= (amCalibrate*0.2)
    offsets[2] /= (amCalibrate*0.2)
    print("Done! " + str(offsets))

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
    tmpAngA = ((trueLen**2) + (lenB**2) - (lenC**2)) / (2*trueLen*lenB)

    angA = 0

    if tmpAngA >= 1:
        angA = 0
    elif tmpAngA <= -1:
        angA = 180
    else:
        angA = math.degrees(math.acos(tmpAngA)) # angle of top arm from vector (shoulder part 2)

    tmpAngB = ((lenB**2) + (lenC**2) - (trueLen**2)) / (2*lenB*lenC)

    angB = 0

    if tmpAngB >= 1:
        angB = 0
    elif tmpAngB <= -1:
        angB = 180
    else:
        angB = math.degrees(math.acos(tmpAngB)) # angle of top arm from vector (shoulder part 2)

    # angB = math.degrees(math.acos(((lenB**2) + (lenC**2) - (trueLen**2)) / (2*lenB*lenC))) # angle of bottom arm from top arm (elbow)

    return [max(min(90+xyAng,160),20), max(min(180-zXYAng-angA,160),20), max(min(angB,160),20)] # angle at elbow has to be inverted

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


bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

for i in range(4):
    moveLegs(calcRots(pos[i],i),i)

sleep(2)

calibrate()

t1 = threading.Thread(target = update, args = ())
t1.start()

while(True):
    print("X: %2f" %rotation[0], "----Y: %2f" %rotation[1], "----Z: %2f" %rotation[2])

    pos[0][2] += rotation[0]*gainsUp*moveTime
    pos[2][2] += rotation[0]*gainsUp*moveTime

    pos[1][2] -= rotation[0]*gainsUp*moveTime
    pos[3][2] -= rotation[0]*gainsUp*moveTime


    pos[2][2] += rotation[1]*gainsUp*moveTime
    pos[3][2] += rotation[1]*gainsUp*moveTime

    pos[0][2] -= rotation[1]*gainsUp*moveTime
    pos[1][2] -= rotation[1]*gainsUp*moveTime

    for i in range(4):
        moveLegs(calcRots(pos[i],i),i)

    sleep(moveTime)
