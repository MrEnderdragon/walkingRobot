import math
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

lenA = 10.875
lenB = 66.95
lenC = 118.5

posit = [0,lenA + lenB,-lenC]

def calcRots(xyz):
    x = xyz[0]
    y = xyz[1]
    z = xyz[2]

    xyAng = math.degrees(math.atan(x/y))
    xyLen = math.sqrt((x**2) + (y**2)) - lenA
    trueLen = math.sqrt((xyLen**2) + (z**2))

    zXYAng = math.degrees(math.atan(xyLen/abs(z)))
    angA = math.degrees(math.acos(((trueLen**2) + (lenB**2) - (lenC**2)) / (2*trueLen*lenB)))
    angB = math.degrees(math.acos(((lenB**2) + (lenC**2) - (trueLen**2)) / (2*lenB*lenC)))

    return [90+xyAng, 180-zXYAng-angA, angB]


def mainLoop():
    while True:
        direct = input("Input Direction: ")
        if direct == 'l':
            posit[0] += 10
        elif direct == 'r':
            posit[0] -= 10
        elif direct == 'u':
            posit[2] += 10
        elif direct == 'd':
            posit[2] -= 10
        elif direct == 'f':
            posit[1] += 10
        elif direct == 'b':
            posit[1] -= 10
        elif direct == 'exit':
            break
        elif direct == 'reset':
            posit = [0,lenA + lenB,-lenC]

        rots = calcRots(posit)

        print(str(rots[0]) + " " + str(rots[1]) + " " + str(rots[2]))
        print(str(posit[0]) + " " + str(posit[1]) + " " + str(posit[2]))
        print("---")

        kit.servo[0].angle = max(min(rots[0],180),0)
        kit.servo[1].angle = max(min(rots[1],180),0)
        kit.servo[2].angle = max(min(rots[2],180),0)

mainLoop()
