from adafruit_servokit import ServoKit
import time
kit = ServoKit(channels=16)

offsets = [
[0,0,0],
[0,0,0],
[0,0,0],
[0,0,0]
]


if __name__ == '__main__':
    while(True):
        direct = input("Input position: ")

        if direct == "end":
            print(offsets)

        pars = direct.split()

        offsets[int(pars[0])][int(pars[1])] += int(pars[2])

        kit.servo[0].angle = 90 + offsets[0][0]
        kit.servo[1].angle = 90 + offsets[0][1]
        kit.servo[2].angle = 90 + offsets[0][2]

        kit.servo[4].angle = 90 + offsets[1][0]
        kit.servo[5].angle = 90 + offsets[1][1]
        kit.servo[6].angle = 90 + offsets[1][2]

        kit.servo[8].angle = 90 + offsets[2][0]
        kit.servo[9].angle = 90 + offsets[2][1]
        kit.servo[10].angle = 90 + offsets[2][2]

        kit.servo[12].angle = 90 + offsets[3][0]
        kit.servo[13].angle = 90 + offsets[3][1]
        kit.servo[14].angle = 90 + offsets[3][2]
