from adafruit_servokit import ServoKit
import time
import offsets as ofs
kit = ServoKit(channels=16)

while True:
  direct = int(input("position"))
  kit.servo[0].angle = direct + ofs.offsets[0][0]
  kit.servo[1].angle = direct + ofs.offsets[0][1]
  kit.servo[2].angle = direct + ofs.offsets[0][2]

  kit.servo[4].angle = direct + ofs.offsets[1][0]
  kit.servo[5].angle = direct + ofs.offsets[1][1]
  kit.servo[6].angle = direct + ofs.offsets[1][2]

  kit.servo[8].angle = direct + ofs.offsets[2][0]
  kit.servo[9].angle = direct + ofs.offsets[2][1]
  kit.servo[10].angle = direct + ofs.offsets[2][2]

  kit.servo[12].angle = direct + ofs.offsets[3][0]
  kit.servo[13].angle = direct + ofs.offsets[3][1]
  kit.servo[14].angle = direct + ofs.offsets[3][2]

time.sleep(1)
