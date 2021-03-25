import math
import time
from pyax12.connection import Connection
import pyax12.utils as utils
import regMove
import sys

serial_connection = Connection(port="/dev/ttyAMA0", rpi_gpio=True, baudrate=1000000)

startId = int(sys.argv[1])
endId = int(sys.argv[2])

gain = 1
thresh = 1

while True:

    tWait = 0

    for dynamixel_id in range(startId, endId+1):
        pres = serial_connection.get_present_position(dynamixel_id)
        goal = serial_connection.get_goal_position(dynamixel_id)
        err = pres-goal
        if abs(err) <= thresh:
            continue
        goto = utils.dxl_angle_to_degrees(goal)+err*gain
        print(str(dynamixel_id) + " - " + str(err))
        # regMove.regMove(serial_connection, dynamixel_id, goto , speed=512, degrees=True)
        regMove.regMove(serial_connection, dynamixel_id, utils.dxl_angle_to_degrees(pres) , speed=512, degrees=True)

        # tWait = max(tWait, abs(utils.dxl_angle_to_degrees(goal)+err*gain - utils.dxl_angle_to_degrees(pres))/10)

        time.sleep(0.01)

    regMove.actAll(serial_connection)
    # print(tWait)
    time.sleep(tWait+0.01)
