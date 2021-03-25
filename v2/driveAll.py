from pyax12.connection import Connection
import time
import sys


# Connect to the serial port
serial_connection = Connection(port="/dev/ttyAMA0", rpi_gpio=True, baudrate=1000000)


startId = int(sys.argv[1])
endId = int(sys.argv[2])
deg = int(sys.argv[3])


for dynamixel_id in range(startId, endId+1):
    serial_connection.goto(dynamixel_id, deg, speed=512, degrees=True)
    time.sleep(0.5)

# Close the serial connection
serial_connection.close()
