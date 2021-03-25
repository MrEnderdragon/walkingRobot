import math
import time
from pyax12.connection import Connection
import pyax12.utils as utils
import pyax12.instruction_packet as ip
import pyax12.packet as pk
import sys

serial_connection = Connection(port="/dev/ttyAMA0", rpi_gpio=True, baudrate=1000000)

def regWrite(dynamixel_id, address, data):
    bytes_address = bytes((address, ))

    if isinstance(data, int):
        bytes_to_write = bytes((data, ))
    else:
        bytes_to_write = bytes(data)

    instruction = ip.REG_WRITE
    params = bytes_address + bytes_to_write
    inst_packet = ip.InstructionPacket(dynamixel_id, instruction, params)

    serial_connection.send(inst_packet)

def regMove(dynamixel_id, position, speed=None, degrees=False):
    if degrees:
        position = utils.degrees_to_dxl_angle(position)

    params = utils.int_to_little_endian_bytes(position)

    if speed is not None:
        params += utils.int_to_little_endian_bytes(speed)

    regWrite(dynamixel_id, pk.GOAL_POSITION, params)

def action(dynamixel_id):
    inst_packet = ip.InstructionPacket(dynamixel_id, ip.ACTION, None)
    serial_connection.send(inst_packet)


startId = int(sys.argv[1])
endId = int(sys.argv[2])
deg = int(sys.argv[3])


for dynamixel_id in range(startId, endId+1):
    regMove(dynamixel_id, deg, speed=512, degrees=True)
    time.sleep(0.5)

action(254)

time.sleep(5)

# Close the serial connection
serial_connection.close()
