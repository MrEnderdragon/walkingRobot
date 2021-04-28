import math
import time
from pyax12.connection import Connection
import pyax12.utils as utils
import pyax12.instruction_packet as ip
import pyax12.packet as pk
import sys

def regWrite(serial_connection, dynamixel_id, address, data):
    bytes_address = bytes((address, ))

    if isinstance(data, int):
        bytes_to_write = bytes((data, ))
    else:
        bytes_to_write = bytes(data)

    instruction = ip.REG_WRITE
    params = bytes_address + bytes_to_write
    inst_packet = ip.InstructionPacket(dynamixel_id, instruction, params)

    serial_connection.send(inst_packet)

def regMove(serial_connection, dynamixel_id, position, speed=None, degrees=False):
    if degrees:
        position = utils.degrees_to_dxl_angle(position)

    params = utils.int_to_little_endian_bytes(position)

    if speed is not None:
        params += utils.int_to_little_endian_bytes(speed)

    regWrite(serial_connection, dynamixel_id, pk.GOAL_POSITION, params)

def action(serial_connection, dynamixel_id):
    inst_packet = ip.InstructionPacket(dynamixel_id, ip.ACTION, None)
    serial_connection.send(inst_packet)

def actAll(serial_connection):
    inst_packet = ip.InstructionPacket(254, ip.ACTION, None)
    serial_connection.send(inst_packet)
