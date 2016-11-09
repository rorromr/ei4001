#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Low level control for torso hardware using Dynamixel fieldbus protocol.
"""
__author__ = "Rodrigo Muñoz"

import time
from dynamixel_driver.dynamixel_io import DynamixelIO
import struct

class TorsoHW(object):
    """
    Low level control for torso hardware using Dynamixel fieldbus protocol.
    """
    # Memory mapping offsets
    GOAL_POSITION = 6
    MOVING_SPEED = 10
    PRESENT_POSITION = 11
    PRESENT_SPEED = 15
    EMERGENCY_STATE = 16
    LAST_POSITION = 17
    LIMITIS = 21
    PID_KP = 22
    PID_KI = 26
    PID_KV = 30

    def __init__(self, dxl_io, dev_id=1):
        """
        Class for Low level control for torso hardware using Dynamixel fieldbus protocol.
        :param dxl_io: Dynamixel interface.
        :type dxl_io: DynamixelIO
        :param dev_id: Device ID. Must be between 1 and 254.
        :type dev_id: int
        :raise TypeError: If `dxl_io` is not a DynamixelIO.
        """
        # Check arm name
        if not isinstance(dxl_io, DynamixelIO):
            raise TypeError("dxl_io must be a DynamixelIO")
        self.dxl = dxl_io
        self.id = dev_id

    def ping(self):
        """
        Send ping request.
        :raise Exception: if request fail.
        :return: Tuple with ping result and timestamp.
        """
        return self.dxl.ping(self.id)

    def get_state(self):
        """
        Get state of torso.
        :raise Exception: if request fail.
        :return: Tuple with present position, present speed and emergency state.
        """
        # Read memory block
        result = self.dxl.read(self.id, TorsoHW.PRESENT_POSITION, 6)
        raw_data = result[5:-2]
        bin_data = struct.pack('B'*len(raw_data), *raw_data)
        # int32, int8, uint8
        return struct.unpack('<ibB', bin_data)

    def get_emergency_state(self):
        """
        Get emergency state of torso.
        :raise Exception: if request fail.
        :return: Tuple with present position, present speed and emergency state.
        """
        # Read memory block
        result = self.dxl.read(self.id, TorsoHW.EMERGENCY_STATE, 1)
        return result[5:-2][0]

    def send_position_goal(self, position=0):
        """
        Send position goal.
        :raise Exception: if request fail.
        :return: Tuple with the response.
        """
        return self.dxl.write(self.id, TorsoHW.GOAL_POSITION, TorsoHW.serialize_int32(position))

    @staticmethod
    def serialize_int32(data):
        """
        Serialize int32 datatype into byte list.
        :return: Byte list.
        """
        return  [ord(a) for a in list(struct.pack('<l', int(data)))]

    @staticmethod
    def deserialize_int32(data):
        """
        Deserialize int32 datatype from byte list.
        :return: Integer obtained from list.
        """
        data_bin = struct.pack('B'*len(data), *data)
        return struct.unpack('<l', data_bin)[0]


if __name__ == '__main__':
    dxl = DynamixelIO('/dev/bender/dxl_test', baudrate=1000000)
    torso = TorsoHW(dxl, dev_id=1)
    goal = 0
    while True:
        goal += 1000
        torso.send_position_goal(goal)
        for i in range(20):
            print torso.get_state()
            time.sleep(0.1)

    #torso.send_position_goal(500)
