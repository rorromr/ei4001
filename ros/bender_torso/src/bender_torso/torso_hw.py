#!/usr/bin/python

import time
from dynamixel_driver.dynamixel_io import DynamixelIO
import struct

# NON ROS HARDWARE INTERFACE

class TorsoHW(object):
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

    def __init__(self, dxl_io, dev_id = 1):
        self.dxl = dxl_io
        self.id = dev_id
        self.state = 0

    def ping(self):
        result = []
        try:
            result = self.dxl.ping(self.id)
        except Exception as e:
            print 'Exception thrown while pinging device {} - {}'.format(self.id, e)
            raise e
        return result

    def get_state(self):
        result = []
        try:
            # Read memory block
            result = self.dxl.read(self.id, TorsoHW.PRESENT_POSITION, 6)
        except Exception as e:
            print 'Exception thrown while reading addres {}'.format(TorsoHW.PRESENT_POSITION)
            return e
        raw_data = result[5:-2]
        bin_data = struct.pack('B' * len(raw_data), *raw_data)
        # int32, int8, uint8
        return struct.unpack('<ibB', bin_data)

    def set_goal_position(self, position = 0):
        result = []
        try:
            result = self.dxl.write(self.id, TorsoHW.GOAL_POSITION, TorsoHW.serialize_int32(position))
        except Exception as e:
            print 'Exception thrown while writing addres {}'.format(TorsoHW.GOAL_POSITION)
            raise e
        return result

    @staticmethod
    def serialize_int32(data):
        return  [ord(a) for a in list(struct.pack('<l',int(data)))]

    @staticmethod
    def deserialize_int32(data):
        data_bin = struct.pack('B' * len(data), *data)
        return struct.unpack('<l', data_bin)[0]


if __name__ == '__main__':
    import math
    dxl = DynamixelIO('/dev/bender/dxl_test', baudrate = 1000000)
    torso = TorsoHW(dxl, dev_id = 1)
    goal = 0
    while True:
        goal += 1000
        torso.set_goal_position(goal)
        for i in range(20):
            print torso.get_state()
            time.sleep(0.1)

    #torso.set_goal_position(500)


