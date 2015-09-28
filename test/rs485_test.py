#!/usr/bin/python
# -*- coding: iso-8859-1 -*-
import serial
import time
from array import array
import sys

ser = serial.Serial('/dev/ttyUSB0', baudrate = 9600, timeout=1)
# [0xFF, 0xFF, ID, Length, Instruction, Parameters, Checksum]
command = ord(str(sys.argv[1])[0])
print 'Command: ' + str(command)
packet = [0xFF, 0xFF, 30, 2, 1, command, 1]
print packet

data = array('B', packet).tostring() # packetStr = ''.join([chr(byte) for byte in packet])

ser.write(data)
  
