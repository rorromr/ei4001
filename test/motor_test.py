#!/usr/bin/python
# -*- coding: iso-8859-1 -*-
import sys
import time
import struct
from dynamixel_driver.dynamixel_io import DynamixelIO

def test_ping(dxl, id):
    test_ids = [7, 8, 16, 32, 45, 37, 72]
    for i in range(15):
        if dxl.ping(test_ids[i%len(test_ids)]):
            print "[FAIL]"

        if not dxl.ping(id):
            print "[FAIL]"

def test_read(dxl, id):
    address = [3,4]
    values = [15,207]
    for i in range(10):
        j = i%len(address)
        try:
            if values[j] != dxl.read(id,address[j],1)[-3]:
                print "[FAIL]"
        except:
            print "[FAIL]"

def test_write(dxl, id):
    values = [10,15,25,36]
    for v in values:
        dxl.write(id, 7, [v])
        r = dxl.read(id,7,1)[-3]
        print r
        if r != v:
            print "[FAIL]"

def get_encoder(dxl, id):
    b = [0,0,0,0]
    b[0] = dxl.read(id,9,1)[-3]
    b[1] = dxl.read(id,10,1)[-3]
    b[2] = dxl.read(id,11,1)[-3]
    b[3] = dxl.read(id,12,1)[-3]
    return struct.unpack('<i',struct.pack('BBBB',b[3],b[2],b[1],b[0]))[0]
            
def main():
    dxl = DynamixelIO('/dev/ttyUSB0', baudrate = 9600)
    
    # command = int(sys.argv[1])
    # print 'Command: ' + str(command)
    
    # [0xFF, 0xFF, ID, Length, Instruction, Parameters, Checksum]
    test_ping(dxl, 5)
    #test_write(dxl, 15)
    print dxl.ping(5)

    dxl.write(5,7,[0])
    
    while True:
        print get_encoder(dxl, 5)
        dxl.write(5,8,[1])
        time.sleep(1)

        print get_encoder(dxl, 5)
        dxl.write(5,8,[0])
        time.sleep(1)
    



if __name__ == '__main__':
    main()
