#!/usr/bin/python

__author__ = 'Rodrigo Munoz'

import math
import rospy

from threading import Thread

from std_msgs.msg import Int32

# Use HW interface
from bender_torso.torso_hw import TorsoHW

class TorsoController:
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        # Only argument stuff
        self.running = False
        self.dxl_io = dxl_io
        self.controller_namespace = controller_namespace
        self.port_namespace = port_namespace
        
    def initialize(self):
        # Get params and allocate msgs
        self.state_update_rate = rospy.get_param(self.controller_namespace + '/rate', 200)
        self.dev_id = rospy.get_param(self.controller_namespace + '/id', 1)
        self.torso = TorsoHW(self.dxl_io, self.dev_id)
        self.state = Int32()
        self.state.data = 0

    def start(self):
        # Create subs, services, publishers, threads
        self.running = True
        self.command_sub = rospy.Subscriber(self.controller_namespace + '/command', Int32, self.process_command)
        self.state_pub = rospy.Publisher(self.controller_namespace + '/state', Int32, queue_size = 5)
        Thread(target=self.update_state).start()

    def stop(self):
        self.running = False
        self.command_sub.unregister()
        self.state_pub.unregister()

    def process_command(self, msg):
        # TODO(rorromr) Add limits
        try:
            self.torso.send_position_goal(msg.data)
        except Exception:
            rospy.logerr("Error sending position goal.")

    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():
            try:
                current_state = self.torso.get_state()
            except Exception:
                rospy.logwarn("Error reading state.")
                continue
            self.state.data = current_state[0]
            # Publish current position
            self.state_pub.publish(self.state)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('led_controller')
    from dynamixel_driver.dynamixel_io import DynamixelIO
    dxl = DynamixelIO('/dev/bender/dxl_test', baudrate=1000000)
    torso = TorsoController(dxl, 'torso', 'torso')
    torso.initialize()
    torso.start()
    rospy.spin()
    torso.stop()
        
