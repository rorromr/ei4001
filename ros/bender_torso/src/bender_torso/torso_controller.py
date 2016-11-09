#!/usr/bin/python

__author__ = 'Rodrigo Munoz'

import math
import rospy

from threading import Thread

from std_msgs.msg import Int32

# Use HW interface
from bender_torso.torso_hw import TorsoHW
import numpy as np
import time
import math

class TorsoController:
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        # Only argument stuff
        self.running = False
        self.dxl_io = dxl_io
        self.controller_namespace = controller_namespace
        self.port_namespace = port_namespace
        
    def initialize(self):
        # Get params and allocate msgs
        self.state_update_rate = rospy.get_param(self.controller_namespace + '/rate', 400)
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
        vel = 5000.0/1.8 # Velocity
        dist = abs(msg.data-self.state.data)
        tf = dist/vel
        segments = int(dist/25)
        data = self.get_parabolic_trajectory(tf=tf, qf=msg.data, q0=self.state.data, segments=segments)
        pos = data[1]
        for point in pos:
            try:
                self.torso.send_position_goal(int(point))
            except Exception:
                rospy.logerr("Error sending position goal.")
            time.sleep(tf/segments)

    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        error_count = 0
        while self.running and not rospy.is_shutdown():
            try:
                #self.torso.get_emergency_state()
                # if not self.torso.ping():
                #     raise Exception("Ping fail")
                current_state = self.torso.get_state()
            except Exception:
                error_count += 1
                rospy.logwarn("Error reading state {}.".format(error_count))
                rospy.sleep(0.05)
                continue
            self.state.data = current_state[0]
            # Publish current position
            self.state_pub.publish(self.state)
            rate.sleep()

    def get_parabolic_trajectory(self, tf, qf, qqf=0, q0=0, qq0=0, segments=50):
        # Polynomial coeficients
        a0=q0
        a1=qq0
        a2=(3.0*(qf-q0)-tf*(2*qq0+qqf))/(tf**2)
        a3=(2.0*(q0-qf)+tf*(qq0+qqf))/(tf**3)
        # Time vector
        t=np.linspace(0,tf,segments)
        t2=t*t
        t3=t2*t
        return (t,a0+a1*t+a2*t2+a3*t3, a1+2*a2*t+3*a3*t2, 2*a2+6*a3*t)

if __name__ == '__main__':
    rospy.init_node('led_controller')
    from dynamixel_driver.dynamixel_io import DynamixelIO
    dxl = DynamixelIO('/dev/bender/dxl_test', baudrate=1000000)
    torso = TorsoController(dxl, 'torso', 'torso')
    torso.initialize()
    torso.start()
    rospy.spin()
    torso.stop()
        
