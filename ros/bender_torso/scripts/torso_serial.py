#!/usr/bin/env python

"""
  torso_joint_state_publisher.py
  
  Publish the torso_controller joint states on the /bender/torso/joint_states topic
  
  Basado en software de Patrick Goebel. Pi Robot Project.

"""

import rospy
import math
from dynamixel_driver.dynamixel_io import DynamixelIO
from sensor_msgs.msg import JointState

class TorsoJointPublisher:
    def __init__(self):
        self.dxl = DynamixelIO('/dev/ttyUSB1', baudrate = 9600)

        rate = rospy.get_param('~rate', 60)
        self.r = rospy.Rate(rate)

        self.pub = rospy.Publisher('/bender/torso/joint_states', JointState, queue_size=10)

        rospy.loginfo("Starting Torso Joint State Publisher at " + str(rate) + "Hz")
        self.msg = JointState()
        self.msg.name = ["torso_lift_joint"]
        self.msg.velocity = [0.0]
        self.msg.effort = [0.0]

    def publish_joint_states(self, pos):
        # Construir mensaje
        try:
          value = self.dxl.read(15, 6, 1)[-3]
          #print value
        except:
          value = 100

        self.msg.header.stamp = rospy.Time.now()
        self.msg.position = [value*1.0/200.0*0.65-0.25]
        self.pub.publish(self.msg)
        #print self.msg
        self.r.sleep()


if __name__ == '__main__':
  rospy.init_node("torso_joint_publisher")

  s = TorsoJointPublisher()
  while not rospy.is_shutdown():
    s.publish_joint_states(.4-(0.5+0.5*math.sin(rospy.get_time()))*.65)
    