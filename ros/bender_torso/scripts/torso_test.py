#!/usr/bin/env python

"""
  torso_joint_state_publisher.py
  
  Publish the torso_controller joint states on the /bender/torso/joint_states topic

"""

import rospy
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

class TorsoJointPublisher:
  def __init__(self):
    rate = rospy.get_param('~rate', 20)
    self.r = rospy.Rate(rate)

    self.sus = rospy.Subscriber('/torso_state', Int32, self.publish_joint_states, queue_size=10)
    self.pub = rospy.Publisher('/bender/torso/joint_states', JointState, queue_size=10)

    rospy.loginfo("Starting Torso Joint State Publisher at " + str(rate) + "Hz")
    self.msg = JointState()
    self.msg.name = ["torso_lift_joint"]
    self.msg.velocity = [0.0]
    self.msg.effort = [0.0]


  def publish_joint_states(self, pos):
    # Construir mensaje
    self.msg.header.stamp = rospy.Time.now()
    self.msg.position = [1.0*pos.data/5000.0]
    self.pub.publish(self.msg)
    #print self.msg


if __name__ == '__main__':
  rospy.init_node("torso_joint_publisher")

  s = TorsoJointPublisher()
  rospy.spin()
