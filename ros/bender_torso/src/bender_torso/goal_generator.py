#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'

import math
# ROS
import rospy
# Msgs
from std_msgs.msg import Float64
# Dynamic reconfigure
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from bender_torso.cfg import GoalGeneratorConfig

class GoalGenerator():
  def __init__(self, rate = 20):
    self.rate = rospy.Rate(rate)
    self.dt = 1.0/rate;
    self.topic_name = 'cmd'
    self.pub = rospy.Publisher(self.topic_name, Float64, queue_size = 5)
    # Dynamic reconfigure server, load dinamic parameters
    self.reconfig_server = DynamicReconfigureServer(GoalGeneratorConfig, self.update_params)
    self.value = 0.0
    
  def update_params(self, config, level):
    # Update params
    self.frec = config.frec
    self.offset = config.offset
    self.amplitude = config.amplitude
    self.type = config.type
    self.sat_en = config.sat_en
    self.sat = config.sat
    if(self.topic_name != config.topic):
      rospy.loginfo("New config topic {}".format(config.topic))
      self.topic_name = config.topic
      self.pub = rospy.Publisher(self.topic_name, Float64, queue_size = 5)
    return config

  def update_value(self):
    if self.type == 0:
      self.sine()
    elif self.type == 1:
      self.square()
    elif self.type == 2:
      self.triangular()

    if self.sat_en:
      self.value = max(min(self.value, self.sat), -self.sat)

  def sine(self):
    self.value = self.amplitude*math.sin(2*math.pi*self.frec*rospy.Time.now().to_sec())+self.offset

  def square(self):
    if math.sin(2*math.pi*self.frec*rospy.Time.now().to_sec()) > 0:
      self.value = self.amplitude+self.offset
    else:
      self.value = -self.amplitude+self.offset

  def triangular(self):
    if math.sin(2*math.pi*self.frec*rospy.Time.now().to_sec()) > 0:
      self.value += 10*(self.amplitude+self.offset)*self.dt
    else:
      self.value += -10*(self.amplitude+self.offset)*self.dt

  def run(self):
    while not rospy.is_shutdown():
      self.update_value()
      self.pub.publish(data=self.value)
      self.rate.sleep()

def main():
  rospy.init_node('goal_generator')
  generator = GoalGenerator(rate = 20)
  rospy.loginfo('Running GoalGenerator in topic: {}'.format(generator.topic_name))
  generator.run()


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException as e:
    rospy.logerr('Exception on GoalGenerator')
    rospy.logerr(e)