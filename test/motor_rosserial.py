import rospy
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class TorsoJointPublisher:
    def __init__(self):
        rate = rospy.get_param('~rate', 20)
        self.r = rospy.Rate(rate)

        self.pub = rospy.Publisher('/referencia', Float64, queue_size=10)
        self.msg = Float64()
        self.freq = 0.08

    def publish(self):
        # Construir mensaje
        self.msg.data = 200*math.sin(2*3.14*self.freq*rospy.get_time())
        # step
        if self.msg.data > 0:
            self.msg.data = 200
        else:
            self.msg.data = -200
        self.pub.publish(self.msg)
        #print self.msg
        self.r.sleep()


if __name__ == '__main__':
  rospy.init_node("torso_joint_publisher")

  s = TorsoJointPublisher()
  while not rospy.is_shutdown():
    s.publish()