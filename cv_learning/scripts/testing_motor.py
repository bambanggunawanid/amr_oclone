#|user/bin/env

# from socket import MsgFlag
# import numpy as np
from numpy import rate
import rospy
from std_msgs.msg import Int16
# import keyboard

def talk_to_me():
  pub = rospy.Publisher('set_tilt_degree', Int16, queue_size=5 )
  rospy.init_node('publisher_node', anonymous=True)
  rate = rospy.Rate(10)
  rospy.loginfo("Controlling Kinect Motor")
  while not rospy.is_shutdown():
    degree=-10
    msg = degree
    pub.publish(msg)
    rate.sleep

if __name__=='__main__':
  try:
    talk_to_me()
  except rospy.ROSInterruptException:
    pass