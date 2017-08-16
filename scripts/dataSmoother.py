#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def smoother(data) : 
    

if __name__ == '__main__' :
  try:
    rospy.init_node('dataSmootherNode')
    velocitySubscriber=rospy.Subscriber('cmd_vel',Twist,smoother)
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
