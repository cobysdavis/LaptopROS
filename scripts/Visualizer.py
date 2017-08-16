#!/usr/bin/env python
import math
import rospy
import time
from geometry_msgs.msg import Pose2D
from ReactiveRobot import *
from robot.msg import ReactiveRobotState
from robot.msg import PathMessage
from graphics import *
class Window():
	def __init__(self):
		self.bound=4
		self.size=600
		self.factor=self.size/(self.bound*2)
		self.win=GraphWin("Robots path",self.size,self.size)
		size=self.size
		bound=self.bound
		factor=self.factor
		self.win.setBackground('white')
		line=Line(Point(size/2,0),Point(size/2,size))
		line.draw(self.win)
		line=Line(Point(0,size/2),Point(size,size/2))
		line.draw(self.win)
		for i in range(-bound+1,bound):
			Text(Point(size/2,(i+bound)*factor),str(i)).draw(self.win)
			Text(Point((i+bound)*factor,size/2),str(i)).draw(self.win)
		self.c=Circle(Point(size/2,size/2),15)
		self.c.draw(self.win)
		self.pose=Pose2D()
		

	def draw(self,state) :
		#self.c.move(self.pose.x-state.pose.x,self.pose.y-state.pose.y)
		#time.sleep(.05)
		p=Point(state.pose.x,state.pose.y)
		p.draw(self.win)
		#self.pose=state.pose


if __name__ == '__main__' :
	window=Window()
	try:
		rospy.init_node('Visualizer')
		stateSubscriber=rospy.Subscriber('RobotState',ReactiveRobotState,window.draw)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass


