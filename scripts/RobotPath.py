#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import Pose2D
from ReactiveRobot import *
from slam_robot.msg import ReactiveRobotState
from slam_robot.msg import PathMessage
from graphics import *

class List :
	def __init__(self,pathMessage):
		self.list=pathMessage.path
		self.distTolerance=0.1
		self.angleTolerance=15
		self.totalPoints=len(pathMessage.path)
		self.listLoc=0



	def sendGoal(self,state) : 
		distToGoal=math.hypot(state.pose.x-state.goal.x,state.pose.y-state.goal.y)
		thetaToGoal=math.fabs(state.pose.theta-state.goal.theta)
		print "current goal: "+ str(self.listLoc) + " Distance: " +str(distToGoal) + " Theta: " + str(thetaToGoal)
		print self.listLoc

		if distToGoal>self.distTolerance or not state.state=="1" or self.listLoc>=self.totalPoints : 
			print "passing"
			pass
		else :
			print "moving, list location: " + str(self.listLoc)
			print "to : " + str(self.list[self.listLoc])
			commandPublisher.publish(self.list[self.listLoc])
			self.listLoc+=1

def listGen(list,manTheta):
	new=[]
	for i in range(0,len(list)):
		point=Pose2D()
		point.x=list[i][0]
		point.y=list[i][1]
		if manTheta==1 :
			point.theta=list[i][2]
		else :
			if i<len(list)-1:
				dx=list[i+1][0]-point.x
				dy=list[i+1][1]-point.y	
				phi=math.degrees(math.atan2(dy,dx))
				point.theta=phi%360
		new.append(point)
	print new
	return new



if __name__ == '__main__' :
	

	n=input("How many points do you want to go to? ")
	print n
	manualTheta=0
	TypeOfInput=input("Type 0 if you want to type your path and 1 if you want to type and also specify theta cords. type any other numer if you want to draw your path. Press enter")
	list=[]
	if TypeOfInput==0:
		for i in range(0,n):
			x=input("enter x"+str(i) + ": ")
			y=input("enter y"+str(i) + ": ")
			list.append((x,y))
		print list
	if TypeOfInput==1:
		for i in range(0,n):
			x=input("enter x "+str(i) + ": ")
			y=input("enter y "+str(i) + ": ")
			theta=input("enter theta "+str(i) + ": ")
			manualTheta=1
			list.append((x,y,theta))
		print list
	else:
		bound=4
		size=900
		factor=size/(bound*2)
		origin=Point(size/2,size/2)
		win = GraphWin('Draw your path', size, size)
		win.setBackground('white')
		line=Line(Point(size/2,0),Point(size/2,size))
		line.draw(win)
		line=Line(Point(0,size/2),Point(size,size/2))
		line.draw(win)
		for i in range(-bound+1,bound):
			Text(Point(size/2,(i+bound)*factor),str(-i)).draw(win)
			Text(Point((i+bound)*factor,size/2),str(i)).draw(win)
		#Text(Point(size/2,15),"Y").draw(win)
		#Text(Point(size-15,size/2),"X").draw(win)
		message = Text(Point(win.getWidth()/2, 30), 'Click on ' + str(n) + ' points')
		message.setTextColor('red')
		message.setSize(20)
		message.draw(win)
		points=[]

		for i in range(0,n):
			p=win.getMouse()
			print "point " + str(i)
			print p
			points.append(p)
			list.append(((p.x-origin.x)/factor,-(p.y-origin.y)/factor))
			pointText=Text(p,"Point " + str(i) +": " + str("{0:.{1}f}".format(list[i][0],2)) + "," + str("{0:.{1}f}".format(list[i][1],2)))
			pointText.setSize(10)
			pointText.draw(win)
		line.setArrow("last")
		for i in range(0,n-1):
			line = Line(points[i],points[i+1])
			line.draw(win)
			line.setArrow("last")

		#win.getMouse()
		#win.close() 
	map=listGen(list,manualTheta)
	pathMsg=PathMessage()
	pathMsg.path=map
	print "path: "
	print map
	try:
		rospy.init_node('RobotPath',log_level=rospy.DEBUG)
		points=List(pathMsg)
		stateSubscriber=rospy.Subscriber('RobotState',ReactiveRobotState,points.sendGoal)
		commandPublisher=rospy.Publisher('CommandLoc',Pose2D,queue_size=3)
		#drawPub=rospy.Publisher('Path',PathMessage,queue_size=3)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
