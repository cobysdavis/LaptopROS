#!/usr/bin/env python
import math
import rospy
from slam_robot.srv import StartOrStop,StartOrStopResponse
from slam_robot.msg import ReactiveRobotState
from slam_robot.srv import SetGoalOrPose,SetGoalOrPoseResponse,SetGoalOrPoseRequest
from slam_robot.srv import QueryPose,QueryPoseResponse,QueryPoseRequest
from slam_robot.srv import QueryState,QueryStateResponse,QueryStateRequest
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_srvs.srv import Empty
from ReactiveRobot import *

class ReactiveWrapper:
  "Encapsulation of a reactive robot"


  def __init__(self, host = "robotworld.cse.yorku.ca", port = 20000):
    print "connect to robot " + host + " at port " + str(port)
    self._robot=ReactiveRobot(host,port)
    self.currentGoal=Pose2D(0,0,0)
    self.currentPose=Pose2D(0,0,0)
    self.posLogo=Pose2D(0,0,0)
    self.angleLogo=Pose2D(0,0,0)
#keeps track of the number of different goals given to the robot 
    self.goalSet=0
#factors that control the weighting of joystick commands. these were determined through experiment
    self.linearFactor=0.2
    self.angularFactor=10
#sequence number to keep track of the robot state
    self.seq=0	
    print ReactiveRobot(host, port)

  def isAlive(self) :
    return self._robot.isAlive()

  def startUp(self, args) :
    print "Attempting to startup the robot"
    if not self.isAlive() :
      return StartOrStopResponse("already running")
    try:
      self._robot.startup()
      alivePub.publish("Alive")
      return StartOrStopResponse("started up")
    except socket.error:
      print "Unable to startup the robot"
      return StartOrStopResponse("error")

  def getPose(self,args) :
    try:
      resp=RobotPose()
      resp=self._robot.queryPose()
      self.currentPose.x=resp.x
      self.currentPose.y=resp.y
      self.currentPose.theta=resp.t%360
      print "current pose: " + str(self.currentPose)
      return QueryPoseResponse(self.currentPose)
    except socket.error:
      print "Unable to query the pose"
      return QueryPoseResponse("error")

  def getGoal(self,args) :
    try:
      print "new goal: " + str(self.currentGoal)
      return QueryPoseResponse(self.currentGoal)
    except socket.error:
      print "Unable to query the goal"
      return QueryPoseResponse("error")

  def disconnect(self) :
    self._theSocket.close()

  def shutDown(self,args) :
    print "Shut the robot down"
    if not self.isAlive() :
      return StartOrStopResponse("Robot already shutdown")
    try:
      self._robot.shutdown()
      return StartOrStopResponse("Shut down success")
    except socket.error:
      rospy.logerr("Unable to shut the robot down")
      return StartOrStopResponse("Shut down failed")

  def setPose(self, r) :
    self.currentPose.x=r.goal.x
    self.currentPose.y=r.goal.y
    self.currentPose.theta=r.goal.theta
    command=RobotPose()
    command.x=r.goal.x
    command.t=r.goal.y
    command.t=r.goal.theta
    if not self.isAlive() :
      return SetGoalOrPoseResponse("Robot is not alive")
    try:
      self._robot.setPose(command)
      return SetGoalOrPoseResponse("pose set")
    except socket.error:
      print "Socket error"
      return SetGoalOrPoseResponse("failure")

  def setGoal(self, r) :
    self.goalSet+=1
    print "number of goals set: " + str(self.goalSet)
    pose=RobotPose()
    pose.x=r.goal.x
    pose.y=r.goal.y
    pose.t=r.goal.theta
    self.currentGoal.x=pose.x
    self.currentGoal.y=pose.y
    self.currentGoal.theta=pose.t%360
    if not self.isAlive() :
      return SetGoalOrPoseResponse("Robot is not alive, failure")
    try:
      print "Setting the goal to: " + str(pose)
      self._robot.setGoal(pose)
      return SetGoalOrPoseResponse("goal set")
    except socket.error:
      print "Socket error"
      return SetGoalOrPoseResponse("failure, socket error")


  def calcDist(self,x1,y1,x2,y2) : 
    return(((x1-x2)**2 + (y1-y2)**2)**0.5)

#called when joystick issues a valid command
  def joyMove(self,vel) :
#publish current state and returns if the robot is currently moving or not
    moving=self.statePub(1)
#command key to startup the robot
    if vel.linear.y==1 :
      self.startUp("starting up")
#gets current pose (also updates attribute "self.currentPose")
    self.getPose("getting pose")
#command key to shutdown the robot
    if vel.linear.z==1 :
      self.shutDown("shutting down")
#command key to set the goal to where the robot is 
    elif vel.angular.x==1:
      goal=SetGoalOrPoseRequest()
      goal.goal=self.currentGoal
      self.setGoal(goal)
#set current pose and goal to (0,0,0)--reset
    elif vel.angular.y==1:
      zero=SetGoalOrPoseRequest()
      zero.goal.x=0
      zero.goal.y=0
      zero.goal.theta=0
      self.setPose(zero)
      self.setGoal(zero)
#if no valid commands are issued do nothing
    elif vel.linear.x==0 and vel.angular.z==0 :
      print "do nothing, no instructions given"
    elif not moving=="1" :
#safeguard to make sure robot does not accept any instructions until it has reached its current goal
      print "need to get closer to current goal or robot is blocked"
    else :
#otherwise, using the joystick data, perform the specified movement (rotation or translation)
      command=SetGoalOrPoseRequest()
      command.goal.x=self.currentGoal.x + self.linearFactor*vel.linear.x*math.cos(math.radians(self.currentPose.theta))
      command.goal.y=self.currentGoal.y + self.linearFactor*vel.linear.x*math.sin(math.radians(self.currentPose.theta))
      command.goal.theta=(self.angularFactor*vel.angular.z+self.currentGoal.theta)%360
      if vel.linear.x<0 :
        command.goal.theta+=180
        command.goal.theta=command.goal.theta%360
      self.setGoal(command)

#called when a list of points is sent by the RobotPath node
  def commandMove(self,point) :
    command=SetGoalOrPoseRequest()
    command.goal.x=point.x
    command.goal.y=point.y
    command.goal.theta=point.theta
    self.setGoal(command)

#publishes robot state
  def statePub(self,count) :
    self.getPose("getting pose")
    state=ReactiveRobotState()
    state.pose.x=self.currentPose.x
    state.pose.y=self.currentPose.y
    state.pose.theta=self.currentPose.theta
    state.goal.x=self.currentGoal.x
    state.goal.y=self.currentGoal.y
    state.goal.theta=self.currentGoal.theta
    state.header.seq=self.seq
    self.seq+=1
    state.header.stamp=rospy.Time.now()
    state.header.frame_id="frame id"
    state.state=str(self._robot.queryState())
    statePublisher.publish(state)
    return state.state

  def setPoseRelativeToLogo(self,data):
      #logo defined as (0,0,0)
      angleLogo.x=data.data[0]
      angleLogo.y=data.data[1]
      angleLogo.z=data.data[2]
      posLogo.x=data.data[3]
      posLogo.y=data.data[4]
      posLogo.z=data.data[5]

if __name__ == '__main__' :
  print "ReactiveWrapper starting up"
  try:
    rospy.init_node('ReactiveWrapper')
    wrapper = ReactiveWrapper('127.0.0.1', 20000)

#services
    wakeService=rospy.Service('startUpService', StartOrStop, wrapper.startUp)
    queryPoseService=rospy.Service('queryPoseService', QueryPose, wrapper.getPose)
    queryGoalService=rospy.Service('queryGoalService', QueryPose, wrapper.getGoal)
    queryAlive=rospy.Service('queryAlive', Empty, wrapper.isAlive)
    shutDownService=rospy.Service('shutDownService',StartOrStop,wrapper.shutDown)
    setGoalService=rospy.Service('setGoalService',SetGoalOrPose,wrapper.setGoal)
    setPoseService=rospy.Service('setPoseService',SetGoalOrPose,wrapper.setPose)

#subscriptions
    matrixSubscriber=rospy.Subscriber('matrix',Float64MultiArray,wrapper.setPoseRelativeToLogo)
    velocitySubscriber=rospy.Subscriber('cmd_vel',Twist,wrapper.joyMove)
    commandSubscriber=rospy.Subscriber('CommandLoc',Pose2D,wrapper.commandMove)
    clockSub=rospy.Subscriber('clock',Int32,wrapper.statePub)

#publications
    statePublisher=rospy.Publisher('RobotState',ReactiveRobotState,queue_size=10)
    alivePub=rospy.Publisher('robotAlive',String,queue_size=1)

    rospy.spin()
  except rospy.ROSInterruptException:
    pass

