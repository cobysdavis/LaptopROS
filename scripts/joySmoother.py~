
#!/usr/bin/env python

import math
import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class joyData :
	def __init__(self) :
		self.data=Joy()
		for i in range(0,17) :
			self.data.axes.append(0)
		for i in range(0,13) :
			self.data.buttons.append(0)


	def callback(self,newRawData) :
		newCleanData=self.cleanUp(newRawData)
		flag=True
		for i in range(0,17) :
			if not newCleanData.axes[i]==self.data.axes[i] :
				flag=False
		for i in range(0,13) :
			if not newCleanData.buttons[i]==self.data.buttons[i] :
				flag=False
		if flag==True :
			pass
		else :
			self.data=newCleanData
			pub.publish(newCleanData)
	
	def cleanUp(self,dirtyData) :
		data=Joy()
		data.axes=[]
		data.buttons=[]
		for i in range(0,17) :
			data.axes.append(dirtyData.axes[i])
		for i in range(0,13) :
			data.buttons.append(dirtyData.buttons[i])
		data.axes[3]=0
		data.axes[4]=0
		data.axes[11]=0
		data.axes[12]=0
		data.axes[13]=0
		data.axes[14]=0
		data.axes[15]=0
		data.axes[16]=0
		return data



if __name__ == '__main__':
	try: 
		joy=joyData()
		rospy.init_node('joySmoother')
		rospy.Subscriber('joy', Joy, joy.callback)
		pub = rospy.Publisher('smoothData', Joy,queue_size=3)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

