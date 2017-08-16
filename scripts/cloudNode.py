#!/usr/bin/env python


import rospy
from sensor_msgs.msg import PointCloud2


def callback(data):
	print data
	




if __name__ == '__main__':
	try: 
		rospy.init_node('cloudNode')
		rospy.Subscriber('mcptam/map_points', PointCloud2, callback)
		pub = rospy.Publisher('filteredCloud', PointCloud2, queue_size=3)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

