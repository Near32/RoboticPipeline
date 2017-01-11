#!/usr/bin/env python

import rospy
import roslib.message
from sensor_msgs.msg import PointCloud2, PointField

#class PointCloudClass:
pointcloud = []
 
def callback(data):
		pointcloud = data.data
		print(pointcloud[1])
		
def listener():
	rospy.init_node('listener',anonymous=True)
	
	rospy.Subscriber("/RP_robot_model_0/depth/points", PointCloud2, callback)#PointCloudClass.callback)
	
	rospy.spin()
		
if __name__ == '__main__' :
	#pc = PointCloudClass()
	#pc.listener()
	listener()
