#!/usr/bin/env python

from __future__ import print_function
from robot_localization.srv import SetPose
from std_srvs.srv import Empty
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def pose_trigger(req):
	
	#print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
	#return AddTwoIntsResponse(req.a + req.b)
	#print(req)
	init_pose = PoseWithCovarianceStamped()
	init_pose.header.frame_id = 'map'
	service_proxy = rospy.ServiceProxy('/set_pose',SetPose)
	resp = service_proxy(init_pose)
	return []
	
def reset_pose_custom():
  	rospy.init_node('Rviz_resetter')
  	s = rospy.Service('/reset_caller',Empty, pose_trigger)
  	#print("Ready to add two ints.")
  	rospy.spin()

if __name__ == "__main__":
	reset_pose_custom()
