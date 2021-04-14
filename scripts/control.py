#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from scout_msgs.msg import ScoutStatus
# from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry,Path

class scout_controller:

	def __init__(self):
		rospy.init_node("scout_controller",anonymous=True)
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
		rospy.Subscriber("/scout_status",ScoutStatus,self.status_callback)
		scout_cmd_vel_msg = Twist()
		# rospy.Subscriber("odom",Odometry,self.odom_callback)
		# rospy.Subscriber("/local_path",Path,self.local_callback)

		rate = rospy.Rate(20)
		while not rospy.is_shutdown():
			# # straight
			scout_cmd_vel_msg.angular.z = 0.0
			scout_cmd_vel_msg.linear.x = 5.0

			# # left 
			# scout_cmd_vel_msg.angular.z = 0.3
			# scout_cmd_vel_msg.linear.x = 1.0

			# # rotation at own point
			# scout_cmd_vel_msg.angular.z = 1.0
			# scout_cmd_vel_msg.linear.x = 0.0

			self.cmd_vel_pub.publish(scout_cmd_vel_msg)
			rate.sleep()

	def status_callback(self,msg):
		print(msg.linear_velocity , msg.angular_velocity)

	# def odom_callback(self,_odom):
		# print("odom")
		# print(_odom)
if __name__ == "__main__":
	test = scout_controller()
