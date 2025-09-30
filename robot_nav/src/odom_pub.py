#!/usr/bin/env python3
import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


def vel_callback(msg):
	global vx,vy,vth
	vx = msg.linear.x
	vy = msg.linear.y
	vth = msg.angular.z
	

       

if __name__ =='__main__':
	rospy.init_node('odometry_publisher')
	sub = rospy.Subscriber('/nav_vel', Twist, vel_callback)
	robot_pub = rospy.Publisher("/robot_cmd", String, queue_size=10)
	x = 0.0
	y = 0.0
	th = 0.0
	vx =0.0
	vy = 0.0
	vth = 0.0
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		X=str(vx)
		Y = str(vy)
		Th = str(vth)
		pb = X+','+Y+','+Th
		robot_pub.publish(pb)


