#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def rotate():
    # Initialize the ROS node
    rospy.init_node('rotate_robot', anonymous=True)
    
    # Create a publisher to the /cmd_vel topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Define a Twist message
    twist = Twist()
    
    # Set the angular velocity (in radians per second)
    angular_speed = 0.5 # Example: 0.5 rad/s
    twist.angular.z = angular_speed
    
    # Duration to rotate 90 degrees (π/2 radians)
    duration = (1.5708) / angular_speed  # 90 degrees in radians
    
    # Publish the message for the required duration
    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now().to_sec()
    
    while rospy.Time.now().to_sec() - start_time < duration:
        pub.publish(twist)
        rate.sleep()
    
    # Stop the robot after rotation
    twist.angular.z = 0
    pub.publish(twist)
    
    rospy.loginfo("Rotation complete")

if __name__ == '__main__':
    try:
        rotate()
    except rospy.ROSInterruptException:
        pass
