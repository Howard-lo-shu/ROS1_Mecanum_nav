#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move_forward():
    # Initialize the ROS node
    rospy.init_node('move_forward_robot', anonymous=True)
    
    # Create a publisher to the /cmd_vel topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Define a Twist message
    twist = Twist()
    
    # Set the linear velocity (in meters per second)
    linear_speed = 0.5  # Example: 0.2 m/s
    twist.linear.x = linear_speed
    
    # Duration to move 1.0 meter
    distance = 1.0  # Distance to move (1 meter)
    duration = distance / linear_speed  # time = distance / speed
    
    # Publish the message for the required duration
    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now().to_sec()
    
    while rospy.Time.now().to_sec() - start_time < duration:
        pub.publish(twist)
        rate.sleep()
    
    # Stop the robot after moving 1 meter
    twist.linear.x = 0
    pub.publish(twist)
    
    rospy.loginfo("Movement complete")

if __name__ == '__main__':
    try:
        move_forward()
    except rospy.ROSInterruptException:
        pass
