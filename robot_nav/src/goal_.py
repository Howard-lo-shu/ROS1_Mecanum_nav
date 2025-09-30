#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import time


def gat_station(msg):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    print(msg.data)
    if msg.data=='A':
        goal.target_pose.pose.position.x = 13.26
        goal.target_pose.pose.position.y = 0.98
        goal.target_pose.pose.orientation.z = 0.02
        goal.target_pose.pose.orientation.w = 0.99

    if msg.data=='B':
        goal.target_pose.pose.position.x = 0.6
        goal.target_pose.pose.position.y = 0.98
        goal.target_pose.pose.orientation.z = 0.02
        goal.target_pose.pose.orientation.w = 0.99

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        if msg.data=='A':
            pb='c'
            pub.publish(pb)
        if msg.data=='H':
            pb='x'
            pub.publish(pb)
        return client.get_result()   
if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py', anonymous=True)
        sub = rospy.Subscriber("/station", String, gat_station, queue_size=10)
        pub = rospy.Publisher('/Correction', String, queue_size=10)
        #rospy.sleep(5)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

