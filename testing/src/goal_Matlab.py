#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

def gat_station(msg):
    print(msg.data)
    client = actionlib.SimpleActionClient('/move_base',MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    
    if msg.data=='A':
        goal.target_pose.pose.position.x = 10.53
        goal.target_pose.pose.position.y = 1.0
        goal.target_pose.pose.orientation.z = 0.004
        goal.target_pose.pose.orientation.w = 0.99
    if msg.data=='B':
        goal.target_pose.pose.position.x = 0.5
        goal.target_pose.pose.position.y = 1.0
        goal.target_pose.pose.orientation.z = 0.004
        goal.target_pose.pose.orientation.w = 0.99

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    if msg.data!='B':
        pb='c'
        pub.publish(pb)
        return client.get_result()   
if __name__ == '__main__':
    try:
        rospy.init_node('robob_hololens_goal_sub', anonymous=True)
        sub = rospy.Subscriber("/station", String, gat_station, queue_size=10)
        pub = rospy.Publisher('/Correction', String, queue_size=10)
        pub_Correction_finish = rospy.Publisher('/corr_f', String, queue_size=1)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

