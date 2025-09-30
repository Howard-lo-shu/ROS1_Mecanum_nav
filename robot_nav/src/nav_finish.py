#!/usr/bin/env python3
import rospy
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalStatus
import math
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

def get_current_yaw():
    try:
        tf_listener = tf.TransformListener()
        tf_listener.waitForTransform("/map", "base_link", rospy.Time(0), rospy.Duration(3.0))
        (trans, rot) = tf_listener.lookupTransform("/map", "base_link", rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
        print(math.degrees(euler[2]))
        return euler[2]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("無法獲取當前朝向")
        return None
    
def rotate_to_target():
    global cmd_vel_pub,TARGET_YAW,cc,pub_robot_stat

    rate = rospy.Rate(10)
    twist = Twist()

    while not rospy.is_shutdown():
        current_yaw = get_current_yaw()
        if current_yaw is None:
            continue

        yaw_error = TARGET_YAW - current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        if abs(yaw_error) > 0.05:
            twist.angular.z = 0.2 * yaw_error
            cmd_vel_pub.publish(twist)
            pub_robot_stat.publish('N')
        else:
            cc = True
            
            break

        rate.sleep()

    twist.angular.z = 0
    cmd_vel_pub.publish(twist)
    pub_robot_stat.publish('F')
    rospy.loginfo("機器人已旋轉到 {:.2f} 度".format(math.degrees(TARGET_YAW)))

def goal_callback(msg):
    global cmd_vel_pub,TARGET_YAW
    _, _, yaw = tf.transformations.euler_from_quaternion([
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w
    ])
    rospy.loginfo(f"Yaw: {yaw}")
    TARGET_YAW = yaw

def status_callback(msg):
    global cmd_vel_pub,TARGET_YAW,cc,pub_stop,pub_robot_stat
    if msg.status_list:
        status = msg.status_list[-1].status
        if status == GoalStatus.ACTIVE:
            #rospy.loginfo("正在導航中...")
            cc = False
            pub_robot_stat.publish('N')
        elif status == GoalStatus.SUCCEEDED:
            #rospy.loginfo("導航完成！")
            if cc == False:
                rotate_to_target()
        elif status == GoalStatus.ABORTED:
            rospy.loginfo("導航失敗！")
            pb='ok'
            pub_stop.publish(pb)
        else:
            rospy.loginfo("導航狀態：%d", status)
    else:
        rospy.loginfo("無導航目標或導航尚未開始")

def monitor_navigation():
    global cmd_vel_pub,TARGET_YAW,cc,pub_robot_stat,pub_stop
    rospy.init_node('navigation_monitor')
    rospy.Subscriber("/move_base/status", GoalStatusArray, status_callback)
    cmd_vel_pub = rospy.Publisher('/nav_vel', Twist, queue_size=10)
    pub_robot_stat = rospy.Publisher("/robot_stop", String, queue_size = 1)
    pub_stop = rospy.Publisher("/finish_nav", String, queue_size = 1)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)
    cc = False

    

    rate = rospy.Rate(1) 
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()

if __name__ == "__main__":
    try:
        TARGET_YAW = 0.0

        monitor_navigation()

    except rospy.ROSInterruptException:
        pass
