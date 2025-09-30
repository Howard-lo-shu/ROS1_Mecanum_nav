#!/usr/bin/python3
import math
import rospy
import time
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from std_msgs.msg import String ,Int64 ,Float32 ,Bool


class agv_position():
    print("Begin")
    def __init__(self):
        self.pose_pub=rospy.Publisher("/base_link_pose",String,queue_size=100)
        self.init_pub=rospy.Publisher("/init_5G",String,queue_size=1)
        rate = rospy.Rate(10.0)
        listener = tf.TransformListener()
        self.point_data=PoseWithCovarianceStamped()

        self.init_string=String()
        self.point_string=String()
        
        while not rospy.is_shutdown():

            try:
                (point_trans,point_rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                self.point_data.pose.pose.position.x=point_trans[0]
                self.point_data.pose.pose.position.y=point_trans[1]
                [point_raw,point_pitch,point_yaw]=euler_from_quaternion(point_rot)
                point_yaw=point_yaw/3.14*180 
                self.point_string=str(point_trans[0])+","+str(point_trans[1])+","+str(point_yaw) 
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                #print("error")
                time.sleep(0.01)
                continue
 
            self.init_pub.publish(self.init_string)
            self.pose_pub.publish(self.point_string)

if __name__ == '__main__':
    rospy.init_node('AMR_position_pub')

    try:
        agv_position()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMR node terminated.")
