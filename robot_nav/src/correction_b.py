#!/usr/bin/env python3

import rospy
import numpy as np
import tf
from enum import Enum
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib.pyplot as plt
from std_msgs.msg import String
import math
import time

MARKER_ID_DETECTION = 4

def newOdom(msg):
    global ar_tak,x_ar,y_ar,z_ar,w_ar
    if(not msg.markers):
        ar_tak = 'non'
    else:
        ar_tak = ' '
        x_ar = msg.markers[0].pose.pose.position.x
        y_ar = msg.markers[0].pose.pose.position.y
        z_ar = msg.markers[0].pose.pose.position.z
        w_ar = msg.markers[0].pose.pose.orientation.w
        

def cbGetRobotOdom(robot_odom_msg):
    global robot_2d_pose_x,robot_2d_pose_y,robot_2d_theta,is_odom_received,previous_robot_2d_theta,total_robot_2d_theta
    if is_odom_received == False:
        is_odom_received = True 
    
    pos_x, pos_y, theta = fnGet2DRobotPose(robot_odom_msg)

    robot_2d_pose_x = pos_x
    robot_2d_pose_y = pos_y
    robot_2d_theta = theta

    if (robot_2d_theta - previous_robot_2d_theta) > 5.:
        d_theta = (robot_2d_theta - previous_robot_2d_theta) - 2 * math.pi
    elif (robot_2d_theta - previous_robot_2d_theta) < -5.:
        d_theta = (robot_2d_theta - previous_robot_2d_theta) + 2 * math.pi
    else:
        d_theta = (robot_2d_theta - previous_robot_2d_theta)

    total_robot_2d_theta = total_robot_2d_theta + d_theta
    previous_robot_2d_theta = robot_2d_theta

    robot_2d_theta = total_robot_2d_theta

def cbGetMarkerOdom(markers_odom_msg):
    global marker_2d_pose_x,marker_2d_pose_y,marker_2d_theta,is_marker_pose_received,y_ar
    for marker_odom_msg in markers_odom_msg.markers:
        if marker_odom_msg.id == MARKER_ID_DETECTION:
            if is_marker_pose_received == False:
                is_marker_pose_received = True

            pos_x, pos_y, theta = fnGet2DMarkerPose(marker_odom_msg)

            marker_2d_pose_x = pos_x
            marker_2d_pose_y = pos_y
            marker_2d_theta = theta - math.pi

        


def fnParking():
    global is_sequence_finished,current_parking_sequence,is_do_correction
    if current_parking_sequence == ParkingSequence.searching_parking_lot.value:
        is_sequence_finished = fnSeqSearchingGoal()
        
        if is_sequence_finished == True: #find marker
            print ("Finished 1")
            current_parking_sequence = ParkingSequence.changing_direction.value
            is_sequence_finished = False

    elif current_parking_sequence == ParkingSequence.changing_direction.value: #turn car
        print ("changing_direction")
        is_sequence_finished = fnSeqChangingDirection()
        
        if is_sequence_finished == True:
            print ("Finished 2")
            current_parking_sequence = ParkingSequence.parking.value
            is_sequence_finished = False
    
    elif current_parking_sequence == ParkingSequence.parking.value:#PID
        is_sequence_finished = fnSeqParking()
        
        if is_sequence_finished == True:
            print ("Finished 3")
            current_parking_sequence = ParkingSequence.moving_nearby_parking_lot.value
            is_sequence_finished = False

    elif current_parking_sequence == ParkingSequence.moving_nearby_parking_lot.value:#turn car
        is_sequence_finished = tt()
            
        if is_sequence_finished == True:
            print ("Finished 4")
            
            current_parking_sequence = ParkingSequence.moving_nearby_parking_oo.value
            is_sequence_finished = False

    elif current_parking_sequence == ParkingSequence.moving_nearby_parking_oo.value:
            is_sequence_finished = ft()
            
            if is_sequence_finished == True:
                print ("Finished 5")
                current_parking_sequence = ParkingSequence.stop.value
                is_sequence_finished = False

    elif current_parking_sequence == ParkingSequence.tur.value:
            #is_sequence_finished = fnSeqParking2()
            
            if is_sequence_finished == True:
                print ("Finished 6")
                current_parking_sequence = ParkingSequence.stop.value
                is_sequence_finished = False
    
    elif current_parking_sequence == ParkingSequence.stop.value:
            fnStop()
            print ("Finished 7")
            current_parking_sequence = ParkingSequence.finished.value
            is_do_correction = False

def ft():
    global y_ar,w_ar,ar_tak
    if y_ar > 0.00:
        desired_angle_turn = 0.02
        fnrt(desired_angle_turn)
        return False
    if y_ar < -0.003:
        desired_angle_turn = -0.02
        fnrt(desired_angle_turn)
        return False
    else:
        fnStop()
        return True

def tt():
    global y_ar,w_ar,ar_tak
    if w_ar > 0.50:
        desired_angle_turn = 0.5
        fnTurn(desired_angle_turn)
        return False
    if w_ar < 0.49 or ar_tak =='non':
        desired_angle_turn = -0.5
        fnTurn(desired_angle_turn)
        return False
    else:
        fnStop()
        return True
    
def fnSeqParking2():
    global desdesired_angle_turn,marker_2d_pose_y,marker_2d_pose_x,is_sequence_finished
    is_sequence_finished = True
    line_y_right_left = math.atan2(marker_2d_pose_y - 0, marker_2d_pose_x - 0)
    fnTrackMarker(line_y_right_left)
    
    print (marker_2d_pose_x)
    if abs(marker_2d_pose_x) < 0.85:
        fnStop()
        return True
    else:
        return False
    
def fnSeqParking():
    global desdesired_angle_turn,marker_2d_pose_y,marker_2d_pose_x,is_sequence_finished
    is_sequence_finished = True
    line_y_right_left = math.atan2(marker_2d_pose_y + 0, marker_2d_pose_x + 0)
    print(line_y_right_left)
    fnTrackMarker(line_y_right_left)
    
    #print (marker_2d_pose_x)
    if abs(marker_2d_pose_x) < 0.92:
        fnStop()
        return True
    else:
        return False

def fnSeqChangingDirection():
        global desired_angle_turn,marker_2d_pose_y,marker_2d_pose_x
        desired_angle_turn = 1. *  math.atan2(marker_2d_pose_y - 0, marker_2d_pose_x - 0)
        fnTurn(desired_angle_turn)   
        #print(desired_angle_turn)    
        if abs(desired_angle_turn) > 3.10:
            fnStop()
            return True
        else:
            return False

def fnSeqSearchingGoal():
    global is_marker_pose_received
    if is_marker_pose_received is False:
        desired_angle_turn = -5.0
        fnTurn(desired_angle_turn)
    else:
        fnStop()
        return True

def fnrt(theta):
    twist = Twist()
    linear_y = theta
    twist.linear.x = 0
    twist.linear.y = linear_y
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub_cmd_vel.publish(twist)

def fnStop():
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub_cmd_vel.publish(twist)

def fnback():
    twist = Twist()
    twist.linear.x = -0.1
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub_cmd_vel.publish(twist)

def fnTurn(theta):
    global carerror,t
    Kp = 0.1

    angular_z = Kp * theta * 0.1
    #angular_z = Kp

    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = -angular_z
    pub_cmd_vel.publish(twist)

def fnGoStraight():
    twist = Twist()
    twist.linear.x = 0.1
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub_cmd_vel.publish(twist)

def fnTrackMarker(theta):
    global carerror,t
    Kp = 0.2

    linear_y = Kp * theta * 0.1

    twist = Twist()
    twist.linear.x = -0.1
    twist.linear.y = linear_y
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub_cmd_vel.publish(twist) 

def fnShutDown():
    rospy.loginfo("Shutting down. cmd_vel will be 0")

    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub_cmd_vel.publish(twist) 

def fnGet2DRobotPose(robot_odom_msg):
    quaternion = (robot_odom_msg.pose.pose.orientation.x, robot_odom_msg.pose.pose.orientation.y, robot_odom_msg.pose.pose.orientation.z, robot_odom_msg.pose.pose.orientation.w)
    theta = tf.transformations.euler_from_quaternion(quaternion)[2]

    if theta < 0:
        theta = theta + np.pi * 2
    if theta > np.pi * 2:
        theta = theta - np.pi * 2

    pos_x = robot_odom_msg.pose.pose.position.x
    pos_y = robot_odom_msg.pose.pose.position.y

    return pos_x, pos_y, theta




def fnGet2DMarkerPose(marker_odom_msg):
    quaternion = (marker_odom_msg.pose.pose.orientation.x, marker_odom_msg.pose.pose.orientation.y, marker_odom_msg.pose.pose.orientation.z, marker_odom_msg.pose.pose.orientation.w)
    theta = tf.transformations.euler_from_quaternion(quaternion)[2]

    theta = theta + np.pi / 2.
    # rospy.loginfo("theta : %f", theta)

    if theta < 0:
        theta = theta + np.pi * 2
    if theta > np.pi * 2:
        theta = theta - np.pi * 2

    pos_x = marker_odom_msg.pose.pose.position.x
    pos_y = marker_odom_msg.pose.pose.position.y

    return pos_x, pos_y, theta

def fnCalcDistPoints(x1, x2, y1, y2):
    return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

def ccc(msg):
    global is_do_correction
    CO = str(msg.data)
    print(CO)
    if CO == 'B':
        is_do_correction = True

if __name__ == '__main__':
    rospy.init_node('automatic_parking_vision')
    sub_odom_robot = rospy.Subscriber('/odom', Odometry, cbGetRobotOdom, queue_size = 1)
    sub_info_marker = rospy.Subscriber('/camera_back_ar_pose_marker', AlvarMarkers, cbGetMarkerOdom, queue_size = 1)
    sub = rospy.Subscriber("/camera_back_ar_pose_marker", AlvarMarkers, newOdom, queue_size = 1)
    sub_corr = rospy.Subscriber("/camera_number", String, ccc, queue_size = 1)

    pub_cmd_vel = rospy.Publisher('/nav_vel', Twist, queue_size=1)

    ParkingSequence = Enum('ParkingSequence', 'searching_parking_lot changing_direction moving_nearby_parking_lot parking stop finished moving_nearby_parking_oo tur ss')
    NearbySequence = Enum('NearbySequence', 'initial_turn go_straight turn_right parking')
    current_nearby_sequence = NearbySequence.initial_turn.value
    current_parking_sequence = ParkingSequence.searching_parking_lot.value
    t=[]
    carerror=[]
    robot_2d_pose_x = .0
    robot_2d_pose_y = .0
    robot_2d_theta = .0
    marker_2d_pose_x = .0
    marker_2d_pose_y = .0
    marker_2d_theta = .0

    previous_robot_2d_theta = .0
    total_robot_2d_theta = .0
    is_triggered = False

    is_sequence_finished = False

    is_odom_received = False
    is_marker_pose_received = False

    is_do_correction = False

    x_ar = 0.0 
    y_ar = 0.0 
    z_ar = 0.0
    w_ar = 0.0
    ar_tak = ' ' 
    CO = ' '

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if is_do_correction == True and is_odom_received == True:
            fnParking()
                
            loop_rate.sleep()
        else:
            current_nearby_sequence = NearbySequence.initial_turn.value
            current_parking_sequence = ParkingSequence.searching_parking_lot.value


    rospy.on_shutdown(fnShutDown)
