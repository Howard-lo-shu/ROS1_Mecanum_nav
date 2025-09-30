#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from std_msgs.msg import String
import numpy as np
import time

plt.ion()
t=[]
minnn = []
def scan_callback(msg):
    distances = np.array(msg.ranges)

    valid_distances = distances[np.isfinite(distances)]
    if len(valid_distances) > 0:
        min_distance = np.min(valid_distances)
        minnn.append(min_distance)
        t.append(time.clock_gettime(3))
        distances_pub.publish(f"{min_distance:.2f}")
        rospy.loginfo(f"最近的障礙物距離: {min_distance:.2f} 米")
        #plt.clf()
        #plt.plot(distances, label="Lidar Distances")
        #plt.axhline(y=min_distance, color='r', linestyle='--', label=f"Min Distance: {min_distance:.2f}m")
        plt.plot(t,minnn)
        plt.xlabel('time')
        plt.ylabel('error')
        plt.title('Error')
        plt.grid(True)
        plt.show()
        
        
        plt.pause(0.1)
    else:
        rospy.logwarn("未檢測到有效的障礙物距離數據")


if __name__ == "__main__":
    rospy.init_node("lidar_plot_node")
    rospy.Subscriber("//scan_pointcloud", LaserScan, scan_callback)
    distances_pub = rospy.Publisher('/valid_distances', String, queue_size=10)
    rospy.spin()
