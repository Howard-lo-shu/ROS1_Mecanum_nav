#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped


accel_data = None
gyro_data = None

def accel_callback(msg):
    global accel_data
    accel_data = msg

def gyro_callback(msg):
    global gyro_data
    gyro_data = msg

def publish_imu():
    global accel_data, gyro_data

    rospy.init_node('imu_converter')

    rospy.Subscriber('/camera_front/accel/sample', Imu, accel_callback)
    rospy.Subscriber('/camera_front/gyro/sample', Imu, gyro_callback)

    imu_pub = rospy.Publisher('/camera/imu', Imu, queue_size=10)


    rate = rospy.Rate(50)  # 50 Hz

    while not rospy.is_shutdown():
        if accel_data and gyro_data:
            imu_msg = Imu()

            imu_msg.linear_acceleration.x = accel_data.linear_acceleration.x
            imu_msg.linear_acceleration.y = accel_data.linear_acceleration.y
            imu_msg.linear_acceleration.z = accel_data.linear_acceleration.z

            imu_msg.angular_velocity.x = gyro_data.angular_velocity.x
            imu_msg.angular_velocity.y = gyro_data.angular_velocity.y
            imu_msg.angular_velocity.z = gyro_data.angular_velocity.z

            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "base_link"

            imu_pub.publish(imu_msg)
            #print(imu_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_imu()
    except rospy.ROSInterruptException:
        pass
