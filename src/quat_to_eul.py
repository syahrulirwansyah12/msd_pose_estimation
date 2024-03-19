#!/usr/bin/env python3

# Import Python Libraries
import rospy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import math

# Global Variables
q_raw = []
q = []

# Initialize ROS Node
rospy.init_node('orientation_monitor_node')

# Main Loop Setup
frequency = (1/10) * 1000
rate = rospy.Rate(frequency)

# Create ROS Publishers
imu_raw_pub = rospy.Publisher('orientation/data_raw', Vector3, queue_size=1)
imu_pub = rospy.Publisher('orientation/data', Vector3, queue_size=1)

# Callback function
def imu_raw_callback(msg: Imu):
    global q_raw
    q_raw = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
imu_raw_sub = rospy.Subscriber("imu/data_raw", Imu, imu_raw_callback)

def imu_callback(msg: Imu):
    global q
    q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
imu_sub = rospy.Subscriber("imu/data", Imu, imu_callback)

try:
    while not rospy.is_shutdown():
        (roll_raw, pitch_raw, yaw_raw) = tf.transformations.euler_from_quaternion(q_raw)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(q)
        
        orientation_raw_msg = Vector3(math.degrees(roll_raw),math.degrees(pitch_raw),math.degrees(yaw_raw))
        orientation_msg = Vector3(math.degrees(roll),math.degrees(pitch),math.degrees(yaw))        
        
        imu_raw_pub.publish(orientation_raw_msg)
        imu_pub.publish(orientation_msg)
        rate.sleep()
        
except BaseException as e:
    print(e)
    pass
