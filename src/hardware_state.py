#!/usr/bin/env python3

# Import Python Libraries
import rospy
import tf
from ros_msd700_msgs.msg import HardwareState
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Point, Quaternion, Vector3
from nav_msgs.msg import Odometry
import numpy as np

# Initialize ROS Node
rospy.init_node('raw_sensor_node')

# Get ROS Parameters (loaded from pose_config.yaml)
compute_period = rospy.get_param("/raw_sensor/compute_period", 10)
max_speed_linear = rospy.get_param("/raw_sensor/max_speed_linear", 0.33)
max_speed_angular = rospy.get_param("/raw_sensor/max_speed_angular", 1.75)
wheel_radius = rospy.get_param("/raw_sensor/wheel_radius", 2.75)	  # in cm
wheel_distance = rospy.get_param("/raw_sensor/wheel_distance", 23.0)    # in cm
gear_ratio = rospy.get_param("/raw_sensor/gear_ratio", 1980.0)
use_imu = rospy.get_param("/raw_sensor/use_imu", 1)
az_offset = rospy.get_param("/raw_sensor/az_offset", 0.31)

# Global Variables
right_motor_pulse_delta = 0
left_motor_pulse_delta = 0
vx = 0.0
wz = 0.0
pose_x = 0.0
pose_y = 0.0
theta = 0.0
roll, roll_filter = 0.0, 0.0
pitch, pitch_filter = 0.0, 0.0
yaw, yaw_filter = 0.0, 0.0
acc_x = 0.0
acc_y = 0.0
acc_z = 0.0
gyr_x = 0.0
gyr_y = 0.0
gyr_z = 0.0
mag_x = 0.0
mag_y = 0.0
mag_z = 0.0
acc_filter = [0, 0, 0]
gyr_filter = [0, 0, 0]

# Main Loop Setup
frequency = (1/compute_period) * 1000
rate = rospy.Rate(frequency)

# Create ROS Publishers
odom_pub = rospy.Publisher('wheel/odom', Odometry, queue_size=1)
imu_raw_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=1)
mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=1)
imu_pub = rospy.Publisher('imu/data', Imu, queue_size=1)

# Utility Function
def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def warpAngle(angle):
    if angle >= 2*np.pi:
        return warpAngle(angle - 2*np.pi)
    elif angle < 0:
        return warpAngle(angle + 2*np.pi)
    else:
        return angle
        
def rotm_from_eul(r, p, y):
    Rx = np.array([[1,         0,          0],
                  [0, np.cos(r), -np.sin(r)],
                  [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[ np.cos(p), 0, np.sin(p)],
                  [         0, 1,         0],
                  [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0],
                  [np.sin(y),  np.cos(y), 0],
                  [        0,          0, 1]])
    R = np.matmul(Rz,np.matmul(Ry,Rx))
    return R
        
def hardware_state_callback(msg: HardwareState):
    global right_motor_pulse_delta, left_motor_pulse_delta, roll, pitch, yaw, acc_x, acc_y, acc_z, \
    gyr_x, gyr_y, gyr_z, mag_x, mag_y, mag_z, az_offset
    right_motor_pulse_delta = msg.right_motor_pulse_delta
    left_motor_pulse_delta = msg.left_motor_pulse_delta
    roll, pitch, yaw = np.radians(msg.roll), np.radians(msg.pitch), warpAngle(np.radians(msg.heading)+np.pi/2)
    acc_x, acc_y, acc_z = msg.acc_x, msg.acc_y, msg.acc_z+az_offset
    gyr_x, gyr_y, gyr_z = np.radians(msg.gyr_y), np.radians(msg.gyr_x), np.radians(msg.gyr_z)
    mag_x, mag_y, mag_z = msg.mag_x/1000000.0, msg.mag_y/1000000.0, msg.mag_z/1000000.0
hardware_state_sub = rospy.Subscriber("hardware_state", HardwareState, hardware_state_callback)

def imu_filter_callback(msg: Imu):
    global roll_filter, pitch_filter, yaw_filter, gyr_filter
    q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    (roll_filter, pitch_filter, yaw_filter) = tf.transformations.euler_from_quaternion(q)
    gyr_filter = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
imu_filter_sub = rospy.Subscriber("imu/from_filter", Imu, imu_filter_callback)

try:
    while not rospy.is_shutdown():
        odom_msg = Odometry()
        imu_raw_msg = Imu()
        mag_msg = MagneticField()
        imu_msg = Imu()
    
        delta_right_angle = right_motor_pulse_delta/gear_ratio*2*np.pi
        delta_left_angle = left_motor_pulse_delta/gear_ratio*2*np.pi

        # Calculate robot poses based on wheel odometry
        pose_x = pose_x + wheel_radius/2.0 * (delta_right_angle + delta_left_angle) * np.sin(theta)
        pose_y = pose_y + wheel_radius/2.0 * (delta_right_angle + delta_left_angle) * np.cos(theta)
        if not use_imu:
            theta = theta + (delta_right_angle - delta_left_angle) * wheel_radius/wheel_distance
            theta = warpAngle(theta)
        else:
            theta = theta + gyr_z * compute_period / 1000.0
            theta = warpAngle(theta)
            
        #print(math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
        
        acc_filter_1 = np.array([[acc_x], [acc_y], [acc_z]])
        rotmax_filter = rotm_from_eul(roll_filter, pitch_filter, 0.0)
        
        acc_filter = np.matmul(rotmax_filter, acc_filter_1)
        #print(az_offset)      
        
        #Assign odometry msg
        odom_msg.header.stamp = rospy.Time.now() 
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position = Point(float(pose_x), float(pose_y), 0.0)
        odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0.0, 0.0, theta))         
        odom_pub.publish(odom_msg)
        
        #Assign imu raw msg
        imu_raw_msg.header.stamp = rospy.Time.now()
        imu_raw_msg.header.frame_id = "imu_raw"
        imu_raw_msg.orientation = Quaternion(*tf.transformations.quaternion_from_euler(roll, pitch, yaw))
        imu_raw_msg.angular_velocity = Vector3(gyr_x, gyr_y, gyr_z)
        imu_raw_msg.linear_acceleration = Vector3(acc_x, acc_y, acc_z)
        imu_raw_pub.publish(imu_raw_msg)
        
        #Assign mag msg
        mag_msg.header.stamp = rospy.Time.now()
        mag_msg.header.frame_id = "magnetometer"
        mag_msg.magnetic_field = Vector3(mag_x, mag_y, mag_z)
        mag_pub.publish(mag_msg)
        
        #Assign imu msg    
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu"
        imu_msg.orientation = Quaternion(*tf.transformations.quaternion_from_euler(roll_filter, pitch_filter, yaw))
        imu_msg.angular_velocity = Vector3(gyr_filter[0], gyr_filter[1], gyr_filter[2])
        imu_msg.linear_acceleration = Vector3(acc_filter[1], acc_filter[0], acc_filter[2])
        imu_pub.publish(imu_msg)
        rate.sleep()
        
except BaseException as e:
    print(e)
    pass
