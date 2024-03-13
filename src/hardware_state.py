#!/usr/bin/env python3

# Import Python Libraries
import rospy
import tf
from ros_msd700_msgs.msg import HardwareCommand, HardwareState
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from nav_msgs.msg import Odometry
import math

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

# Global Variables
right_motor_pulse_delta = 0
left_motor_pulse_delta = 0
vx = 0.0
wz = 0.0
pose_x = 0.0
pose_y = 0.0
theta = 0.0
roll = 0.0
pitch = 0.0
yaw = 0.0
acc_x = 0.0
acc_y = 0.0
acc_z = 0.0
gyr_x = 0.0
gyr_y = 0.0
gyr_z = 0.0
mag_x = 0.0
mag_y = 0.0
mag_z = 0.0

# Main Loop Setup
frequency = (1/compute_period) * 1000
rate = rospy.Rate(frequency)

# Create ROS Publishers
odom_pub = rospy.Publisher('wheel/odom', Odometry, queue_size=1)
imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=1)
mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=1)

# Utility Function
def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def warpAngle(angle):
    if angle >= 6.28318531:
        return warpAngle(angle - 6.28318531)
    elif angle < 0:
        return warpAngle(angle + 6.28318531)
    else:
        return angle
        
def hardware_state_callback(msg: HardwareState):
    global right_motor_pulse_delta, left_motor_pulse_delta, roll, pitch, yaw, acc_x, acc_y, acc_z, \
    gyr_x, gyr_y, gyr_z, mag_x, mag_y, mag_z
    right_motor_pulse_delta = msg.right_motor_pulse_delta
    left_motor_pulse_delta = msg.left_motor_pulse_delta
    roll, pitch, yaw = msg.roll, msg.pitch, msg.heading
    acc_x, acc_y, acc_z = msg.acc_x, msg.acc_y, msg.acc_z
    gyr_x, gyr_y, gyr_z = msg.gyr_x, msg.gyr_y, msg.gyr_z
hardware_state_sub = rospy.Subscriber("hardware_state", HardwareState, hardware_state_callback)

try:
    while not rospy.is_shutdown():
        odom_msg = Odometry()
        imu_msg = Imu()
        mag_msg = MagneticField()
    
        delta_right_angle = right_motor_pulse_delta/gear_ratio*6.28318531
        delta_left_angle = left_motor_pulse_delta/gear_ratio*6.28318531

        # Calculate robot poses based on wheel odometry
        pose_x = pose_x + wheel_radius/2.0 * (delta_right_angle + delta_left_angle) * math.sin(theta);
        pose_y = pose_y + wheel_radius/2.0 * (delta_right_angle + delta_left_angle) * math.cos(theta);
        if not use_imu:
            theta = theta + (delta_right_angle - delta_left_angle) * wheel_radius/wheel_distance;
            theta = warpAngle(theta)
        else:
            theta = theta + gyr_z * compute_period / 1000.0
            theta = warpAngle(theta)
            
        #Assign odometry msg
        odom_msg.header.stamp = rospy.Time.now() 
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position = Point(float(pose_x), float(pose_y), 0.0)
        odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0.0, 0.0, theta))         
        odom_pub.publish(odom_msg)
        
        #Assign imu msg
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu"
        imu_msg.orientation = Quaternion(*tf.transformations.quaternion_from_euler(roll, pitch, yaw))
        imu_msg.angular_velocity = Vector3(gyr_x, gyr_y, gyr_z)
        imu_msg.linear_acceleration = Vector3(acc_x, acc_y, acc_z)
        imu_pub.publish(imu_msg)
        
        #Assign mag msg
        mag_msg.header.stamp = rospy.Time.now()
        mag_msg.header.frame_id = "magnetometer"
        mag_msg.magnetic_field = Vector3(mag_x, mag_y, mag_z)
        mag_pub.publish(mag_msg)
        rate.sleep()
        
except BaseException as e:
    print(e)
    pass
