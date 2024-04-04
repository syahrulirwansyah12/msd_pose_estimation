MSD Pose Estimation
====================
A robot localization stack developed by ITB de Labo for Nakayama Iron Works's MSD700 product to fuse some localization sensors using [robot_localization](https://docs.ros.org/en/noetic/api/robot_localization/html/index.html) package from ROS.

Installation
------------
1. Clone this repo to your Catkin workspace's src folder.
2. Install the dependencies. Navigate to `install_requires` folder and run `noetic_dep.sh`.
```bash
cd ~/catkin_ws/src/msd_pose_estimation/install_requires
sudo chmod +x noetic_dep.sh
./noetic_dep.sh
```
3. Compile the stack.
```bash
cd ~/catkin_ws
catkin_make
```

Overview
--------

### Launch file
This stack contains several launch files as follows:
* `imu_filter.launch` : this launch file corrects the IMU measurement by accounting for the earth's magnetic field and the robot's tilt [[1]]. This uses "imu_filter_madgwick" node from "imu_tools" package and "hardware_state.py" node.

[1]: http://wiki.ros.org/imu_tools

* `robot_localization.launch` : this launches "imu_filter.launch" and "ekf_localization_node" from "robot_localization" package [[2]].

[2]: https://docs.ros.org/en/noetic/api/robot_localization/html/index.html

### Nodes
This stack contains several custom nodes as follows:
* `hardware_state.py` : this node subscribes to the "hardware_state" topic that consists of the `ros_msd700_msgs::HardwareState` type of messages as its main data to process. Please refer to [this link](https://github.com/itbdelaboprogramming/ros_msd700_msgs) for the message details. In the end, this node will publish the "wheel/odom" (`nav_msgs::Odometry`) topic and the "imu/data" (`sensor_msgs::Imu`) topic that will be supplied to the robot_localization node.

* `quat_to_eul.py` : this node can be set to subscribe to the arbitrary topics that contain orientation data in quaternion such as `nav_msgs::Odometry` or `sensor_msgs::Imu`, then this node will convert those quaternions orientation into Euler angles.

### Visualization
This stack provides an R-viz config to visualize both "wheel/odom" and "odometry/filtered" topics by simply run:
```bash

```
