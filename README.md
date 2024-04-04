# MSD Pose Estimation
A ROS package developed by ITB de Labo for Nakayama Iron Works's MSD700 product to fuse some localization sensors using [robot_localization](https://docs.ros.org/en/noetic/api/robot_localization/html/index.html) package from ROS.

# Installation
1. Clone this repo to your Catkin workspace's src folder.
2. Install the dependencies. Navigate to `install_requires` folder and run `noetic_dep.sh`
```bash
cd ~/catkin_ws/src/msd_pose_estimation/install_requires
sudo chmod +x noetic_dep.sh
./noetic_dep.sh
```
3. Compile the stack
```bash
cd ~/catkin_ws
catkin_make
```

# Launch File
