#!/bin/zsh
# Change all 'zsh' to 'bash' if you use bash shell.

# Find ROS and Launch Rviz2
rviz_config="./rviz_config/config.rviz"
ros_path1="/opt/ros/humble/local_setup.zsh"
ros_path2="/home/do/ros2_humble/install/local_setup.zsh"
if  [ -f "$ros_path1" ]; then
	nohup zsh -c "source $ros_path1; rviz2 -d $rviz_config" > rviz.log 2>&1 &
elif [ -f "$ros_path2" ]; then
	nohup zsh -c "source $ros_path2; rviz2 -d $rviz_config" > rviz.log 2>&1 &
else
	echo "[Error] Could not find ROS. You should set 'ros_path' manually."
	echo "ros_path1 : $ros_path1"
	echo "ros_path2 : $ros_path2"
	exit 1
fi


## IsaacSIm ROS2 Env
isaacsim_path="/home/do/isaacsim"
if [ ! -d "$isaacsim_path" ]; then
	echo "[Error] Could not find IsaacSIM. You should set 'isaacsim_path' manually."
	echo "isaacsim_path : $isaacsim_path"
	exit 1
fi


export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaacsim_path/exts/isaacsim.ros2.bridge/humble/lib


cd ~/IsaacLab
./isaaclab.sh -p /home/do/Desktop/IsaacSIM-Robot-Simulation/scripts/robot_simulation.py
