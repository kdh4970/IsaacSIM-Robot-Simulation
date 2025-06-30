#!/bin/bash

wd=$PWD

# Find ROS and Launch Rviz2
rviz_config="./config/config.rviz"
ros_path1="/opt/ros/humble/local_setup.bash"
ros_path2="$HOME/ros2_humble/install/local_setup.bash"
if  [ -f "$ros_path1" ]; then
	nohup bash -c "source $ros_path1; rviz2 -d $rviz_config" > rviz.log 2>&1 &
	rviz_pid=$!
elif [ -f "$ros_path2" ]; then
	nohup bash -c "source $ros_path2; rviz2 -d $rviz_config" > rviz.log 2>&1 &
	rviz_pid=$!
else
	echo "[Error] Could not find ROS. You should set 'ros_path' manually."
	echo "ros_path1 : $ros_path1"
	echo "ros_path2 : $ros_path2"
	exit 1
fi
echo "Launching Rviz...                 PID : $rviz_pid" 

## IsaacSIm ROS2 Env
isaacsim_path="$HOME/isaacsim"
if [ ! -d "$isaacsim_path" ]; then
	echo "[Error] Could not find IsaacSIM. You should set 'isaacsim_path' manually."
	echo "isaacsim_path : $isaacsim_path"
	kill $rviz_pid 2>/dev/null
	pkill -f "rviz2" 2>/dev/null
	exit 1
fi


export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaacsim_path/exts/isaacsim.ros2.bridge/humble/lib

## IsaacSIM
cd $isaacsim_path
$isaacsim_path/python.sh $wd/scripts/dependecy_check.py
$isaacsim_path/python.sh $wd/scripts/preset_selector.py

## Run headless sim and gui sim on separate GPUs
gnome-terminal -- bash -c "cd $isaacsim_path; export CUDA_VISIBLE_DEVICES=1;$isaacsim_path/python.sh $wd/scripts/headless_sim.py; exec bash" &
headless_pid=$!
echo "Launching IsaacSIM headless...    PID : $headless_pid" 
sleep 20

## GUI SIM : CUDA_VISIBLE_DEVICES must be set to $DISPLAY value.
gnome-terminal -- bash -c "cd $isaacsim_path; export CUDA_VISIBLE_DEVICES=$DISPLAY;$isaacsim_path/python.sh $wd/scripts/gui_sim.py; exec bash" &
gui_pid=$!
echo "Launching IsaacSIM GUI...         PID : $gui_pid"
echo ""
echo "Press 'q' to quit and cleanup all processes..."
while true; do
    read -n 1 -s key
    if [ "$key" = "q" ]; then
        echo ""
        echo "Terminating processes..."

        if [ ! -z "$rviz_pid" ]; then
            kill $rviz_pid 2>/dev/null
        fi
        pkill -f "rviz2" 2>/dev/null
        
        pkill -f "headless_sim.py" 2>/dev/null
        pkill -f "gui_sim.py" 2>/dev/null
        pkill -f "python.sh" 2>/dev/null
        
        if [ ! -z "$headless_pid" ]; then
            kill $headless_pid 2>/dev/null
        fi
        if [ ! -z "$gui_pid" ]; then
            kill $gui_pid 2>/dev/null
        fi
        
        echo "Cleaning up IPC resources..."
        ipcrm -M 1000 2>/dev/null
        ipcrm -S 1001 2>/dev/null
        
        echo "Cleanup completed."
        break
    fi
done