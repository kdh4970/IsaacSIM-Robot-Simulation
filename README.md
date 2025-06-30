# IsaacSIM-Robot-Simulation

## Introduction

Navigation and Sensor Data Logging of a Mobile Robot in an Isaac Sim-Based Virtual Environment.  

1. You can control the robot by using keyboard "UP", "DOWN", "LEFT", "RIGHT".  
2. Sensor data of enabled sensors will be published into ROS2 topic automatically.  

Robots : 
 * UGV : turtlebot3_burger  
 * UAV : Not implemented  
 * Humanoid : Unitree G1 (Implementing)

Scenes :
 * Cave : Subterranean cave  
 * Office : Indoor office
 * Rivermark : Middle scale city
 * Gameready City : Small scale city
 * NVIDIA City : Large scale city

Simulation :  
 * Physics rate : 100 Hz  
 * Render rate : 30 Hz  

Sensors and rates :   
 * Camera : 30 Hz RGBD  (depend on Render)
 * IMU : 100 Hz (depend on Physics)
 * Lidar : 10Hz Ouster and Livox  (depend on Render, downsampled)

## Not Implemented Features  
Unitree G1 - TF and Odom
Dual Sim - stereo camera

## Dependencies  

This project was conducted on below environments.  
 * Ubuntu 20.04  
 * IsaacSim 4.5.0  
 * ROS2 Humble  
 * NVIDIA GeForce RTX 3080 Ti x2  

## How to run  

### 0. Before Run.  

Install nvidia graphic driver, IsaacSIM 4.5.0 and ROS2.  

### 1. Clone this Repo and Submodule  
```
git clone --recursive https://github.com/kdh4970/IsaacSIM-Robot-Simulation.git
```

### 2. Edit Script  
Change below things in *run_single_sim.sh* and *run_dual_sim.sh* file.  
 - Check IsaacSIM path. (default : ~/isaacsim)
 - Check ROS2 path.  

If you have a single GPU, remove *export CUDA_VISIBLE_DEVICES* in *run_dual_sim.sh*.  
And also change the GPU number if you need.  

### 3. Get USD Scenes  
1. Unzip scene datas in usd_scenes directory.  
2. You can download *Office* and *Outdoor-Rivermark* scenes from NVIDIA IsaacSIM Assets.  
Direct Download Link : https://download.isaacsim.omniverse.nvidia.com/isaac-sim-assets-1%404.5.0-rc.36%2Brelease.19112.f59b3005.zip
3. Extract */Assets/Isaac/4.5/Isaac/Environments/Office* and */Assets/Isaac/4.5/Isaac/Environments/Outdoor* in zip file.  
4. Move *Office* and *Outdoor* to "IsaacSIM-Robot-Simulation/usd_scenes" directory.

### 4. Lidar Configs  
Move "lidar_configs" directory to "$isaacsim_path/exts/isaacsim.sensors.rtx/data/lidar_configs".  
Replace "extension.toml" to "$isaacsim_path/exts/isaacsim.sensors.rtx/config/extension.toml".  

Tested lidar models.  
 * OS0_REV6_128ch30hz1024res  
 * OS0_REV6_128ch30hz512res  
 * OS0_REV6_64ch30hz1024res  
 * OS0_REV6_32ch30hz1024res  
 * Livox_MID360  

### 5. Start Simulation  

#### Single Simulation : Launch single isaacsim which performs physics and render.

```
cd IsaacSIM-Robot-Simulation
./run_single_sim.sh
```

Before control the robot, click IsaacSIM GUI once.  
Then, you can control the robot by using keyboard. (Up, Down, Right, Left)  


#### (Unstable) Dual Simulation : Launch two isaacsim. (headless_sim and gui_sim)
Before using this, edit *CUDA_VISIBLE_DEVICES* in *run_dual_sim.sh*.  
GUI SIM must use display GPU(id = $DISPLAY).  
```
cd IsaacSIM-Robot-Simulation
./run_dual_sim.sh
```

Before control the robot, click Headless sim console once.  
Then, you can control the robot by using keyboard. (Up, Down, Right, Left)  

### 6. Optional

When simulation performance is good enough, Using cyclone_dds would be better for stability of sensor rates.  

## Demo


https://github.com/user-attachments/assets/1bf2f319-edc1-41eb-a132-e0f983242f7a




