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
 * This is default preset. You can change it by changing defines.PHYSICS_DT and defines.RENDER_DT.  
 * If you set defines.ENABLE_REALTIME_SYNC to False, Simulation will be perfomed with best performance.

Sensors and rates :   
 * Camera : 30 Hz RGBD  (depend on Render)
 * IMU : 100 Hz (depend on Physics)
 * Lidar : 10Hz Ouster and Livox  (depend on Render, downsampled)

## Dependencies  

This project was conducted on below environments.  
 * Ubuntu 20.04  
 * IsaacSim 4.5.0  
 * ROS2 Humble  
 * NVIDIA GeForce RTX 3080 Ti x2  

## How to run  

### 0. Before Run.  

Install IsaacSIM 4.5.0 and ROS2.  

### 1. Clone this Repo and Submodule  
```
git clone --recursive https://github.com/kdh4970/IsaacSIM-Robot-Simulation.git
```

### 2. Edit Script  
Change below things in run_sim.sh file.  
 - If isaacsim path is not "~/isaacsim", change it manually.  
 - Check your ROS path.  

### 3. Unzip Scenes  
Unzip scene datas in usd_scenes directory.  

### 4. Lidar Configs  
Move "lidar_configs" directory to "isaacsim/exts/isaacsim.sensors.rtx/data/lidar_configs".  
Replace "extension.toml" to "isaacsim/exts/isaacsim.sensors.rtx/config/extension.toml".  

Tested lidar models.  
 * OS0_REV6_128ch30hz1024res  
 * OS0_REV6_128ch30hz512res  
 * OS0_REV6_64ch30hz1024res  
 * OS0_REV6_32ch30hz1024res  
 * Livox_MID360  

### 5. Start Simulation  

```
cd IsaacSIM-Robot-Simulation
./run-sim.sh
```

## Demo


https://github.com/user-attachments/assets/1bf2f319-edc1-41eb-a132-e0f983242f7a




