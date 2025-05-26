# IsaacSIM-Robot-Simulation

## Introduction

Navigation and Sensor Data Logging of a Mobile Robot in an Isaac Sim-Based Virtual Environment.  

Robot : 
 * UGV : turtlebot3_burger  
 * UAV : Not implemented  
 * Humanoid : Unitree G1 (Implementing)

Sensors and rate :   
 * Camera : 30 Hz RGBD  
 * IMU : 100 Hz  
 * Lidar : 10Hz Ouster and Livox  

Scenes :
 * Cave : Subterranean cave  
 * Office : Indoor office
 * Rivermark : Middle scale city
 * Gameready City : Small scale city
 * NVIDIA City : Large scale city

## Dependencies  

This project was conducted on below environments.  
 * Ubuntu 20.04  
 * IsaacSim 4.5.0  
 * ROS2 Humble  

## How to run  

### 0. Before Run.  

Install IsaacSIM 4.5.0 and ROS2.  

### 1. Clone this Repo  
```
git clone https://github.com/kdh4970/IsaacSIM-Robot-Simulation.git
```

### 2. Edit Script  
Change below things in run_sim.sh file.  
 - If isaacsim path is not "~/isaacsim", change it manually.  
 - Check your ROS path.  
 - If You dont use zsh, change "setup.zsh" to your shell.  

### 3. Unzip Scenes  
Unzip scene datas in usd_scenes directory.  

### 4. Lidar Configs  
Move "lidar_configs" directory to "isaacsim/exts/isaacsim.sensors.rtx/data/lidar_configs"
Replace "extension.toml" to "isaacsim/exts/isaacsim.sensors.rtx/config/extension.toml"

### 5. Start Simulation  

```
cd IsaacSIM-Robot-Simulation
./run-sim.sh
```

## Demo


https://github.com/user-attachments/assets/1bf2f319-edc1-41eb-a132-e0f983242f7a




