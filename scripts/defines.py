
CAVE_USD_PATH = "/home/do/Desktop/IsaacSIM-Robot-Simulation/usd_scenes/Cave/cave_only.usd"
OFFICE_USD_PATH = "/home/do/Desktop/IsaacSIM-Robot-Simulation/usd_scenes/Collected_office/office.usd"
RIVERMARK_USD_PATH = "/home/do/Desktop/IsaacSIM-Robot-Simulation/usd_scenes/Outdoor/Rivermark/rivermark_turtlebot.usd"
CITY_USD_PATH = "/home/do/Desktop/IsaacSIM-Robot-Simulation/usd_scenes/Full_Gameready_City_Buildings/city_turtlebot_simplified.usd"
NVIDIA_CITY_USD_PATH = "/home/do/Desktop/IsaacSIM-Robot-Simulation/usd_scenes/AECO_CityDemoPack_NVD@10011/Demos/AEC/TowerDemo/CityDemopack/World_CityDemopack_turtlebot.usd"
SENSOR_PACK_URDF_PATH="/home/do/Desktop/IsaacSIM-Robot-Simulation/sensor_pack/sensor_pack.urdf"

PHYSICS_DT = 1/100
RENDER_DT = 1/30

ENABLE_SENSORS = {
    "Camera":True,
    "Camera2":True, # first Camera must be set to True before using this
    "Imu":True,
    "Lidar":True,
    "DebugLidar":False,
    "TfOdom":True
}

ENABLE_REALTIME_SYNC = False  # If false, the simulator operates at maximum performance. Only turn on this option when simulator runs faster than desired Hz.

DISP_FPS        = 1<<0
DISP_AXIS       = 1<<1
DISP_RESOLUTION = 1<<3
DISP_SKELEKETON   = 0<<9
DISP_MESH       = 0<<10
DISP_PROGRESS   = 0<<11
DISP_DEV_MEM    = 1<<13
DISP_HOST_MEM   = 1<<14

LAUNCH_CONFIG = {
    "headless":False,
    "fast_shutdown":True,
    "renderer":"RayTracedLighting",
    "multi_gpu":True,
    "window_width":1920,
    "window_height":1080,
    "display_options": DISP_FPS|DISP_RESOLUTION|DISP_MESH|DISP_DEV_MEM|DISP_HOST_MEM,
    }



## prim path for sensors. 
# Camera and Lidar : include instance path
# IMU path : only parent path
CAMERA_PREFIX_PATH = "/sensors_link/camera_link"
IMU_PREFIX_PATH = "/sensors_link/imu_link"
LIDAR_PREFIX_PATH = "/sensors_link/lidar_link"

ROS_CAMERA_GRAPH_PATH = "/ROS_Camera"
ROS_IMU_GRAPH_PATH = "/ROS_IMU"
ROS_TF_ODOM_GRAPH_PATH = "/ROS_Tf_Odom"

LIDAR_MODEL = "OS0_REV6_128ch10hz1024res"



EXTENSIONS = [
    "omni.graph.window.core", # OmniGraph
    "omni.graph.window.generic",
    "omni.graph.window.action",
    "omni.graph.bundle.action",
    "omni.anim.window.timeline", # Timeline
    "isaacsim.sensors.physx", # Sensor Simulation
    "isaacsim.sensors.physx.ui",
    "isaacsim.sensors.physics.ui",
    "isaacsim.sensors.rtx.ui",
    "isaacsim.robot_setup.assembler", # Robot setup
    "isaacsim.robot_setup.xrdf_editor",
    "isaacsim.robot.wheeled_robots.ui",
    "isaacsim.ros2.bridge", # ROS2
    "isaacsim.ros2.urdf",
    "omni.kit.viewport.bundle", # Viewport
    "omni.kit.window.script_editor", # debug
    "omni.kit.viewport.rtx",
    "omni.kit.profiler.window" # Profiler
]
