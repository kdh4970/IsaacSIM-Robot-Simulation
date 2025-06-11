## 0. Preset Selection
ENVS = ["Cave", "Office", "Rivermark", "GameReady City", "NVIDIA City"]
ROBOTS = ["turtlebot3_burger", "unitree_g1"]

USD_PATH = {
    ENVS[0]: "/usd_scenes/Cave/cave_only.usd",
    ENVS[1]: "/usd_scenes/Collected_office/office.usd",
    ENVS[2]: "/usd_scenes/Outdoor/Rivermark/rivermark_simple_flattened.usd",
    ENVS[3]: "/usd_scenes/Full_Gameready_City_Buildings/city_turtlebot_simplified.usd",
    ENVS[4]: "/usd_scenes/AECO_CityDemoPack_NVD@10011/Demos/AEC/TowerDemo/CityDemopack/World_CityDemopack_turtlebot.usd"
}
G1_USD_PATH = "/usd_scenes/g1.usd"
SENSOR_PACK_URDF_PATH="/sensor_pack/sensor_pack.urdf"

TURTLEBOT_POSITION = {
    ENVS[0]: [0.0, 0.0, 0.05],
    ENVS[1]: [0.0, 0.0, 0.05],
    ENVS[2]: [0.0, 0.0, 5.9],
    ENVS[3]: [0.0, 0.0, 0.05],
    ENVS[4]: [-13.0, 0.0, 0.0]
}

## 1. App Initialization 
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

## 3. Sensor Configuration
ENABLE_SENSORS = {
    "Camera":True,
    "Camera2":False, # first Camera must be set to True before using this
    "Imu":True,
    "Lidar":True,
    "DebugLidar":False,
    "TfOdom":True
}
CAMERA_PREFIX_PATH = "/sensors_link/camera_link"
IMU_PREFIX_PATH = "/sensors_link/imu_link"
LIDAR_PREFIX_PATH = "/sensors_link/lidar_link"

ROS_CAMERA_GRAPH_PATH = "/ROS_Camera"
ROS_IMU_GRAPH_PATH = "/ROS_IMU"
ROS_TF_ODOM_GRAPH_PATH = "/ROS_Tf_Odom"

# # LIDAR_MODEL = "OS0_REV6_128ch30hz1024res"
# # LIDAR_MODEL = "OS0_REV6_128ch30hz512res"
# # LIDAR_MODEL = "OS0_REV6_64ch30hz1024res"
# LIDAR_MODEL = "OS0_REV6_32ch30hz1024res"
# # LIDAR_MODEL = "Livox_MID360"

LIDAR_MODELS = [
    "OS0_REV6_128ch30hz1024res",
    "OS0_REV6_128ch30hz512res",
    "OS0_REV6_64ch30hz1024res",
    "OS0_REV6_32ch30hz1024res",
    "Livox_MID360"
    ]