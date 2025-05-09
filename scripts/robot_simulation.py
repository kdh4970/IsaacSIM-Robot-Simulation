#############################################
###                                       ###
###           0. Preset Selection         ###
###                                       ###
#############################################
import defines
import os
from time import sleep,perf_counter
from utils import PresetSelector, IntervalChecker
from PyQt5.QtWidgets import QApplication
import sys
import numpy as np


sleep(2) # waiting for RVIZ loading first
app = QApplication(sys.argv)
lstPreset = [
    "Cave + turtlebot3_burger",
    "Office + turtlebot3_burger",
    "Rivermark + turtlebot3_burger",
    "City + turtlebot3_burger",
    "NVIDIA_City + turtlebot3_burger",
    "Custom + Custom"]
preset_selector = PresetSelector(lstPreset)
env,robot = preset_selector.env, preset_selector.robot

if env==None or robot==None:
    os.system("pgrep -f rviz | xargs -r kill -15")
    exit()
elif env=="Cave" and robot =="turtlebot3_burger":
    usdPath=defines.CAVE_USD_PATH
    robotPrimPath = "/fuel/turtlebot3_burger"
    robotPosition = np.array([0, 0.0, 0.05])
    sensorPackPrimPath = robotPrimPath + "/base_footprint/base_link/sensor_pack"
elif env=="Office" and robot =="turtlebot3_burger":
    usdPath=defines.OFFICE_USD_PATH
    robotPrimPath = "/World/turtlebot3_burger"
    robotPosition = np.array([0, 0.0, 0.05])
    sensorPackPrimPath = robotPrimPath + "/base_footprint/base_link/sensor_pack"
elif env=="Rivermark" and robot =="turtlebot3_burger":
    usdPath=defines.RIVERMARK_USD_PATH
    robotPrimPath = "/World/turtlebot3_burger"
    robotPosition = np.array([0, 0.0, 5.9])
    sensorPackPrimPath = robotPrimPath + "/base_footprint/base_link/sensor_pack"
elif env=="City" and robot =="turtlebot3_burger":
    usdPath=defines.CITY_USD_PATH
    robotPrimPath = "/scene/turtlebot3_burger"
    robotPosition = np.array([0, 0.0, 0.05])
    sensorPackPrimPath = robotPrimPath + "/base_footprint/base_link/sensor_pack"
elif env=="NVIDIA_City" and robot =="turtlebot3_burger":
    usdPath=defines.NVIDIA_CITY_USD_PATH
    robotPrimPath = "/World/turtlebot3_burger"
    robotPosition = np.array([-13, 0.0, 0.0])
    sensorPackPrimPath = robotPrimPath + "/base_footprint/base_link/sensor_pack"
else:
    ## Uncomment below lines and modify it
    # usdPath="path to your usd file"
    # robotPrimPath = "prim path of your robot"
    # sensorPackPrimPath = robotPrimPath + "/base_footprint/base_link/sensor_pack"

    ## Remove below lines
    print("Not Imeplemented.")
    os.system("pgrep -f rviz | xargs -r kill -15")
    exit()

#############################################
###                                       ###
###          1. App Initalization         ###
###                                       ###
#############################################

# Redefine logger
def print_log(text):
    return omni.kit.app.get_app().print_and_log(text)

# Initialize Simulation App
from isaacsim import SimulationApp
app = SimulationApp(launch_config=defines.LAUNCH_CONFIG)

# Late import for Isaacsim & Omniverse API
import omni
import omni.kit.commands
import omni.graph.core as og
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf
import usdrt.Sdf
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.api import SimulationContext
import omni.replicator.core as rep


# Load Extensions
print_log("\n\n     Loading Extensions...\n")
for ext in defines.EXTENSIONS:
    enable_extension(ext)
app.update()

#############################################
###                                       ###
###        2. Scene + Robot Loading       ###
###                                       ###
#############################################

## USD Scene
print_log("\n\n     Loading USD Scene...\n")
open_stage(usdPath)
stage = omni.usd.get_context().get_stage()

# from omni.kit.menu.stage.content_browser_options import ContentBrowserOptions
# #url = "/home/do/Downloads/office.usd"
# url = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Environments/Office/office.usd"
# ContentBrowserOptions._add_file_to_stage(url, _settings, False)

app.update()


if env=="Cave":  # Change stage lighting to camera lighting.
    action_registry = omni.kit.actions.core.get_action_registry()
    action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
    action.execute()
    app.update()


from isaacsim.core.api import World
my_world = World(stage_units_in_meters=1.0,physics_dt=defines.PHYSICS_DT,rendering_dt=defines.RENDER_DT)
# simulation_context = SimulationContext(stage_units_in_meters=1.0,set_defaults=False)
# simulation_context.set_simulation_dt(physics_dt=defines.PHYSICS_DT,rendering_dt=defines.RENDER_DT)



app.update()




## Robot

print_log("Creating robot...")
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.prims import is_prim_path_valid

asset_path = get_assets_root_path() + "/Isaac/Robots/Turtlebot/turtlebot3_burger.usd"
my_world = World(stage_units_in_meters=1.0)
if is_prim_path_valid(robotPrimPath):
    print(f"[INFO] Existing robot found at {robotPrimPath}, reusing it.")
    robot = WheeledRobot(
        prim_path=robotPrimPath,
        name="turtlebot3_burger",
        wheel_dof_names=["wheel_left_joint", "wheel_right_joint"],
        create_robot=False  # 이미 있으므로 생성하지 않음
    )
    my_world.scene.add(robot)
else:
    print(f"[INFO] No robot found at {robotPrimPath}, creating new robot.")
    robot = my_world.scene.add(
    WheeledRobot(
        prim_path=robotPrimPath,
        name="turtlebot3_burger",
        wheel_dof_names=["wheel_left_joint", "wheel_right_joint"],
        create_robot=True,
        usd_path=asset_path,
        position=robotPosition,
        )
    )

my_controller = DifferentialController(name="simple_control", wheel_radius=0.025, wheel_base=0.16,max_linear_speed=1.5,max_angular_speed=1.0)


## Sensor Pack
print_log(f"\n\n     Searching sensor_pack...\n")
found_prims = []
target_name = "sensor_pack"
for prim in stage.Traverse():
    if prim.GetName() == target_name:
        found_prims.append(prim.GetPath())

if found_prims:
    print_log(f"\n\n     Prims named '{target_name}' found at paths:")
    for path in found_prims:
        print(f"     - {path}")
        if path != sensorPackPrimPath:
            omni.kit.commands.execute('ClearPhysicsComponentsCommand',
                stage=stage,
                prim_paths=[path])
            omni.kit.commands.execute('MovePrims',
                paths_to_move={path: sensorPackPrimPath},
                keep_world_transform=False,
                destructive=True)
            
else:
    print_log("\n\n     Could not find sensor_pack.\n     Configuring Sensor Pack...\n")
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.convex_decomp = False
    import_config.fix_base = False
    import_config.distance_scale = 1.0
    import_config.density = 0.0 

    status, sensor_pack = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=defines.SENSOR_PACK_URDF_PATH,
        import_config=import_config,
        get_articulation_root=False,
    )

    omni.kit.commands.execute('ClearPhysicsComponentsCommand',
        stage=stage,
        prim_paths=[sensor_pack])

    omni.kit.commands.execute('MovePrims',
        paths_to_move={sensor_pack: sensorPackPrimPath},
        keep_world_transform=False,
        destructive=True)



#############################################
###                                       ###
###        3. Sensors Configuration       ###
###                                       ###
#############################################

## Camera
print_log("\n\n     Configuring Sensors...\n")
if defines.ENABLE_SENSORS["Camera"]:
    camera_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(sensorPackPrimPath + defines.CAMERA_PREFIX_PATH + "/Camera", "Camera"))
    xform_api = UsdGeom.XformCommonAPI(camera_prim)
    if env in ["Cave", "Office"]:
        # xform_api.SetTranslate(Gf.Vec3d(0.03, 0.0, 0.16)) # this for 1x scale
        xform_api.SetTranslate(Gf.Vec3d(0.06, 0.0, 0.32)) # this for 2x scale
        # xform_api.SetTranslate(Gf.Vec3d(0.1, 0.0, 0.5)) # this for 3x scale
    elif env=="Rivermark":
        xform_api.SetTranslate(Gf.Vec3d(0.06, 0.0, 0.7))
    elif env=="City":
        xform_api.SetTranslate(Gf.Vec3d(0.06, 0.0, 0.32))
    elif env=="NVIDIA_City":
        xform_api.SetTranslate(Gf.Vec3d(0.06, 0.0, 0.32))
    xform_api.SetRotate((90, 0, -90), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    camera_prim.GetHorizontalApertureAttr().Set(21)
    camera_prim.GetVerticalApertureAttr().Set(16)
    camera_prim.GetProjectionAttr().Set("perspective")
    camera_prim.GetFocalLengthAttr().Set(24)
    camera_prim.GetFocusDistanceAttr().Set(400)
if defines.ENABLE_SENSORS["Camera2"]:
    camera_prim2 = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(sensorPackPrimPath + defines.CAMERA_PREFIX_PATH + "/Camera2", "Camera"))
    xform_api = UsdGeom.XformCommonAPI(camera_prim2)
    xform_api.SetTranslate(Gf.Vec3d(-1.2, 0.0, 1.5)) 
    xform_api.SetRotate((45, 0, -90), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    camera_prim2.GetHorizontalApertureAttr().Set(21)
    camera_prim2.GetVerticalApertureAttr().Set(16)
    camera_prim2.GetProjectionAttr().Set("perspective")
    camera_prim2.GetFocalLengthAttr().Set(24)
    camera_prim2.GetFocusDistanceAttr().Set(400)

## IMU
if defines.ENABLE_SENSORS["Imu"]:
    success, imu_prim = omni.kit.commands.execute(
        "IsaacSensorCreateImuSensor",
        path="Imu",
        parent=sensorPackPrimPath + defines.IMU_PREFIX_PATH,
        sensor_period=-1.0,
        linear_acceleration_filter_size=1,
        angular_velocity_filter_size=1,
        orientation_filter_size=1,
    )
app.update()


# Creating an on-demand push graph with cameraHelper nodes to generate ROS image publishers
keys = og.Controller.Keys
if defines.ENABLE_SENSORS["Camera"]:
    cameraNodeGraph_mono = {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("createViewport", "isaacsim.core.nodes.IsaacCreateViewport"),
                ("getRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"),
                ("setCamera", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"),
                ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
                ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
                ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                ("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("createViewport.inputs:viewportId", 0),
                ("cameraHelperRgb.inputs:frameId", "camera_link"),
                ("cameraHelperRgb.inputs:topicName", "rgb"),
                ("cameraHelperRgb.inputs:type", "rgb"),
                ("cameraHelperInfo.inputs:frameId", "camera_link"),
                ("cameraHelperInfo.inputs:topicName", "camera_info"),
                ("cameraHelperDepth.inputs:frameId", "camera_link"),
                ("cameraHelperDepth.inputs:topicName", "depth"),
                ("cameraHelperDepth.inputs:type", "depth"),
                ("setCamera.inputs:cameraPrim", [usdrt.Sdf.Path(sensorPackPrimPath + defines.CAMERA_PREFIX_PATH + "/Camera")]),
            ],
        }
    cameraNodeGraph_stereo = {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("createViewport", "isaacsim.core.nodes.IsaacCreateViewport"),("createViewport2", "isaacsim.core.nodes.IsaacCreateViewport"),
                ("getRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"), ("getRenderProduct2", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"),
                ("setCamera", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"), ("setCamera2", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"),
                ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"), ("cameraHelperRgb2", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"), ("cameraHelperInfo2", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"), ("cameraHelperDepth2", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "createViewport.inputs:execIn"), ("OnTick.outputs:tick", "createViewport2.inputs:execIn"),
                ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"), ("createViewport2.outputs:execOut", "getRenderProduct2.inputs:execIn"),
                ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"), ("createViewport2.outputs:viewport", "getRenderProduct2.inputs:viewport"),
                ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"), ("getRenderProduct2.outputs:execOut", "setCamera2.inputs:execIn"),
                ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"), ("getRenderProduct2.outputs:renderProductPath", "setCamera2.inputs:renderProductPath"),
                ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"), ("setCamera2.outputs:execOut", "cameraHelperRgb2.inputs:execIn"),
                ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"), ("setCamera2.outputs:execOut", "cameraHelperInfo2.inputs:execIn"),
                ("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"), ("setCamera2.outputs:execOut", "cameraHelperDepth2.inputs:execIn"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"), ("getRenderProduct2.outputs:renderProductPath", "cameraHelperRgb2.inputs:renderProductPath"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"), ("getRenderProduct2.outputs:renderProductPath", "cameraHelperInfo2.inputs:renderProductPath"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"), ("getRenderProduct2.outputs:renderProductPath", "cameraHelperDepth2.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("createViewport.inputs:viewportId", 0), ("createViewport2.inputs:viewportId", 1),
                ("cameraHelperRgb.inputs:frameId", "camera_link"), ("cameraHelperRgb2.inputs:frameId", "camera_link"),
                ("cameraHelperRgb.inputs:topicName", "rgb"), ("cameraHelperRgb2.inputs:topicName", "rgb2"),
                ("cameraHelperRgb.inputs:type", "rgb"), ("cameraHelperRgb2.inputs:type", "rgb"),
                ("cameraHelperInfo.inputs:frameId", "camera_link"), ("cameraHelperInfo2.inputs:frameId", "camera_link"),
                ("cameraHelperInfo.inputs:topicName", "camera_info"), ("cameraHelperInfo2.inputs:topicName", "camera_info2"),
                ("cameraHelperDepth.inputs:frameId", "camera_link"), ("cameraHelperDepth2.inputs:frameId", "camera_link"),
                ("cameraHelperDepth.inputs:topicName", "depth"), ("cameraHelperDepth2.inputs:topicName", "depth2"),
                ("cameraHelperDepth.inputs:type", "depth"), ("cameraHelperDepth2.inputs:type", "depth"),
                ("setCamera.inputs:cameraPrim", [usdrt.Sdf.Path(sensorPackPrimPath + defines.CAMERA_PREFIX_PATH + "/Camera")]), ("setCamera2.inputs:cameraPrim", [usdrt.Sdf.Path(sensorPackPrimPath + defines.CAMERA_PREFIX_PATH + "/Camera2")]),
            ],
        }
    cameraNodeGraph = cameraNodeGraph_stereo if defines.ENABLE_SENSORS["Camera2"] else cameraNodeGraph_mono
    (ros_camera_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": defines.ROS_CAMERA_GRAPH_PATH,
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        cameraNodeGraph,
    )
if defines.ENABLE_SENSORS["Imu"]:

    (ros_imu_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": defines.ROS_IMU_GRAPH_PATH,
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
                ("IsaacReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("ROS2Context","isaacsim.ros2.bridge.ROS2Context"),
                ("IsaacReadSimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ROS2PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            keys.CONNECT: [
                ("OnPhysicsStep.outputs:step", "IsaacReadIMU.inputs:execIn"),
                ("IsaacReadIMU.outputs:execOut", "ROS2PublishIMU.inputs:execIn"),
                ("IsaacReadIMU.outputs:angVel", "ROS2PublishIMU.inputs:angularVelocity"),
                ("IsaacReadIMU.outputs:linAcc", "ROS2PublishIMU.inputs:linearAcceleration"),
                ("IsaacReadIMU.outputs:orientation", "ROS2PublishIMU.inputs:orientation"),
                ("ROS2Context.outputs:context", "ROS2PublishIMU.inputs:context"),
                ("IsaacReadSimulationTime.outputs:simulationTime", "ROS2PublishIMU.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("ROS2PublishIMU.inputs:frameId", "imu_link"),
                ("IsaacReadIMU.inputs:readGravity", True),
                ("ROS2PublishIMU.inputs:topicName", "imu"),
                ("ROS2PublishIMU.inputs:publishAngularVelocity", True),
                ("ROS2PublishIMU.inputs:publishLinearAcceleration", True),
                ("ROS2PublishIMU.inputs:publishOrientation", True),
                ("ROS2Context.inputs:useDomainIDEnvVar",True),
                ("IsaacReadIMU.inputs:imuPrim",[usdrt.Sdf.Path(sensorPackPrimPath + defines.IMU_PREFIX_PATH + "/Imu")]),
            ],
        },
    )
if defines.ENABLE_SENSORS["TfOdom"]:
    (ros_tf_odom_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": defines.ROS_TF_ODOM_GRAPH_PATH,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("ROS2Context","isaacsim.ros2.bridge.ROS2Context"),
                ("IsaacComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("ROS2PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("ROS2PublishRawTransformTree", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ("ROS2PublishRawTransformTree1", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ("ROS2PublishTransformTree", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("ROS2PublishTransformTreeSensorPack", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("IsaacReadSimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "IsaacComputeOdometry.inputs:execIn"),
                ("OnTick.outputs:tick", "ROS2PublishRawTransformTree.inputs:execIn"),
                ("OnTick.outputs:tick", "ROS2PublishRawTransformTree1.inputs:execIn"),
                ("OnTick.outputs:tick", "ROS2PublishTransformTree.inputs:execIn"),
                ("OnTick.outputs:tick", "ROS2PublishTransformTreeSensorPack.inputs:execIn"),
                ("ROS2Context.outputs:context", "ROS2PublishOdometry.inputs:context"),
                ("ROS2Context.outputs:context", "ROS2PublishRawTransformTree.inputs:context"),
                ("ROS2Context.outputs:context", "ROS2PublishRawTransformTree1.inputs:context"),
                ("ROS2Context.outputs:context", "ROS2PublishTransformTree.inputs:context"),
                ("ROS2Context.outputs:context", "ROS2PublishTransformTreeSensorPack.inputs:context"),
                ("IsaacReadSimulationTime.outputs:simulationTime", "ROS2PublishOdometry.inputs:timeStamp"),
                ("IsaacReadSimulationTime.outputs:simulationTime", "ROS2PublishRawTransformTree.inputs:timeStamp"),
                ("IsaacReadSimulationTime.outputs:simulationTime", "ROS2PublishRawTransformTree1.inputs:timeStamp"),
                ("IsaacReadSimulationTime.outputs:simulationTime", "ROS2PublishTransformTree.inputs:timeStamp"),
                ("IsaacReadSimulationTime.outputs:simulationTime", "ROS2PublishTransformTreeSensorPack.inputs:timeStamp"),
                ("IsaacComputeOdometry.outputs:execOut", "ROS2PublishOdometry.inputs:execIn"),
                ("IsaacComputeOdometry.outputs:angularVelocity", "ROS2PublishOdometry.inputs:angularVelocity"),
                ("IsaacComputeOdometry.outputs:linearVelocity", "ROS2PublishOdometry.inputs:linearVelocity"),
                ("IsaacComputeOdometry.outputs:orientation", "ROS2PublishRawTransformTree.inputs:rotation"),
                ("IsaacComputeOdometry.outputs:orientation", "ROS2PublishOdometry.inputs:orientation"),
                ("IsaacComputeOdometry.outputs:position", "ROS2PublishRawTransformTree.inputs:translation"),
                ("IsaacComputeOdometry.outputs:position", "ROS2PublishOdometry.inputs:position"),
            ],
            keys.SET_VALUES: [
                ("ROS2Context.inputs:useDomainIDEnvVar",True),
                ("IsaacComputeOdometry.inputs:chassisPrim",robotPrimPath+"/base_footprint"), ## this must be set to robot's articulation root
                ("ROS2PublishOdometry.inputs:chassisFrameId", "base_link"),
                ("ROS2PublishOdometry.inputs:odomFrameId", "odom"),
                ("ROS2PublishOdometry.inputs:topicName", "odom"),
                ("ROS2PublishRawTransformTree.inputs:childFrameId", "base_link"),
                ("ROS2PublishRawTransformTree.inputs:parentFrameId", "odom"),
                ("ROS2PublishRawTransformTree.inputs:topicName", "tf"),
                ("ROS2PublishRawTransformTree1.inputs:childFrameId", "odom"),
                ("ROS2PublishRawTransformTree1.inputs:parentFrameId", "world"),
                ("ROS2PublishRawTransformTree1.inputs:topicName", "tf"),
                ("ROS2PublishTransformTree.inputs:parentPrim", robotPrimPath + "/base_footprint/base_link"),
                ("ROS2PublishTransformTree.inputs:targetPrims", [
                    robotPrimPath + "/base_footprint",
                    robotPrimPath + "/wheel_left_link",
                    robotPrimPath + "/wheel_right_link",
                    sensorPackPrimPath,
                    sensorPackPrimPath + "/sensors_link",
                    sensorPackPrimPath + "/sensors_link/camera_link",
                    sensorPackPrimPath + "/sensors_link/imu_link",
                    sensorPackPrimPath + "/sensors_link/lidar_link",
                    ]),
                ("ROS2PublishTransformTree.inputs:topicName", "tf"),
                ("ROS2PublishTransformTreeSensorPack.inputs:parentPrim", sensorPackPrimPath),
                ("ROS2PublishTransformTreeSensorPack.inputs:targetPrims", [
                    sensorPackPrimPath + "/sensors_link",
                    sensorPackPrimPath + "/sensors_link/camera_link",
                    sensorPackPrimPath + "/sensors_link/imu_link",
                    sensorPackPrimPath + "/sensors_link/lidar_link",
                    ]),
                ("ROS2PublishTransformTreeSensorPack.inputs:topicName", "tf"),
            ],
        },
    )
    # (ros_tf_odom_graph, _, _, _) = og.Controller.edit(
    #     {
    #         "graph_path": defines.ROS_TF_ODOM_GRAPH_PATH,
    #         "evaluator_name": "push",
    #         "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
    #     },
    #     {
    #         keys.CREATE_NODES: [
    #             ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
    #             ("ROS2Context","isaacsim.ros2.bridge.ROS2Context"),
    #             ("IsaacComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
    #             ("ROS2PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
    #             ("ROS2PublishRawTransformTree", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
    #             ("ROS2PublishRawTransformTree1", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
    #             ("ROS2PublishTransformTree", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
    #             ("ROS2PublishTransformTreeSensorPack", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
    #             ("IsaacReadSimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
    #         ],
    #         keys.CONNECT: [
    #             ("OnPhysicsStep.outputs:step", "IsaacComputeOdometry.inputs:execIn"),
    #             ("OnPhysicsStep.outputs:step", "ROS2PublishRawTransformTree.inputs:execIn"),
    #             ("OnPhysicsStep.outputs:step", "ROS2PublishRawTransformTree1.inputs:execIn"),
    #             ("OnPhysicsStep.outputs:step", "ROS2PublishTransformTree.inputs:execIn"),
    #             ("OnPhysicsStep.outputs:step", "ROS2PublishTransformTreeSensorPack.inputs:execIn"),
    #             ("ROS2Context.outputs:context", "ROS2PublishOdometry.inputs:context"),
    #             ("ROS2Context.outputs:context", "ROS2PublishRawTransformTree.inputs:context"),
    #             ("ROS2Context.outputs:context", "ROS2PublishRawTransformTree1.inputs:context"),
    #             ("ROS2Context.outputs:context", "ROS2PublishTransformTree.inputs:context"),
    #             ("ROS2Context.outputs:context", "ROS2PublishTransformTreeSensorPack.inputs:context"),
    #             ("IsaacReadSimulationTime.outputs:simulationTime", "ROS2PublishOdometry.inputs:timeStamp"),
    #             ("IsaacReadSimulationTime.outputs:simulationTime", "ROS2PublishRawTransformTree.inputs:timeStamp"),
    #             ("IsaacReadSimulationTime.outputs:simulationTime", "ROS2PublishRawTransformTree1.inputs:timeStamp"),
    #             ("IsaacReadSimulationTime.outputs:simulationTime", "ROS2PublishTransformTree.inputs:timeStamp"),
    #             ("IsaacReadSimulationTime.outputs:simulationTime", "ROS2PublishTransformTreeSensorPack.inputs:timeStamp"),
    #             ("IsaacComputeOdometry.outputs:execOut", "ROS2PublishOdometry.inputs:execIn"),
    #             ("IsaacComputeOdometry.outputs:angularVelocity", "ROS2PublishOdometry.inputs:angularVelocity"),
    #             ("IsaacComputeOdometry.outputs:linearVelocity", "ROS2PublishOdometry.inputs:linearVelocity"),
    #             ("IsaacComputeOdometry.outputs:orientation", "ROS2PublishRawTransformTree.inputs:rotation"),
    #             ("IsaacComputeOdometry.outputs:orientation", "ROS2PublishOdometry.inputs:orientation"),
    #             ("IsaacComputeOdometry.outputs:position", "ROS2PublishRawTransformTree.inputs:translation"),
    #             ("IsaacComputeOdometry.outputs:position", "ROS2PublishOdometry.inputs:position"),
    #         ],
    #         keys.SET_VALUES: [
    #             ("ROS2Context.inputs:useDomainIDEnvVar",True),
    #             ("IsaacComputeOdometry.inputs:chassisPrim",robotPrimPath+"/base_footprint"), ## this must be set to robot's articulation root
    #             ("ROS2PublishOdometry.inputs:chassisFrameId", "base_link"),
    #             ("ROS2PublishOdometry.inputs:odomFrameId", "odom"),
    #             ("ROS2PublishOdometry.inputs:topicName", "odom"),
    #             ("ROS2PublishRawTransformTree.inputs:childFrameId", "base_link"),
    #             ("ROS2PublishRawTransformTree.inputs:parentFrameId", "odom"),
    #             ("ROS2PublishRawTransformTree.inputs:topicName", "tf"),
    #             ("ROS2PublishRawTransformTree1.inputs:childFrameId", "odom"),
    #             ("ROS2PublishRawTransformTree1.inputs:parentFrameId", "world"),
    #             ("ROS2PublishRawTransformTree1.inputs:topicName", "tf"),
    #             ("ROS2PublishTransformTree.inputs:parentPrim", robotPrimPath + "/base_footprint/base_link"),
    #             ("ROS2PublishTransformTree.inputs:targetPrims", [
    #                 robotPrimPath + "/base_footprint",
    #                 robotPrimPath + "/wheel_left_link",
    #                 robotPrimPath + "/wheel_right_link",
    #                 sensorPackPrimPath,
    #                 ]),
    #             ("ROS2PublishTransformTree.inputs:topicName", "tf"),
    #             ("ROS2PublishTransformTreeSensorPack.inputs:parentPrim", sensorPackPrimPath),
    #             ("ROS2PublishTransformTreeSensorPack.inputs:targetPrims", [
    #                 sensorPackPrimPath + "/sensors_link",
    #                 sensorPackPrimPath + "/sensors_link/camera_link",
    #                 sensorPackPrimPath + "/sensors_link/imu_link",
    #                 sensorPackPrimPath + "/sensors_link/lidar_link",
    #                 ]),
    #             ("ROS2PublishTransformTreeSensorPack.inputs:topicName", "tf"),
    #         ],
    #     },
    # )

# Run the ROS Camera graph once to generate ROS image publishers in SDGPipeline
if defines.ENABLE_SENSORS["Camera"]: og.Controller.evaluate_sync(ros_camera_graph)
app.update()

# dock in
if defines.ENABLE_SENSORS["Camera"] and defines.ENABLE_SENSORS["Camera2"]:
    import omni.kit.viewport.utility as vp_utils
    # Dock the second camera window
    left_viewport = omni.ui.Workspace.get_window("Viewport")
    right_viewport = omni.ui.Workspace.get_window("1")
    if right_viewport is not None and left_viewport is not None:
        right_viewport.dock_in(left_viewport, omni.ui.DockPosition.RIGHT)
    right_viewport = None
    left_viewport = None


# Lidar : /Render/PostProcess/SDGPipeline/Isaac_PostProcessDispatchIsaacSimulationGate
# Rgb : /Render/PostProcess/SDGPipeline/omni_kit_widget_viewport_ViewportTexture_0_LdrColorSDIsaacSimulationGate
# Camerainfo : /Render/PostProcess/SDGPipeline/omni_kit_widget_viewport_ViewportTexture_0_PostProcessDispatchIsaacSimulationGate


# Need to initialize physics getting any articulation..etc
# simulation_context.initialize_physics()
my_world.stop()

# Lidar
if defines.ENABLE_SENSORS["Lidar"]:
    _, lidar_prim = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=sensorPackPrimPath + defines.LIDAR_PREFIX_PATH + "/Lidar",
        parent=None,
        config=defines.LIDAR_MODEL,
        # translation=(-0.03, 0, 0.18), # this for 1x scale
        translation=(-0.07, 0, 0.4), # this for 2x scale
        # translation=(-0.1, 0, 0.57), # this for 3x scale
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
    )
    # RTX sensors are cameras and must be assigned to their own render product
    lidar_render_product = rep.create.render_product(lidar_prim.GetPath(), [1, 1], name="Isaac")
    # Create Point cloud publisher pipeline in the post process graph
    writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
    writer.initialize(topicName="point_cloud", frameId="lidar_link")
    writer.attach([lidar_render_product])

    if defines.ENABLE_SENSORS["DebugLidar"]:
        # Create the debug draw pipeline in the post process graph
        writer1 = rep.writers.get("RtxLidar" + "DebugDrawPointCloud")
        writer1.attach([lidar_render_product])

lidar_gate_path = "/Render/PostProcess/SDGPipeline/Isaac_PostProcessDispatchIsaacSimulationGate"
lidar_step_size = 3
og.Controller.attribute(lidar_gate_path+".inputs:step").set(lidar_step_size)




## Change Settings
import carb
import carb.settings
_settings = carb.settings.get_settings()
# For Performance
_settings.set_bool("/rtx/ecoMode/enabled", False) 
_settings.set_bool("/rtx/directLighting/enabled", False) 
_settings.set_bool("/rtx/indirectDiffuse/enabled", False) 
_settings.set_bool("/rtx/ambientOcclusion/enabled", False) 
_settings.set_bool("/rtx/reflections/enabled", False) 
_settings.set_bool("/rtx/translucency/enabled", False) 
_settings.set_bool("/rtx/post/histogram/enabled", False) 

# Brightness
_settings.set_float("/rtx/sceneDb/ambientLightIntensity", 0.4)
omni.kit.commands.execute('ChangeSetting',
	path='/rtx/sceneDb/ambientLightColor',
	value=[1.0, 1.0, 1.0])



## Configure Physics
print_log(f"\n\n     Configuring Physics...\n")
found_prims = []
target_name = "physicsScene"
for prim in stage.Traverse():
    if prim.GetName() == target_name:
        found_prims.append(prim.GetPath())

physics_path=""
if found_prims:
    for path in found_prims:
        physics_scene = UsdPhysics.Scene(stage.GetPrimAtPath(path))
        physics_path = path
else:
    physics_path = "/physicsScene"
    physics_scene = UsdPhysics.Scene.Define(stage, physics_path)


# preset for best performance simulation
omni.kit.commands.execute('ChangeProperty',
	prop_path=Sdf.Path(f'{physics_path}.physxScene:solverType'),
	value='PGS',
	prev=None,
	usd_context_name=stage)
omni.kit.commands.execute('ChangeProperty',
	prop_path=Sdf.Path(f'{physics_path}.physxScene:enableGPUDynamics'),
	value=False,
	prev=None,
	usd_context_name=stage)
omni.kit.commands.execute('ChangeProperty',
	prop_path=Sdf.Path(f'{physics_path}.physxScene:broadphaseType'),
	value='MBP',
	prev=None,
	usd_context_name=stage)
omni.kit.commands.execute('ChangeProperty',
	prop_path=Sdf.Path(f'{physics_path}.physxScene:enableCCD'),
	value=False,
	prev=None,
	usd_context_name=stage)
omni.kit.commands.execute('ChangeProperty',
	prop_path=Sdf.Path(f'{physics_path}.physxScene:enableStabilization'),
	value=True,
	prev=None,
	usd_context_name=stage)







print_log("\n\n     Simulator Ready!\n")




#############################################
###                                       ###
###              4. Main Loop             ###
###                                       ###
#############################################
my_world.reset()
my_world.stop()
my_controller.reset()
from pynput import keyboard as pynkeyboard
import numpy as np

print("Control Instructions:")
print("  W: Move Forward")
print("  A: Rotate Left")
print("  D: Rotate Right")
print("  S: Stop")

# 키 상태 저장용 딕셔너리
key_state = {'w': False, 'a': False, 's': False, 'd': False, 'x': False, 'q': False}

# 키 눌렀을 때
def on_press(key):
    try:
        k = key.char.lower()
        if k in key_state:
            key_state[k] = True
    except AttributeError:
        pass  # special keys 무시

# 키 뗐을 때
def on_release(key):
    try:
        k = key.char.lower()
        if k in key_state:
            key_state[k] = False
    except AttributeError:
        pass

# 비동기 리스너 시작
listener = pynkeyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# 메인 루프
tick = 0
reset_needed = False
velocity=[0.0,0.0]
dv = 0.01
dr = np.pi/120.0
while app.is_running():
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True

    app.update()
    if tick != 0:
        tick = 0

    ## Real-time Syncronous Simulation
    while my_world.is_playing() and defines.ENABLE_REALTIME_SYNC:
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            reset_needed = False

        now = perf_counter()

        if tick == 0:
            next_physics_time = now + defines.PHYSICS_DT
            next_render_time = now + defines.RENDER_DT

        while now > next_physics_time:
            my_world.step(render=False)
            next_physics_time += defines.PHYSICS_DT

        if now > next_render_time:
            my_world.render()
            next_render_time += defines.RENDER_DT
            if defines.ENABLE_SENSORS["TfOdom"]: ros_tf_odom_graph.evaluate()
            # 키 입력에 따라 로봇 제어, 100hz시 키보드 컨트롤로 인한 지연 예방을 위해 30hz 루프에서 실행
            if key_state['w']:
                velocity[0]+=dv
                robot.apply_wheel_actions(my_controller.forward(command=velocity))
                print("Forward:", robot.get_linear_velocity())
            elif key_state['a']:
                velocity[1]+=dr
                robot.apply_wheel_actions(my_controller.forward(command=velocity))
                print("Rotate Left:", robot.get_angular_velocity())
            elif key_state['d']:
                velocity[1]-=dr
                robot.apply_wheel_actions(my_controller.forward(command=velocity))
                print("Rotate Right:", robot.get_angular_velocity())
            elif key_state['s']:
                velocity[0] = 0.0
                velocity[1] = 0.0
                robot.apply_wheel_actions(my_controller.forward(command=velocity))
                print("Stop")
            elif key_state['x']:
                velocity[0]-=dv
                robot.apply_wheel_actions(my_controller.forward(command=velocity))
                print("Backward:", robot.get_linear_velocity())
            elif key_state['q']:
                my_world.stop()
                break

        tick += 1
        if tick > 1e8:
            tick = 1

    ## Best Performance Simulation
    while my_world.is_playing() and not defines.ENABLE_REALTIME_SYNC:
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            reset_needed = False


        my_world.step(render=False)

        if tick % 3==2:
            my_world.render()

            if defines.ENABLE_SENSORS["TfOdom"]: ros_tf_odom_graph.evaluate()

            # 키 입력에 따라 로봇 제어, 100hz시 키보드 컨트롤로 인한 지연 예방을 위해 30hz 루프에서 실행
            if key_state['w']:
                velocity[0]+=dv
                robot.apply_wheel_actions(my_controller.forward(command=velocity))
                print("Forward:", robot.get_linear_velocity())
            elif key_state['a']:
                velocity[1]+=dr
                robot.apply_wheel_actions(my_controller.forward(command=velocity))
                print("Rotate Left:", robot.get_angular_velocity())
            elif key_state['d']:
                velocity[1]-=dr
                robot.apply_wheel_actions(my_controller.forward(command=velocity))
                print("Rotate Right:", robot.get_angular_velocity())
            elif key_state['s']:
                velocity[0] = 0.0
                velocity[1] = 0.0
                robot.apply_wheel_actions(my_controller.forward(command=velocity))
                print("Stop")
            elif key_state['x']:
                velocity[0]-=dv
                robot.apply_wheel_actions(my_controller.forward(command=velocity))
                print("Backward:", robot.get_linear_velocity())
            elif key_state['q']:
                my_world.stop()
                break

        tick += 1
        if tick > 1e8:
            tick = 1

my_world.stop()
app.close()
os.system("pgrep -f rviz | xargs -r kill -15")
exit()
