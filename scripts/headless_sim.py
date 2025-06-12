#############################################
###                                       ###
###           0. Preset Selection         ###
###                                       ###
#############################################
import defines
import os
from time import sleep, perf_counter
from preset_selector import PresetSelector
from PyQt5.QtWidgets import QApplication, QDialog
import sys
import numpy as np

sleep(2) # waiting for RVIZ startup
app = QApplication(sys.argv)

preset_selector = PresetSelector(env_list=defines.ENVS, robot_list=defines.ROBOTS, sensor_config=defines.ENABLE_SENSORS, lidar_list=defines.LIDAR_MODELS)

if preset_selector.exec_() == QDialog.Accepted:
    _env, _robot, _performance_mode, _physics_steps, _render_steps, _sensor_settings, _lidar_model = preset_selector.get_selections()
    _physics_dt = 1.0 / _physics_steps
    _render_dt = 1.0 / _render_steps
    _sync_with_realtime = False if _performance_mode == "Best performance" else True

    print(f"Selected Environment: {_env}")
    print(f"Selected Robot: {_robot}")
    print(f"Performance Mode : {_performance_mode}")
    print(f"Simulation Physics_dt : {_physics_dt}")
    print(f"Simuation Render_dt : {_render_dt}")
    print(f"Sensor Settings: {_sensor_settings}")
    print(f"Selected LiDAR: {_lidar_model}")
else:
    print("Selection canceled")
    os.system("pgrep -f rviz | xargs -r kill -15")
    exit()
app.quit()

from pathlib import Path

script_dir = Path(__file__).resolve().parent
parent_dir = str(script_dir.parent)
print(parent_dir)

usdPath = parent_dir + defines.USD_PATH[_env]
robotPrimPath = f"/{_robot}"
if _robot == "turtlebot3_burger":
    robotPosition = defines.TURTLEBOT_POSITION[_env]
    sensorPackPrimPath = robotPrimPath + "/base_footprint/base_link/sensor_pack"
elif _robot == "unitree_g1":
    if _env == "Rivermark":
        robotPosition = [0.0,0.0,6.5]
    else:
        robotPosition = [0.0,0.0,0.76]
else:
    os.system("pgrep -f rviz | xargs -r kill -15")
    raise NotImplementedError
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
from isaacsim.core.utils.stage import open_stage, add_reference_to_stage
from isaacsim.core.api import SimulationContext
import omni.replicator.core as rep
from sim_utils import DistanceCalculator, IntervalChecker

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
# In cave, texture will be broken when using add_reference_to_stage().
# Also, unitree_g1 locomotion is not working when using open_stage().
print_log("\n\n     Loading USD Scene...\n")
if _env == "Cave": 
    open_stage(usdPath)
else:
    add_reference_to_stage(
        usd_path=usdPath,
        prim_path="/World/env"
    )
stage = omni.usd.get_context().get_stage()

# from omni.kit.menu.stage.content_browser_options import ContentBrowserOptions
# #url = "/home/do/Downloads/office.usd"
# url = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Environments/Office/office.usd"
# ContentBrowserOptions._add_file_to_stage(url, _settings, False)

app.update()


if _env=="Cave":  # Change stage lighting to camera lighting.
    action_registry = omni.kit.actions.core.get_action_registry()
    action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
    action.execute()
    app.update()


from isaacsim.core.api import World
world = World(stage_units_in_meters=1.0,physics_dt=_physics_dt,rendering_dt=_render_dt)
# simulation_context = SimulationContext(stage_units_in_meters=1.0,set_defaults=False)
# simulation_context.set_simulation_dt(physics_dt=_physics_dt,rendering_dt=_render_dt)



app.update()


## Robot

print_log(f"Creating robot at {robotPrimPath}...")
world = World(stage_units_in_meters=1.0)
if _robot == "turtlebot3_burger":
    from isaacsim.robot.wheeled_robots.robots import WheeledRobot
    from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
    from isaacsim.storage.native import get_assets_root_path
    from isaacsim.core.utils.prims import is_prim_path_valid

    asset_path = get_assets_root_path() + "/Isaac/Robots/Turtlebot/turtlebot3_burger.usd"
    
    for _ in defines.ROBOTS:
        try:
            omni.kit.commands.execute('DeletePrims',
                paths=[Sdf.Path('/'+_)],
                destructive=False)
        except:
            pass
    robot_prim = world.scene.add(
        WheeledRobot(
            prim_path=robotPrimPath,
            name="turtlebot3_burger",
            wheel_dof_names=["wheel_left_joint", "wheel_right_joint"],
            create_robot=True,
            usd_path=asset_path,
            position=np.array(robotPosition),
        )
    )
    if _env == "Rivermark":
        my_controller = DifferentialController(name="simple_control", wheel_radius=0.25, wheel_base=1.6,max_linear_speed=2.0,max_angular_speed=1.0)
    else:
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
            urdf_path=parent_dir + defines.SENSOR_PACK_URDF_PATH,
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
        
elif _robot == "unitree_g1":
    from isaacsim_g1_locomotion.g1 import G1FlatTerrainPolicy
    for _ in defines.ROBOTS:
        try:
            omni.kit.commands.execute('DeletePrims',
                paths=[Sdf.Path('/'+_)],
                destructive=False)
        except:
            pass

    base_command = [0, 0, 0]
    g1 = G1FlatTerrainPolicy(prim_path=robotPrimPath, name=_robot,usd_path=parent_dir + defines.G1_USD_PATH)
    # xform_api = UsdGeom.XformCommonAPI(robotPrimPath)
    # xform_api.SetTranslate(Gf.Vec3d(0.0, 0.0, 0.75))

    omni.kit.commands.execute('ChangeProperty',
        prop_path=Sdf.Path(f'{robotPrimPath + "/g1_minimal"}.xformOp:translate'),
        value=Gf.Vec3d(*robotPosition),
        prev=None,
        usd_context_name=stage)

app.update()

#############################################
###                                       ###
###        3. Sensors Configuration       ###
###                                       ###
#############################################

## Camera
print_log("\n\n     Configuring Sensors...\n")
if _sensor_settings["Camera"]:
    if _robot == "turtlebot3_burger":
        camera1_prim_path = sensorPackPrimPath + defines.CAMERA_PREFIX_PATH + "/Camera"
    elif _robot == "unitree_g1":
        camera1_prim_path = robotPrimPath + "/g1_minimal/torso_link/head_joint/d435_link" + "/Camera"
    else:
        os.system("pgrep -f rviz | xargs -r kill -15")
        raise NotImplementedError
    
    camera_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(camera1_prim_path, "Camera"))
    xform_api = UsdGeom.XformCommonAPI(camera_prim)
    if _robot == "turtlebot3_burger":
        if _env in ["Cave", "Office"]:
            # xform_api.SetTranslate(Gf.Vec3d(0.03, 0.0, 0.16)) # this for 1x scale
            xform_api.SetTranslate(Gf.Vec3d(0.06, 0.0, 0.32)) # this for 2x scale
            # xform_api.SetTranslate(Gf.Vec3d(0.1, 0.0, 0.5)) # this for 3x scale
        elif _env=="Rivermark":
            xform_api.SetTranslate(Gf.Vec3d(0.0, 0.0, 0.13))
        elif _env=="City":
            xform_api.SetTranslate(Gf.Vec3d(0.06, 0.0, 0.32))
        elif _env=="NVIDIA_City":
            xform_api.SetTranslate(Gf.Vec3d(0.06, 0.0, 0.32))
    elif _robot == "unitree_g1":
        xform_api.SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))
    else:
        os.system("pgrep -f rviz | xargs -r kill -15")
        raise NotImplementedError
    xform_api.SetRotate((90, 0, -90), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    camera_prim.GetHorizontalApertureAttr().Set(21)
    camera_prim.GetVerticalApertureAttr().Set(16)
    camera_prim.GetProjectionAttr().Set("perspective")
    camera_prim.GetFocalLengthAttr().Set(24)
    camera_prim.GetFocusDistanceAttr().Set(400)
if _sensor_settings["Camera2"]:
    if _robot == "turtlebot3_burger":
        camera2_prim_path = sensorPackPrimPath + defines.CAMERA_PREFIX_PATH + "/Camera2"
        cam2_traslation = Gf.Vec3d(-1.2, 0.0, 1.5)
        cam2_rotation = (45, 0, -90)
    elif _robot == "unitree_g1":
        camera2_prim_path = robotPrimPath + "/g1_minimal/torso_link/head_joint/d435_link" + "/Camera2"
        cam2_traslation = Gf.Vec3d(-2.6, 0.0, -2.2)
        cam2_rotation = (120, 0, -90)
    else:
        os.system("pgrep -f rviz | xargs -r kill -15")
        raise NotImplementedError
    camera_prim2 = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(camera2_prim_path, "Camera"))
    xform_api = UsdGeom.XformCommonAPI(camera_prim2)
    xform_api.SetTranslate(cam2_traslation) 
    xform_api.SetRotate(cam2_rotation, UsdGeom.XformCommonAPI.RotationOrderXYZ)
    camera_prim2.GetHorizontalApertureAttr().Set(21)
    camera_prim2.GetVerticalApertureAttr().Set(16)
    camera_prim2.GetProjectionAttr().Set("perspective")
    camera_prim2.GetFocalLengthAttr().Set(24)
    camera_prim2.GetFocusDistanceAttr().Set(400)

## IMU
if _sensor_settings["Imu"]:
    if _robot == "turtlebot3_burger":
        imu_prim_path = sensorPackPrimPath + defines.IMU_PREFIX_PATH
    elif _robot == "unitree_g1":
        imu_prim_path = robotPrimPath + "/g1_minimal/imu_link"
    else:
        os.system("pgrep -f rviz | xargs -r kill -15")
        raise NotImplementedError
    
    success, imu_prim = omni.kit.commands.execute(
        "IsaacSensorCreateImuSensor",
        path="Imu",
        parent=imu_prim_path,
        sensor_period=-1.0,
        linear_acceleration_filter_size=1,
        angular_velocity_filter_size=1,
        orientation_filter_size=1,
    )
app.update()


# Creating an on-demand push graph with cameraHelper nodes to generate ROS image publishers
keys = og.Controller.Keys
if _sensor_settings["Camera"]:
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
                ("setCamera.inputs:cameraPrim", [usdrt.Sdf.Path(camera1_prim_path)]),
            ],
        }
    if _sensor_settings["Camera2"]:
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
                ("setCamera.inputs:cameraPrim", [usdrt.Sdf.Path(camera1_prim_path)]), ("setCamera2.inputs:cameraPrim", [usdrt.Sdf.Path(camera2_prim_path)]),
            ],
        }
    cameraNodeGraph = cameraNodeGraph_stereo if _sensor_settings["Camera2"] else cameraNodeGraph_mono
    (ros_camera_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": defines.ROS_CAMERA_GRAPH_PATH,
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        cameraNodeGraph,
    )
if _sensor_settings["Imu"]:
    
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
                ("IsaacReadIMU.inputs:imuPrim",[usdrt.Sdf.Path(imu_prim_path + "/Imu")]),
            ],
        },
    )
if _sensor_settings["TfOdom"]:
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

# Run the ROS Camera graph once to generate ROS image publishers in SDGPipeline
if _sensor_settings["Camera"]: og.Controller.evaluate_sync(ros_camera_graph)
app.update()

# dock in
if _sensor_settings["Camera"] and _sensor_settings["Camera2"]:
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


if _sensor_settings["Lidar"]:
    if _env == "Rivermark":
        lidar_offset = (-0.03, 0, 0.21)
    else:
        lidar_offset = (-0.07, 0, 0.4)
    if _robot == "turtlebot3_burger":
        lidar_prim_path = sensorPackPrimPath + defines.LIDAR_PREFIX_PATH + "/Lidar1"
    else:
        lidar_prim_path = robotPrimPath + "/g1_minimal/torso_link/head_joint/mid360_link" + "/Lidar"


    if _lidar_model == "Livox_MID360":
        lidar_prims=[]
        lidar_render_products = []
        writers = []
        num_livox_parts = 5
        lidar_steps = [11,12,13,14,15]

        # Create parts of MID360
        for _ in range(num_livox_parts):
            _, lidar_prim = omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                path=lidar_prim_path,
                parent=None,
                config="MID360_" + str(_+1),
                # translation=(-0.03, 0, 0.18), # this for 1x scale
                translation=lidar_offset, # this for 2x scale
                # translation=(-0.1, 0, 0.57), # this for 3x scale
                orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
            )
            lidar_prims.append(lidar_prim)

            # RTX sensors are cameras and must be assigned to their own render product
            lidar_render_product = rep.create.render_product(lidar_prim.GetPath(), [1, 1], name="Isaac")
            lidar_render_products.append(lidar_render_product)

            # Create Point cloud publisher pipeline in the post process graph
            writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
            writer.initialize(topicName="point_cloud", frameId="lidar_link")
            writer.attach([lidar_render_product])
            writers.append(writer)
            if _sensor_settings["DebugLidar"]:
                # Create the debug draw pipeline in the post process graph
                writerd = rep.writers.get("RtxLidar" + "DebugDrawPointCloud")
                writerd.attach([lidar_render_product])
                writers.append(writerd)

        lidar_gate_paths = [
            "/Render/PostProcess/SDGPipeline/Isaac_PostProcessDispatchIsaacSimulationGate",
            "/Render/PostProcess/SDGPipeline/Isaac_01_PostProcessDispatchIsaacSimulationGate",
            "/Render/PostProcess/SDGPipeline/Isaac_02_PostProcessDispatchIsaacSimulationGate",
            "/Render/PostProcess/SDGPipeline/Isaac_03_PostProcessDispatchIsaacSimulationGate",
            "/Render/PostProcess/SDGPipeline/Isaac_04_PostProcessDispatchIsaacSimulationGate"
            ]
        for path,step in zip(lidar_gate_paths,lidar_steps):
            og.Controller.attribute(path+".inputs:step").set(step)
            

    else:
        _, lidar_prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=lidar_prim_path,
            parent=None,
            config=_lidar_model,
            translation=lidar_offset,
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
        )
        # RTX sensors are cameras and must be assigned to their own render product
        lidar_render_product = rep.create.render_product(lidar_prim.GetPath(), [1, 1], name="Isaac")
        # Create Point cloud publisher pipeline in the post process graph
        writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
        writer.initialize(topicName="point_cloud", frameId="lidar_link")
        writer.attach([lidar_render_product])

        if _sensor_settings["DebugLidar"]:
            # Create the debug draw pipeline in the post process graph
            writer1 = rep.writers.get("RtxLidar" + "DebugDrawPointCloud")
            writer1.attach([lidar_render_product])

        lidar_gate_path = "/Render/PostProcess/SDGPipeline/Isaac_PostProcessDispatchIsaacSimulationGate"
        lidar_step_size = 3
        og.Controller.attribute(lidar_gate_path+".inputs:step").set(lidar_step_size)
            # Change Settings
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
world.reset()
world.stop()

#############################################
###                                       ###
###              4. Main Loop             ###
###                                       ###
#############################################
import numpy as np
if _robot == "turtlebot3_burger":
    my_controller.reset()
    print("[ Tuetlebot Control Instructions ]")
    print(" ↑ : Increase linear velocity")
    print(" ↓ : Decrease linear velocity")
    print(" ← : Rotate Left")
    print(" → : Rotate Right")
    print(" S : Stop")

    ## Setup Keyboard
    velocity=[0.0,0.0] # lin_vel,ang_vel
    dv = 0.05
    dr = np.pi/36.0

    _key_to_control = {
        "UP"   : [dv,  0.0],
        "DOWN" : [-dv, 0.0],
        "LEFT" : [0.0, dr],
        "RIGHT": [0.0, -dr],
    }

    _input = carb.input.acquire_input_interface()
    _keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    need_update_vel = True

    def _on_keyboard_event(event):
        global velocity, need_update_vel
        """Checks for a keyboard event and assign the corresponding command control depending on key pressed."""
        if event.type in [carb.input.KeyboardEventType.KEY_PRESS, carb.input.KeyboardEventType.KEY_REPEAT]:
            # Arrow keys map to pre-defined command vectors to control navigation of robot
            if event.input.name in _key_to_control:
                velocity[0] += _key_to_control[event.input.name][0]
                velocity[1] += _key_to_control[event.input.name][1]
                print(f"  Keybord : {event.input.name} >> Set Velocity : {velocity}")
                need_update_vel = True
            elif event.input.name == "S":
                velocity = [0.0,0.0]
                print(f"  Keybord : {event.input.name} >> Set Velocity : {velocity}")
                need_update_vel = True


elif _robot == "unitree_g1":
    print("[ G1 Control Instructions ]")
    print(" ↑ : Move Forward")
    print(" ↓ : Stop")
    print(" ← : Rotate Left")
    print(" → : Rotate Right")

    _key_to_control = {
        "UP": [0.5,0,0],
        "DOWN": [0,0,0],
        "LEFT": [0,0,1],
        "RIGHT": [0,0,-1],
    }

    # Set up Keyboard
    def _on_keyboard_event(event):
        global base_command
        """Checks for a keyboard event and assign the corresponding command control depending on key pressed."""
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            # Arrow keys map to pre-defined command vectors to control navigation of robot
            if event.input.name in _key_to_control:
                base_command = _key_to_control[event.input.name]
                print(f"Keybord : {event.input.name}                    ")
        # On key release, the robot stops moving
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            base_command = _key_to_control["DOWN"]

else:
    os.system("pgrep -f rviz | xargs -r kill -15")
    raise NotImplementedError()

"""Sets up interface for keyboard input and registers the desired keys for control."""
_input = carb.input.acquire_input_interface()
_keyboard = omni.appwindow.get_default_app_window().get_keyboard()
_sub_keyboard = _input.subscribe_to_keyboard_events(_keyboard, _on_keyboard_event)

# 메인 루프
tick = 0
reset_needed = False
need_update_vel = False

trace_prim = robotPrimPath+"/base_footprint" if _robot == "turtlebot3_burger" else robotPrimPath+"/g1_minimal/torso_link"
distance_calculator = DistanceCalculator(trace_prim)
print(f"Target tracing prim : {trace_prim}")

world.play()
while app.is_running():
    if world.is_stopped() and not reset_needed:
        reset_needed = True

    app.update()
    if tick != 0: tick = 0

    ## Real-time Syncronous Simulation
    while world.is_playing() and _sync_with_realtime:
        if reset_needed:
            world.reset()
            if _robot == "turtlebot3_burger": my_controller.reset()
            reset_needed = False

        if tick == 0:
            if _robot == "unitree_g1":
                world.reset()
                g1.initialize()
                
                def on_physics_step(step_size):
                    if g1:
                        g1.forward(step_size, base_command)
                world.add_physics_callback("physics_step", callback_fn=on_physics_step)
            now = perf_counter()
            next_physics_time = now + _physics_dt
            next_render_time = now + _render_dt

        now = perf_counter()

        while now > next_physics_time:
            world.step(render=False)
            next_physics_time += _physics_dt
            total_distance = distance_calculator.update_distance()

        if now > next_render_time:
            world.render()
            next_render_time += _render_dt
            print(f" Traveled distance: {total_distance:.3f} meters",end = "\r")
            if _sensor_settings["TfOdom"]: ros_tf_odom_graph.evaluate()
            
        if _robot == "turtlebot3_burger": robot_prim.apply_wheel_actions(my_controller.forward(command=velocity))

        tick += 1
        if tick > 1e8:
            tick = 1

    ## Best Performance Simulation
    while world.is_playing() and not _sync_with_realtime:
        if reset_needed:
            world.reset()
            if _robot == "turtlebot3_burger": my_controller.reset()
            reset_needed = False
        if tick == 0:
            if _robot == "unitree_g1":
                world.reset()
                g1.initialize()
                
                def on_physics_step(step_size):
                    if g1:
                        g1.forward(step_size, base_command)
                world.add_physics_callback("physics_step", callback_fn=on_physics_step)
            now = perf_counter()
            next_physics_time = now + _physics_dt
            next_render_time = now + _render_dt


        world.step(render=False)
        total_distance = distance_calculator.update_distance()
        
        if tick % 3==2:
            world.render()
            print(f" Traveled distance: {total_distance:.3f} meters",end = "\r")
            if _sensor_settings["TfOdom"]: ros_tf_odom_graph.evaluate()
            
        if need_update_vel:
            need_update_vel = False
            if _robot == "turtlebot3_burger": robot_prim.apply_wheel_actions(my_controller.forward(command=velocity))

        tick += 1
        if tick > 1e8:
            tick = 1

world.stop()
app.close()
os.system("pgrep -f rviz | xargs -r kill -15")
exit()
