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
import json
import numpy as np
from pathlib import Path

script_dir = Path(__file__).resolve().parent
parent_dir = str(script_dir.parent)
print(parent_dir)

def load_config(config_file=f'{parent_dir}/config/sim_config.json'):
    try:
        if not os.path.exists(config_file):
            raise FileNotFoundError
            
        with open(config_file, 'r', encoding='utf-8') as f:
            config = json.load(f)
            
        return config
        
    except json.JSONDecodeError as e:
        print(f"Failed to parse json file: {e}")
        return None
    except Exception as e:
        print(e)
        return None


config = load_config()

if config is not None:
    _env = config.get('environment')
    _robot = config.get('robot')
    _performance_mode = config.get('performance_mode')
    _physics_steps = config.get('physics_steps')
    # _render_steps = config.get('render_steps')
    _render_steps = 10
    _physics_dt = 1.0 / _physics_steps
    _render_dt = 1.0 / _render_steps
    _sync_with_realtime = config.get('sync_with_realtime')
    _sensor_settings = config.get('sensor_settings')
    _lidar_model = config.get('lidar_model')
    
    print(f"Environment: {_env}")
    print(f"Robot: {_robot}")
    print(f"Performance Mode: {_performance_mode}")
    print(f"Physics Steps: {_physics_steps}")
    print(f"Render Steps: {_render_steps}")
    print(f"Physics dt: {_physics_dt}")
    print(f"Render dt: {_render_dt}")
    print(f"Sync with Realtime: {_sync_with_realtime}")
    print(f"Sensor Settings: {_sensor_settings}")
    print(f"LiDAR Model: {_lidar_model}")

usdPath = parent_dir + defines.USD_PATH[_env]
robotPrimPath = f"/{_robot}"
if _robot == "turtlebot3_burger":
    robotPosition = defines.TURTLEBOT_POSITION[_env]
    sensorPackPrimPath = robotPrimPath + "/base_footprint/base_link/sensor_pack"
elif _robot == "unitree_g1":
    if _env == "Rivermark":
        robotPosition = [-0.5,0.0,6.5]
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
launch_config = defines.LAUNCH_CONFIG
launch_config["headless"] = True
launch_config["multi_gpu"] = False
launch_config["rtx_realtime_mgpu_enabled"] = False


app = SimulationApp(launch_config=launch_config)


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

app.update()


if _env=="Cave":  # Change stage lighting to camera lighting.
    action_registry = omni.kit.actions.core.get_action_registry()
    action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
    action.execute()
    app.update()


from isaacsim.core.api import World
world = World(stage_units_in_meters=1.0,physics_dt=_physics_dt,rendering_dt=_render_dt)


app.update()

## Robot

print_log(f"Creating robot at {robotPrimPath}...")
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
        stage.GetPrimAtPath(robotPrimPath).GetAttribute("xformOp:scale").Set((10,10,10))
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

## Dummy camera
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
    xform_api.SetRotate((90,0, -90), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    camera_prim.GetHorizontalApertureAttr().Set(21)
    camera_prim.GetVerticalApertureAttr().Set(16)
    camera_prim.GetProjectionAttr().Set("perspective")
    camera_prim.GetFocalLengthAttr().Set(24)
    camera_prim.GetFocusDistanceAttr().Set(400)


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

app.update()


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
        num_livox_parts = 4
        lidar_steps = [3,4,5,7,11]

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

        lidar_gate_paths = ["/Render/PostProcess/SDGPipeline/Isaac_PostProcessDispatchIsaacSimulationGate"]
        for _ in range(1,num_livox_parts):
            lidar_gate_paths.append(f"/Render/PostProcess/SDGPipeline/Isaac_0{_}_PostProcessDispatchIsaacSimulationGate",)
    
        for _ in range(num_livox_parts):
            og.Controller.attribute(lidar_gate_paths[_]+".inputs:step").set(lidar_steps[_])
            

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
        lidar_step_size = 1
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

## Configure shared memory
print_log("Setting shared memory...")
import sysv_ipc, struct

try:
    path = '/tmp/shared_data'
    fd = os.open(path, flags=os.O_CREAT)
    os.close(fd)
    key = 1000
except Exception as e:
    print(e)
    exit()

try:
    sem = sysv_ipc.Semaphore(key + 1, sysv_ipc.IPC_CREX, initial_value=1)
except sysv_ipc.ExistentialError:
    sem = sysv_ipc.Semaphore(key + 1)

if _sensor_settings["Camera2"]:
    print("Using 2 cameras. Second camera movement is not implemented.")

try:
    # cam1 : x, y, z, r, p, y
    # cam2 : x, y, z, r, p, y
    # 6*2* 4byte = 48 bytes
    shm_size = 48 if _sensor_settings["Camera2"] else 24
    shm = sysv_ipc.SharedMemory(key, size=shm_size, flags=sysv_ipc.IPC_CREAT, mode=0o644)
except sysv_ipc.ExistentialError:
    shm = sysv_ipc.SharedMemory(key)

## Array : Translation(x,y,z), Rotation(r,p,y)

camera1_prim = stage.GetPrimAtPath(camera1_prim_path)
from scipy.spatial.transform import Rotation as R


def write_shm():
    sem.acquire()
    try:
        data = []
        
        timeline = omni.timeline.get_timeline_interface()
        timecode = timeline.get_current_time() * timeline.get_time_codes_per_seconds()

        # 월드 변환 행렬 획득
        world_transform = omni.usd.get_world_transform_matrix(camera1_prim, timecode)

        # 위치와 회전 정보 추출
        translation = world_transform.ExtractTranslation()
        rotation_quat = world_transform.ExtractRotation().GetQuaternion()

        imaginary = rotation_quat.GetImaginary()
        r = R.from_quat([imaginary[0], imaginary[1], imaginary[2], rotation_quat.GetReal()])
        euler_angles = r.as_euler('xyz', degrees=True)
        data.extend(translation)
        data.extend(euler_angles)

        # print(f"writing data : {data}")
        if _sensor_settings["Camera2"]:
            shm.write(struct.pack('12f', *data))
        else:
            shm.write(struct.pack('6f', *data))
    finally:
        sem.release()


print_log("\n\n     Simulator Ready!\n")



world.reset()
world.stop()

from pynput import keyboard
#############################################
###                                       ###
###              4. Main Loop             ###
###                                       ###
#############################################
import numpy as np
if _robot == "turtlebot3_burger":
    my_controller.reset()
    print("[ Turtlebot Control Instructions ]")
    print(" ↑ : Increase linear velocity")
    print(" ↓ : Decrease linear velocity")
    print(" ← : Rotate Left")
    print(" → : Rotate Right")
    print(" S : Stop")

    # Setup Keyboard
    velocity = [0.0, 0.0]
    dv = 0.05
    dr = np.pi/36.0
    need_update_vel = True
    running = True

    _key_to_control = {
        keyboard.Key.up: [dv, 0.0],
        keyboard.Key.down: [-dv, 0.0],
        keyboard.Key.left: [0.0, dr],
        keyboard.Key.right: [0.0, -dr],
    }

    def on_key_press(key):
        global velocity, need_update_vel, running
        
        try:
            if key in _key_to_control:
                velocity[0] += _key_to_control[key][0]
                velocity[1] += _key_to_control[key][1]
                print(f"  Keyboard : {key} >> Set Velocity : {velocity}")
                need_update_vel = True
            elif hasattr(key, 'char') and key.char == 's':
                velocity = [0.0, 0.0]
                print(f"  Keyboard : {key.char} >> Set Velocity : {velocity}")
                need_update_vel = True
        except AttributeError:
            if key == keyboard.Key.esc:
                running = False
                return False 

    def on_key_release(key):
        pass


elif _robot == "unitree_g1":
    print("[ G1 Control Instructions ]")
    print(" ↑ : Move Forward")
    print(" ← : Rotate Left")
    print(" → : Rotate Right")
    print(" ↓ or None : Stop")

    _key_to_control = {
        keyboard.Key.up: [0.5, 0, 0],
        keyboard.Key.down: [0, 0, 0],
        keyboard.Key.left: [0, 0, 1],
        keyboard.Key.right: [0, 0, -1],
    }

    # Set up Keyboard
    def on_key_press(key):
        global base_command
        if key in _key_to_control:
            base_command = _key_to_control[key]
            print(f"Keyboard : {key}                    ")

    def on_key_release(key):
        global base_command
        if key in _key_to_control:
            base_command = _key_to_control[keyboard.Key.down]
        


else:
    os.system("pgrep -f rviz | xargs -r kill -15")
    raise NotImplementedError()

listener = keyboard.Listener(
        on_press=on_key_press,
        on_release=on_key_release
    )
listener.start()

# 메인 루프
tick = 0
need_update_vel = False

trace_prim = robotPrimPath+"/base_footprint" if _robot == "turtlebot3_burger" else robotPrimPath+"/g1_minimal/torso_link"
distance_calculator = DistanceCalculator(trace_prim)
print(f"Target tracing prim : {trace_prim}")

world.play()
while app.is_running():
    app.update()
    if tick != 0: tick = 0

    ## Real-time Syncronous Simulation
    while world.is_playing() and _sync_with_realtime:
        if tick == 0:
            if _robot == "unitree_g1":
                world.reset()
                g1.initialize()
                
                def on_physics_step(step_size):
                    if g1:
                        try:
                            g1.forward(step_size, base_command)
                        except Exception as e:
                            print(e)
                world.add_physics_callback("physics_step", callback_fn=on_physics_step)
            elif _robot == "turtlebot3_burger":
                world.reset()
                my_controller.reset()
            now = perf_counter()
            next_physics_time = now + _physics_dt
            next_render_time = now + _render_dt

        now = perf_counter()

        while now > next_physics_time:
            world.step(render=False)
            write_shm()
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
        if tick == 0:
            if _robot == "unitree_g1":
                world.reset()
                g1.initialize()
                
                def on_physics_step(step_size):
                    if g1:
                        try:
                            g1.forward(step_size, base_command)
                        except Exception as e:
                            print(e)
                world.add_physics_callback("physics_step", callback_fn=on_physics_step)
            elif _robot == "turtlebot3_burger":
                world.reset()
                my_controller.reset()
            now = perf_counter()


        world.step(render=False)
        write_shm()
        total_distance = distance_calculator.update_distance()

        if tick % _render_steps==0:
            world.render()
            print(f" Traveled distance: {total_distance:.3f} meters",end = "\r")

            if _sensor_settings["TfOdom"]: ros_tf_odom_graph.evaluate()
            
        if need_update_vel:
            need_update_vel = False
            if _robot == "turtlebot3_burger": 
                try:
                    robot_prim.apply_wheel_actions(my_controller.forward(command=velocity))
                except Exception as e:
                    print(e)
        tick += 1
        if tick > 1e8:
            tick = 1

world.stop()
app.close()
os.system("pgrep -f rviz | xargs -r kill -15")
exit()
