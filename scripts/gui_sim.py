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
    _render_steps = config.get('render_steps')
    _render_steps = 50
    _physics_dt = 1.0 / _physics_steps
    _render_dt = 1.0 / _render_steps
    # _sync_with_realtime = config.get('sync_with_realtime')
    _sync_with_realtime = True
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
launch_config["headless"] = False
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
usdPath = parent_dir + defines.USD_PATH[_env]

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

#############################################
###                                       ###
###        3. Sensors Configuration       ###
###                                       ###
#############################################

## Camera
print_log("\n\n     Configuring Sensors...\n")
if _sensor_settings["Camera"]:
    camera1_prim_path = "/Camera"
    
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
    camera2_prim_path = "/Camera2"
    cam2_traslation = Gf.Vec3d(-1.2, 0.0, 1.5)
    cam2_rotation = (45, 0, -90)
    camera_prim2 = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(camera2_prim_path, "Camera"))
    xform_api = UsdGeom.XformCommonAPI(camera_prim2)
    xform_api.SetTranslate(cam2_traslation) 
    xform_api.SetRotate(cam2_rotation, UsdGeom.XformCommonAPI.RotationOrderXYZ)
    camera_prim2.GetHorizontalApertureAttr().Set(21)
    camera_prim2.GetVerticalApertureAttr().Set(16)
    camera_prim2.GetProjectionAttr().Set("perspective")
    camera_prim2.GetFocalLengthAttr().Set(24)
    camera_prim2.GetFocusDistanceAttr().Set(400)

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



# if _sensor_settings["Lidar"]:
#     if _env == "Rivermark":
#         lidar_offset = (-0.03, 0, 0.21)
#     else:
#         lidar_offset = (-0.07, 0, 0.4)
#     if _robot == "turtlebot3_burger":
#         lidar_prim_path = "/Lidar1"
#     else:
#         lidar_prim_path = "/Lidar"


#     if _lidar_model == "Livox_MID360":
#         lidar_prims=[]
#         lidar_render_products = []
#         writers = []
#         num_livox_parts = 5
#         lidar_steps = [11,12,13,14,15]

#         # Create parts of MID360
#         for _ in range(num_livox_parts):
#             _, lidar_prim = omni.kit.commands.execute(
#                 "IsaacSensorCreateRtxLidar",
#                 path=lidar_prim_path,
#                 parent=None,
#                 config="MID360_" + str(_+1),
#                 # translation=(-0.03, 0, 0.18), # this for 1x scale
#                 translation=lidar_offset, # this for 2x scale
#                 # translation=(-0.1, 0, 0.57), # this for 3x scale
#                 orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
#             )
#             lidar_prims.append(lidar_prim)

#             # RTX sensors are cameras and must be assigned to their own render product
#             lidar_render_product = rep.create.render_product(lidar_prim.GetPath(), [1, 1], name="Isaac")
#             lidar_render_products.append(lidar_render_product)

#             # Create Point cloud publisher pipeline in the post process graph
#             writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
#             writer.initialize(topicName="point_cloud", frameId="lidar_link")
#             writer.attach([lidar_render_product])
#             writers.append(writer)
#             if _sensor_settings["DebugLidar"]:
#                 # Create the debug draw pipeline in the post process graph
#                 writerd = rep.writers.get("RtxLidar" + "DebugDrawPointCloud")
#                 writerd.attach([lidar_render_product])
#                 writers.append(writerd)

#         lidar_gate_paths = [
#             "/Render/PostProcess/SDGPipeline/Isaac_PostProcessDispatchIsaacSimulationGate",
#             "/Render/PostProcess/SDGPipeline/Isaac_01_PostProcessDispatchIsaacSimulationGate",
#             "/Render/PostProcess/SDGPipeline/Isaac_02_PostProcessDispatchIsaacSimulationGate",
#             "/Render/PostProcess/SDGPipeline/Isaac_03_PostProcessDispatchIsaacSimulationGate",
#             "/Render/PostProcess/SDGPipeline/Isaac_04_PostProcessDispatchIsaacSimulationGate"
#             ]
#         for i in range(num_livox_parts):
#             og.Controller.attribute(lidar_gate_paths[i]+".inputs:step").set(lidar_steps[_])
            

#     else:
#         _, lidar_prim = omni.kit.commands.execute(
#             "IsaacSensorCreateRtxLidar",
#             path=lidar_prim_path,
#             parent=None,
#             config=_lidar_model,
#             translation=lidar_offset,
#             orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
#         )
#         # RTX sensors are cameras and must be assigned to their own render product
#         lidar_render_product = rep.create.render_product(lidar_prim.GetPath(), [1, 1], name="Isaac")
#         # Create Point cloud publisher pipeline in the post process graph
#         writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
#         writer.initialize(topicName="point_cloud", frameId="lidar_link")
#         writer.attach([lidar_render_product])

#         if _sensor_settings["DebugLidar"]:
#             # Create the debug draw pipeline in the post process graph
#             writer1 = rep.writers.get("RtxLidar" + "DebugDrawPointCloud")
#             writer1.attach([lidar_render_product])

#         lidar_gate_path = "/Render/PostProcess/SDGPipeline/Isaac_PostProcessDispatchIsaacSimulationGate"
#         lidar_step_size = 3
#         og.Controller.attribute(lidar_gate_path+".inputs:step").set(lidar_step_size)


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
_settings.set_float("/rtx/sceneDb/ambientLightIntensity", 0.2)
# omni.kit.commands.execute('ChangeSetting',
# 	path='/rtx/sceneDb/ambientLightColor',
# 	value=[1.0, 1.0, 1.0])



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


print_log("Setting shared memory...")
import sysv_ipc, struct                              

path = '/tmp/shared_data'
if not os.path.exists(path):
    assert f"Could not find {path}. You should execute headleess_sim first."
    exit()

key = 1000

try:
    sem = sysv_ipc.Semaphore(key + 1)
except sysv_ipc.ExistentialError:
    print(f"Could not find semaphore. Key : {key}")
    exit()

try:
    shm = sysv_ipc.SharedMemory(key)
except sysv_ipc.ExistentialError:
    print(f"Could not find shared memory. Key : {key}")
    exit()

target_prim = stage.GetPrimAtPath(camera1_prim_path)
if _sensor_settings["Camera2"]:
    shm_size = 48
    print("Using 2 cameras. Second camera movement is not implemented.")
else:
    shm_size = 24

def read_shm():
    sem.acquire()
    try:
        raw_data = shm.read(shm_size) 
        if _sensor_settings["Camera2"]:
            data = list(struct.unpack('12f', raw_data))
        else:
            data = list(struct.unpack('6f', raw_data))
        # print(f"reading data : {data}")
        xform_api.SetTranslate(Gf.Vec3d(*data[0:3]))
        xform_api.SetRotate(tuple(data[3:]), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    finally:
        sem.release()

print_log("\n\n     Simulator Ready!\n")
world.reset()
world.stop()

#############################################
###                                       ###
###              4. Main Loop             ###
###                                       ###
#############################################

# 메인 루프
tick = 0
reset_needed = False
need_update_vel = False

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
            reset_needed = False

        now = perf_counter()
        if tick == 0:
            next_render_time = now + _render_dt

        
        if now > next_render_time:
            read_shm()
            world.render()
            next_render_time += _render_dt
            
        tick += 1
        if tick > 1e8:
            tick = 1

    ## Best Performance Simulation
    while world.is_playing() and not _sync_with_realtime:
        if reset_needed:
            world.reset()
            reset_needed = False
        read_shm()
        world.render()
        
        tick += 1
        if tick > 1e8:
            tick = 1

world.stop()
app.close()
os.system("pgrep -f rviz | xargs -r kill -15")
exit()
