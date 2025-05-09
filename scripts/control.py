import defines
from isaacsim import SimulationApp
app = SimulationApp(launch_config=defines.LAUNCH_CONFIG)
app.update()
from isaacsim.core.api import World
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.storage.native import get_assets_root_path
import numpy as np
asset_path = get_assets_root_path() + "/Isaac/Robots/Turtlebot/turtlebot3_burger.usd"
my_world = World(stage_units_in_meters=1.0)
turtlebot = my_world.scene.add(
    WheeledRobot(
        prim_path="/scene/turtlebot",
        name="my_turtlebot",
        wheel_dof_names=["wheel_left_joint", "wheel_right_joint"],
        create_robot=True,
        usd_path=asset_path,
        position=np.array([0, 0.0, 0.1]),
    )
)
my_world.scene.add_default_ground_plane()
my_controller = DifferentialController(name="simple_control", wheel_radius=0.025, wheel_base=0.16,max_linear_speed=1.5,max_angular_speed=1.0)
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
print("Control Instructions:")
print("  W: Move Forward")
print("  A: Rotate Left")
print("  D: Rotate Right")
print("  S: Stop")


reset_needed = False
while app.is_running():
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    app.update()
    while my_world.is_playing():
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            reset_needed = False
        my_world.step(render=True)
        # 키 입력 받아서 제어
        if key_state['w']:
            turtlebot.apply_wheel_actions(my_controller.forward(command=[0.3, 0]))
            print("Forward:", turtlebot.get_linear_velocity())
        elif key_state['a']:
            turtlebot.apply_wheel_actions(my_controller.forward(command=[0.0, np.pi / 6]))
            print("Rotate Left:", turtlebot.get_angular_velocity())
        elif key_state['d']:
            turtlebot.apply_wheel_actions(my_controller.forward(command=[0.0, -np.pi / 6]))
            print("Rotate Right:", turtlebot.get_angular_velocity())
        elif key_state['s']:
            turtlebot.apply_wheel_actions(my_controller.forward(command=[0.0, 0.0]))
            print("Stop")
        elif key_state['x']:
            turtlebot.apply_wheel_actions(my_controller.forward(command=[-0.3, 0.0]))
            print("Backward:", turtlebot.get_linear_velocity())
        elif key_state['q']:
            my_world.stop()
            break

