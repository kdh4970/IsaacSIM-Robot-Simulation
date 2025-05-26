from time import perf_counter
import omni
import numpy as np

class IntervalChecker:
    def __init__(self, text= None):
        self.text = text
        self.elapsed = None

    def __enter__(self):
        self.start = perf_counter()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        end = perf_counter()
        self.elapsed = end - self.start
        if self.text:print(f"During {self.text} : {1000 * self.elapsed:.3f}ms")


class DistanceCalculator:
    def __init__(self, robot_prim_path):
        self.robot_prim_path = robot_prim_path
        self.stage = omni.usd.get_context().get_stage()
        self.prev_position = None
        self.total_distance = 0.0
        self.prim = self.stage.GetPrimAtPath(self.robot_prim_path)

    def get_current_position(self):
        transform = omni.usd.get_world_transform_matrix(self.prim)
        translation = transform.ExtractTranslation()
        return np.array([translation[0], translation[1], translation[2]])

    def update_distance(self):
        current_pos = self.get_current_position()
        if self.prev_position is not None:
            distance = np.linalg.norm(current_pos - self.prev_position)
            self.total_distance += distance
        self.prev_position = current_pos
        return self.total_distance
