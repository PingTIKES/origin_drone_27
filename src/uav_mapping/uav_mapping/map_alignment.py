"""Fixed simulation spawn alignment between the field map and PX4 odometry."""
import math


def map_to_odom(spawn, position, yaw):
    """Align the initial odometry pose with the known Gazebo spawn pose."""
    c, s = math.cos(yaw), math.sin(yaw)
    x, y = position
    return (spawn[0] - c*x - s*y, spawn[1] + s*x - c*y, -yaw)


class SpawnAlignment:
    def __init__(self, spawn):
        self.spawn = spawn
        self.transform = None

    def latch(self, position, yaw):
        if self.transform is None:
            self.transform = map_to_odom(self.spawn, position, yaw)
        return self.transform
