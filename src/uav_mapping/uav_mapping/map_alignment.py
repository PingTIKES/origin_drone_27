"""Fixed Gazebo ENU map alignment with PX4's local NWU odometry."""
import math

SPAWN_ENU = ((1.3, 9.4), (-1.3, 9.4), (1.3, 11.6), (-1.3, 11.6))


def map_to_odom(spawn, position):
    """Gazebo east/north = -PX4 NWU west/north, anchored at spawn."""
    north, west = position
    return (spawn[0] + west, spawn[1] - north, math.pi / 2)


class SpawnAlignment:
    def __init__(self, spawn):
        self.spawn = spawn
        self.transform = None

    def latch(self, position):
        if self.transform is None:
            self.transform = map_to_odom(self.spawn, position)
        return self.transform
