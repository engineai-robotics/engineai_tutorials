"""Planar initial-pose alignment. No map matching or odometry reset is implied."""
from dataclasses import dataclass
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float
    z: float = 0.

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.x, self.y, self.z, self.yaw)):
            raise ValueError('Pose contains non-finite values')


def yaw_from_quaternion(x, y, z, w):
    values = (x, y, z, w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Quaternion contains non-finite values')
    norm = math.hypot(*values)
    if norm < 1e-9:
        raise ValueError('Zero quaternion')
    x, y, z, w = (v/norm for v in values)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def compose(parent, child):
    c, s = math.cos(parent.yaw), math.sin(parent.yaw)
    return Pose(parent.x+c*child.x-s*child.y,
                parent.y+s*child.x+c*child.y, wrap(parent.yaw+child.yaw),
                parent.z+child.z)


def map_to_odom(map_base, odom_base):
    """T_map_odom = T_map_base * inverse(T_odom_base), at the SAME instant."""
    angle = wrap(map_base.yaw-odom_base.yaw)
    c, s = math.cos(angle), math.sin(angle)
    return Pose(map_base.x-c*odom_base.x+s*odom_base.y,
                map_base.y-s*odom_base.x-c*odom_base.y, angle,
                map_base.z-odom_base.z)
