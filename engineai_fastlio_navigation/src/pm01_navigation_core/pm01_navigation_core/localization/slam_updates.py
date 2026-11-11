"""Versioned SLAM snapshots; independent of ROS and DDS message arrival order."""
from collections import OrderedDict
from dataclasses import dataclass
import json
import math


@dataclass(frozen=True)
class SlamUpdate:
    epoch: str
    revision: int
    stamp_ns: int
    accepted: bool
    event: str
    reason: str
    pose: tuple = ()
    odom: tuple = ()
    optimization_revision: int = 0
    anchor_id: int = -1

    @classmethod
    def parse(cls, data):
        data = json.loads(data) if isinstance(data, str) else data
        if data.get('schema') != 1 or data.get('backend') != 'pm01-slam-2.6.10':
            raise ValueError('Unsupported SLAM backend metadata')
        result = cls(str(data['epoch']), int(data['revision']), int(data['stamp_ns']),
                     data['accepted'], str(data['event']), str(data['reason']),
                     tuple(data.get('pose', ())), tuple(data.get('odom', ())),
                     int(data.get('optimization_revision', 0)), int(data.get('anchor_id', -1)))
        if (not result.epoch or result.revision < 0 or result.stamp_ns < 0 or
                not isinstance(result.accepted, bool) or
                not 0 <= result.optimization_revision <= result.revision):
            raise ValueError('Invalid SLAM revision')
        if result.accepted and (len(result.pose) != 3 or len(result.odom) != 3 or
                                not all(math.isfinite(v) for v in result.pose+result.odom)):
            raise ValueError('Invalid accepted SLAM poses')
        return result

    @property
    def anchor_pose(self):
        return self.pose

    @property
    def correction(self):
        x, y, yaw = self.pose
        ox, oy, oyaw = self.odom
        a = math.atan2(math.sin(yaw-oyaw), math.cos(yaw-oyaw))
        c, s = math.cos(a), math.sin(a)
        return x-c*ox+s*oy, y-s*ox-c*oy, 0., a


class SnapshotInbox:
    """Join a bounded map queue with metadata by exact integer source time.

    An epoch change requires a restart; never merge odometry histories from
    separate backend sessions. Maps without metadata are not adopted.
    """
    def __init__(self, capacity=32):
        self.capacity = capacity
        self.maps = OrderedDict()
        self.updates = OrderedDict()
        self.epoch = None
        self.revision = -1

    def _put(self, target, stamp, value):
        target[stamp] = value
        while len(target) > self.capacity:
            target.popitem(last=False)
        ready = []
        for key in self.maps.keys() & self.updates.keys():
            update = self.updates[key]
            if self.epoch is not None and update.epoch != self.epoch:
                raise ValueError('SLAM epoch changed; restart the mapping session')
            if update.revision > self.revision:
                ready.append((update.revision, key, update))
        if not ready:
            return None
        _, key, update = max(ready)
        grid = self.maps[key]
        self.epoch, self.revision = update.epoch, update.revision
        for stamp in list(self.updates):
            if self.updates[stamp].revision <= self.revision:
                self.updates.pop(stamp)
                self.maps.pop(stamp, None)
        return grid, update

    def map(self, stamp_ns, grid):
        return self._put(self.maps, stamp_ns, grid)

    def update(self, update):
        if not update.accepted:
            return None
        return self._put(self.updates, update.stamp_ns, update)
