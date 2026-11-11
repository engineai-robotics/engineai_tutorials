"""Geometry for a gravity-aligned virtual scan; no ROS or robot dependency."""
import json
import math

import numpy as np
from pm01_navigation_core.geometry import rotation_xyzw as rotation
from pm01_navigation_core.pointcloud import read_xyz


def yaw(x, y, z, w):
    r = rotation(x, y, z, w)
    return math.atan2(r[1, 0], r[0, 0])


def ranges(points, *, beams=720, range_min=.6, range_max=12.):
    """Nearest actual hit per azimuth; unobserved bins remain NaN.

    In particular, missing points do not prove the full ray free. The caller
    supplies points already transformed to a yaw-only, gravity-aligned frame.
    """
    if beams < 8 or not math.isfinite(range_min+range_max) or not 0 < range_min < range_max:
        raise ValueError('Invalid virtual scan limits')
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    distance = np.linalg.norm(points[:, :2], axis=1)
    valid = np.isfinite(points).all(axis=1) & (distance >= range_min) & (distance <= range_max)
    xy, distance = points[valid, :2], distance[valid]
    index = np.floor((np.arctan2(xy[:, 1], xy[:, 0])+math.pi)/(2*math.pi/beams)).astype(int) % beams
    result = np.full(beams, np.inf, dtype=np.float32)
    np.minimum.at(result, index, distance)
    result[~np.isfinite(result)] = np.nan
    return result


class Keyframes:
    """Bound mapping graph growth while retaining in-place turns.

    Humble SLAM Toolbox's front gate tests translation even for pure rotation.
    Its distance gate is disabled when this selector supplies mapping scans.
    A short bootstrap uses distinct incoming scans, never a replayed cache.
    """
    def __init__(self, distance=.2, angle=.15, bootstrap=5):
        if not math.isfinite(distance+angle) or distance <= 0 or angle <= 0 or bootstrap < 1:
            raise ValueError('Invalid keyframe thresholds')
        self.distance, self.angle, self.remaining = distance, angle, bootstrap
        self.last = None

    def accept(self, x, y, heading):
        if not np.isfinite([x, y, heading]).all(): return False
        turn = (0. if self.last is None else abs(math.atan2(
            math.sin(heading-self.last[2]), math.cos(heading-self.last[2]))))
        take = (self.remaining > 0 or self.last is None or
                math.hypot(x-self.last[0], y-self.last[1]) >= self.distance or turn >= self.angle)
        if take:
            self.last = x, y, heading
            self.remaining = max(0, self.remaining-1)
        return take


class MappingScans:
    """Spatial keyframes normally; fresh bounded retries while SLAM recovers.

    A retry is only a candidate observation. The SLAM backend owns validation,
    recovery counting and graph insertion. No scan is cached or restamped here.
    """
    def __init__(self, distance=.2, angle=.15, retry_period=1., bootstrap=5):
        if not math.isfinite(retry_period) or retry_period <= 0:
            raise ValueError('Invalid mapping retry period')
        self.keyframes = Keyframes(distance, angle, bootstrap)
        self.retry_ns = round(retry_period*1e9)
        # Bootstrap frames can precede the backend subscription. Keep bounded
        # fresh retries until it acknowledges an accepted observation.
        self.recovering = True
        self.request_after_stamp = -1
        self.epoch, self.retired_epochs = None, set()
        self.update_stamp = self.last_seen_stamp = self.last_sent_stamp = -1

    def request_recovery(self):
        # A late acceptance from before the input interruption cannot release
        # this request. Only a newly observed and accepted scan can do so.
        self.request_after_stamp = max(self.request_after_stamp, self.last_seen_stamp)
        self.recovering = True

    def observe_match(self, data):
        try:
            data = json.loads(data) if isinstance(data, str) else data
            epoch, stamp = data['epoch'], data['stamp_ns']
            accepted = data['accepted']
            if (data.get('schema') != 1 or data.get('backend') != 'pm01-slam-2.6.10' or
                    not isinstance(epoch, str) or not epoch or
                    not isinstance(stamp, int) or isinstance(stamp, bool) or stamp < 0 or
                    not isinstance(accepted, bool) or epoch in self.retired_epochs):
                return False
        except (ValueError, TypeError, KeyError):
            return False
        if epoch != self.epoch:
            if self.epoch is not None:
                self.retired_epochs.add(self.epoch)
                self.request_after_stamp = -1
            self.epoch, self.update_stamp = epoch, -1
        if stamp <= self.update_stamp:
            return False
        self.update_stamp = stamp
        if not accepted:
            self.recovering = True
        elif stamp > self.request_after_stamp:
            self.recovering = False
        return True

    def select(self, x, y, heading, stamp_ns):
        if (stamp_ns <= self.last_seen_stamp or
                not all(math.isfinite(v) for v in (x, y, heading))):
            return ''
        self.last_seen_stamp = stamp_ns
        spatial = self.keyframes.accept(x, y, heading)
        retry = self.recovering and stamp_ns-self.last_sent_stamp >= self.retry_ns
        if spatial or retry:
            self.last_sent_stamp = stamp_ns
            return 'keyframe' if spatial else 'recovery_observation'
        return ''
