"""Automatic ground calibration, bounded tracking and shared-model validation."""
from collections import deque
from dataclasses import replace
import json
import math
import time

import numpy as np

from pm01_navigation_core.ground.flat_ground import GroundPlane, estimate_ground


def height_at(plane, xy):
    return plane.a*xy[0]+plane.b*xy[1]+plane.c


def normal_angle(a, b):
    na, nb = np.array([-a.a, -a.b, 1.]), np.array([-b.a, -b.b, 1.])
    return math.acos(float(np.clip(na@nb/(np.linalg.norm(na)*np.linalg.norm(nb)), -1., 1.)))


def bootstrap_ground(points, centre, sensor, min_height=.7, max_height=1.5):
    """Find the lowest supported horizontal plane in the mounting-height band.

    The height band is a physical mounting constraint, not a remembered map Z.
    A histogram supplies candidate priors; each still needs the normal ground
    estimator's measured inliers, tile coverage, tilt and residual checks.
    """
    points = np.asarray(points, float).reshape(-1, 3)
    sensor, centre = np.asarray(sensor, float), np.asarray(centre, float)[:2]
    distance = np.linalg.norm(points[:, :2]-centre, axis=1)
    valid = (np.isfinite(points).all(axis=1) & (distance >= .5) & (distance <= 5.) &
             (points[:, 2] >= sensor[2]-max_height-.05) &
             (points[:, 2] <= sensor[2]-min_height+.05))
    z = points[valid, 2]
    if len(z) < 100:
        return None
    edges = np.arange(sensor[2]-max_height-.06, sensor[2]-min_height+.081, .02)
    counts, edges = np.histogram(z, bins=edges)
    seeds = []
    for i in np.argsort(-counts, kind='stable'):
        if counts[i] < 10:
            break
        seed = .5*(edges[i]+edges[i+1])
        if all(abs(seed-old) >= .06 for old in seeds):
            seeds.append(float(seed))
        if len(seeds) == 6:
            break
    fits = []
    for seed in seeds:
        plane = estimate_ground(points, centre, seed)
        if plane is not None and min_height <= sensor[2]-height_at(plane, sensor[:2]) <= max_height:
            fits.append(plane)
    if not fits:
        return None
    return min(fits, key=lambda p: height_at(p, centre))


class GroundTracker:
    """Calibrate across stationary frames; never silently rebase after a jump."""
    def __init__(self, *, min_height=.7, max_height=1.5, samples=3, calibration_span=.4,
                 max_age=.75, max_height_rate=.02, max_anchor_offset=.08):
        limits = [min_height, max_height, calibration_span, max_age,
                  max_height_rate, max_anchor_offset]
        if (not np.isfinite(limits).all() or min_height <= 0 or max_height <= min_height or
                samples < 3 or min(limits[2:]) <= 0):
            raise ValueError('Invalid automatic ground limits')
        self.min_height, self.max_height = min_height, max_height
        self.samples, self.calibration_span, self.max_age = samples, calibration_span, max_age
        self.max_height_rate, self.max_anchor_offset = max_height_rate, max_anchor_offset
        self.anchor = self.plane = None
        self.plane_stamp = self.last_stamp = -math.inf
        self.candidates = []
        self.reason, self.revision = 'calibrating', 0
        self._suspended = False

    def suspend(self, reason):
        # Keep an established anchor: recovery cannot relabel a large LIO jump
        # as a new floor. A new navigation session creates a new tracker.
        self._suspended = True
        self.candidates.clear()
        self.reason = reason

    def update(self, points, centre, sensor, stamp):
        if not math.isfinite(stamp) or stamp <= self.last_stamp:
            return None
        self.last_stamp = stamp
        centre, sensor = np.asarray(centre, float)[:2], np.asarray(sensor, float)
        if not np.isfinite([*centre, *sensor]).all():
            self.suspend('invalid_sensor_pose')
            return None
        self._suspended = False
        if self.anchor is None:
            plane = bootstrap_ground(points, centre, sensor, self.min_height, self.max_height)
            if plane is None:
                self.candidates.clear(); self.reason = 'calibrating_no_ground'
                return None
            if self.candidates:
                first = self.candidates[0]
                if (np.linalg.norm(sensor-first[2]) > .08 or
                        abs(height_at(plane, centre)-height_at(first[1], centre)) > .02 or
                        normal_angle(plane, first[1]) > math.radians(1.5)):
                    self.candidates.clear()
            self.candidates.append((stamp, plane, sensor.copy()))
            self.candidates = self.candidates[-10:]
            self.reason = 'calibrating'
            if (len(self.candidates) < self.samples or
                    stamp-self.candidates[0][0] < self.calibration_span):
                return None
            # Use the most recent measured plane, rather than blending nearby
            # elevated surfaces into an artificial intermediate floor.
            self.anchor = self.plane = plane
        else:
            plane = estimate_ground(points, centre, height_at(self.plane, centre))
            if plane is None:
                self.reason = 'ground_unavailable'
                return self.current(stamp)
            dt = min(max(stamp-self.plane_stamp, 0.), self.max_age)
            delta = abs(height_at(plane, centre)-height_at(self.plane, centre))
            if (delta > .012+self.max_height_rate*dt or
                    abs(height_at(plane, centre)-height_at(self.anchor, centre)) > self.max_anchor_offset or
                    normal_angle(plane, self.plane) > math.radians(1.5) or
                    normal_angle(plane, self.anchor) > math.radians(3.) or
                    not self.min_height <= sensor[2]-height_at(plane, sensor[:2]) <= self.max_height):
                self.reason = 'ground_change_rejected'
                return self.current(stamp)
            self.plane = plane
        self.plane_stamp = stamp
        self.reason = 'tracking'
        self.revision += 1
        return self.plane

    def current(self, now):
        if (self._suspended or self.plane is None or
                not -.05 <= now-self.plane_stamp <= self.max_age):
            return None
        return self.plane


def transform_plane(plane, matrix, translation):
    """Transform n.p+d=0 to another frame, retaining a ground-relative model."""
    normal = np.asarray(matrix)@np.array([-plane.a, -plane.b, 1.])
    d = -plane.c-float(normal@np.asarray(translation))
    if normal[2] <= 0 or math.hypot(*normal[:2])/normal[2] > math.tan(math.radians(5)):
        raise ValueError('Transformed ground is not horizontal')
    return replace(plane, a=float(-normal[0]/normal[2]), b=float(-normal[1]/normal[2]),
                   c=float(-d/normal[2]))


class GroundInbox:
    """Bounded, stamped history shared by the map and obstacle consumers."""
    def __init__(self, max_age=.75):
        self.max_age, self.history = max_age, deque(maxlen=20)
        self.packet_stamp = self.received = -math.inf
        self.valid = False
        self.epoch = None

    def receive(self, packet_stamp, frame, level, values, received=None):
        received = time.monotonic() if received is None else received
        if not math.isfinite(packet_stamp) or packet_stamp < self.packet_stamp:
            return
        self.packet_stamp, self.received = packet_stamp, received
        self.valid = False
        try:
            epoch = json.loads(values['epoch'])
            if not isinstance(epoch, str) or not epoch:
                raise ValueError('Missing model epoch')
            if epoch != self.epoch:
                self.history.clear(); self.epoch = epoch
            if level != 0:
                self.history.clear()
                return
            p = json.loads(values['ground_plane'])
            plane = GroundPlane(p['a'], p['b'], p['c'], p['inliers'], p['candidates'], p['rms_m'])
            stamp = float(json.loads(values['plane_stamp']))
            model_id = json.loads(values['model_id'])
            if (not frame or not isinstance(model_id, str) or not model_id or
                    not np.isfinite([plane.a, plane.b, plane.c, plane.rms, stamp]).all() or
                    not -.05 <= packet_stamp-stamp <= self.max_age or
                    plane.inliers < 100 or plane.candidates < plane.inliers or not 0 <= plane.rms <= .01 or
                    math.hypot(plane.a, plane.b) > math.tan(math.radians(5))):
                raise ValueError('Invalid ground model')
            self.history.append((stamp, plane, frame, model_id))
            self.valid = True
        except (ValueError, TypeError, KeyError, OverflowError):
            self.history.clear()

    def get(self, cloud_stamp, now, received_now=None):
        received_now = time.monotonic() if received_now is None else received_now
        if (not self.valid or not -.05 <= now-self.packet_stamp <= self.max_age or
                not 0 <= received_now-self.received <= self.max_age):
            return None
        for stamp, plane, frame, model_id in reversed(self.history):
            if (not -.05 <= now-stamp <= self.max_age or
                    not -.05 <= cloud_stamp-stamp <= self.max_age):
                continue
            return plane, stamp, frame, model_id
        return None
