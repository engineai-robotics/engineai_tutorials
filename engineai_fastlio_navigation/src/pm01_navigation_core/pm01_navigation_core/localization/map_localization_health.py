"""Separate measured local observations from map-frame localization and motion."""
import math


def local_observation_support(level, reason, values):
    """Qualify a measured LIO update, without claiming six-DoF observability.

    A horizontal plane can still be measured when yaw or planar translation is
    weakly constrained. Its geometric checks and map matching remain separate
    requirements; an IMU-only prediction or rejected correction is not accepted.
    """
    if ((level, reason) not in ((0, 'geometry_supported'), (1, 'weak_geometry')) or
            values.get('lidar_update_applied') != 'true' or
            values.get('lidar_correction_rejected') != 'false' or
            values.get('recovery_candidate_rejected') != 'false'):
        return 'unsupported'
    try:
        speed = float(values['velocity_norm_mps'])
    except (KeyError, TypeError, ValueError):
        return 'unsupported'
    # Preserve the estimator's existing abnormal-speed guard, independently of
    # motion_supported, whose full-rank condition is too strict for observations.
    if not math.isfinite(speed) or not 0 <= speed < 2.:
        return 'unsupported'
    return 'tracking' if level == 0 else 'weak_geometry_observation'


def observation_health_current(health, now, received_now, max_age, *, future_tolerance=.05):
    """Check source and reception clocks without renewing either timestamp.

    Measurement sources retain the .05 s bound. Callers handling localization
    status headers may allow .2 s for asynchronous clock callbacks, independently
    of all real LIO, cloud and ground measurement checks.
    """
    return bool(health is not None and health[2] and
                -future_tolerance <= now-health[1] <= max_age and
                0 <= received_now-health[0] <= max_age)


class MotionSinceMatch:
    """Measure a path at a fixed spatial resolution, not at the odometry rate.

    Independent anchors retain slow translation and rotation. Sub-resolution
    motion remains in the reported tail, but stationary jitter is not summed
    repeatedly. This is a diagnostic distance, not a replacement odometry pose.
    """

    def __init__(self, translation_resolution=.02, rotation_resolution=.01):
        if not all(math.isfinite(v) and v > 0 for v in
                   (translation_resolution, rotation_resolution)):
            raise ValueError('Motion resolutions must be finite and positive')
        self.translation_resolution = translation_resolution
        self.rotation_resolution = rotation_resolution
        self.reset(None)

    def reset(self, pose):
        self.position = None if pose is None else pose[:2]
        self.heading = None if pose is None else pose[2]
        self.path = self.rotation = 0.

    def update(self, pose):
        if self.position is None:
            self.reset(pose)
        distance = math.hypot(pose[0]-self.position[0], pose[1]-self.position[1])
        angle = abs(math.atan2(math.sin(pose[2]-self.heading), math.cos(pose[2]-self.heading)))
        if distance >= self.translation_resolution:
            self.path += distance
            self.position = pose[:2]
            distance = 0.
        if angle >= self.rotation_resolution:
            self.rotation += angle
            self.heading = pose[2]
            angle = 0.
        return self.path+distance, self.rotation+angle


def localization_support(topic, level, reason, values):
    if topic == '/pm01/localization_status':
        # Continue storing valid local observations in odom while the SLAM
        # adapter holds a bounded, previously accepted alignment. This remains
        # a degraded localization state and is not a new matching observation.
        if (level == 1 and values.get('mode') == '"slam"' and
                values.get('held_alignment_usable') == 'true'):
            return 'held_slam_alignment'
        return ('map_matched' if level == 0 and values.get('matched') == 'true'
                and values.get('mode') in ('"slam"', '"amcl"') else 'unsupported')
    return 'unsupported'
