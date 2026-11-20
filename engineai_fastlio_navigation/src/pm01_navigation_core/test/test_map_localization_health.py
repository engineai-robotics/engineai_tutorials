"""Local measured observations and global localization have different eligibility."""
from pathlib import Path
import math
import sys

import pytest

from pm01_navigation_core.localization.map_localization_health import (
    local_observation_support, localization_support, observation_health_current, MotionSinceMatch)


def measured_lio():
    return {'pose_rank': '5', 'motion_supported': 'false', 'lidar_update_applied': 'true',
        'lidar_correction_rejected': 'false', 'recovery_candidate_rejected': 'false',
        'velocity_norm_mps': '0.2'}


def test_weak_measured_update_allows_observation_without_claiming_supported_motion():
    values = measured_lio()
    assert local_observation_support(1, 'weak_geometry', values) == 'weak_geometry_observation'
    assert localization_support('/lio/status', 1, 'weak_geometry', values) == 'unsupported'
    assert local_observation_support(0, 'geometry_supported', values) == 'tracking'
    # Local observations cannot manufacture global map alignment.
    assert localization_support('/pm01/localization_status', 1, 'recovering',
                                {'mode': '"slam"', 'matched': 'false'}) == 'unsupported'


@pytest.mark.parametrize('key,value', [('lidar_update_applied', 'false'),
    ('lidar_correction_rejected', 'true'), ('recovery_candidate_rejected', 'true'),
    ('velocity_norm_mps', 'nan'), ('velocity_norm_mps', 'inf'),
    ('velocity_norm_mps', '-0.1'), ('velocity_norm_mps', '2.0')])
def test_predictions_rejected_corrections_and_abnormal_motion_cannot_supply_observations(key, value):
    values = measured_lio(); values[key] = value
    assert local_observation_support(1, 'weak_geometry', values) == 'unsupported'


@pytest.mark.parametrize('key', ['lidar_update_applied', 'lidar_correction_rejected',
                               'recovery_candidate_rejected', 'velocity_norm_mps'])
def test_incomplete_local_diagnostics_fail_closed(key):
    values = measured_lio(); del values[key]
    assert local_observation_support(1, 'weak_geometry', values) == 'unsupported'


@pytest.mark.parametrize('level', [2, 3])
def test_failed_updates_cannot_supply_observations(level):
    assert local_observation_support(level, 'unavailable', measured_lio()) == 'unsupported'


def test_diagnostic_wording_does_not_override_explicit_measurement_flags():
    assert local_observation_support(1, 'imu_packets_recovered', measured_lio()) == 'weak_geometry_observation'
    values=measured_lio();values['lidar_update_applied']='false'
    assert local_observation_support(0, 'geometry_supported', values) == 'unsupported'


def test_health_requires_fresh_measurement_and_reception_without_renewing_source_time():
    assert observation_health_current((20., 10_000_000_000, True), 10_200_000_000, 20.2, .75)
    assert not observation_health_current((20., 10_000_000_000, True), 11_000_000_000, 20.2, .75)
    assert not observation_health_current((20., 10_000_000_000, True), 10_200_000_000, 21., .75)
    assert not observation_health_current((20., 10_000_000_000, True), 9_800_000_000, 20.2, .75)
    assert not observation_health_current((20., 10_000_000_000, False), 10_200_000_000, 20.2, .75)
    assert not observation_health_current(None, 10_200_000_000, 20.2, .75)


def test_only_explicit_status_metadata_allows_clock_callback_phase_difference():
    status = (20., 10_100_000_000, True)
    assert observation_health_current(status, 10_000_000_000, 20.05, .75, future_tolerance=.2)
    assert not observation_health_current(status, 10_000_000_000, 20.05, .75)
    assert not observation_health_current(status, 11_000_000_000, 20.05, .75, future_tolerance=.2)
    assert not observation_health_current(status, 10_000_000_000, 21., .75, future_tolerance=.2)
    assert not observation_health_current((20., 10_300_000_000, True), 10_000_000_000, 20.05, .75, future_tolerance=.2)


@pytest.mark.parametrize('topic', ['/lio/status', '/alignment_status', '/pm01/manual_alignment_status', '/unknown/status'])
@pytest.mark.parametrize('level', [0, 1])
def test_unsupported_localization_sources_are_rejected(topic, level):
    assert localization_support(topic, level, 'tracking', {'aligned': 'true'}) == 'unsupported'


def test_held_slam_alignment_supports_local_evidence_without_claiming_a_match():
    values={'mode':'"slam"','matched':'false','held_alignment_usable':'true'}
    assert localization_support('/pm01/localization_status',1,'recovering',values)=='held_slam_alignment'
    values['held_alignment_usable']='false'
    assert localization_support('/pm01/localization_status',1,'recovering',values)=='unsupported'
    values.update(mode='"amcl"',held_alignment_usable='true')
    assert localization_support('/pm01/localization_status',1,'recovering',values)=='unsupported'


def test_stationary_noise_does_not_accumulate_with_sample_rate_or_duration():
    for hz in (10, 100):
        motion = MotionSinceMatch()
        motion.reset((0., 0., 0.))
        for i in range(600*hz):
            travel, turn = motion.update((.003*math.sin(i*.7), .003*math.cos(i*.9), .002*math.sin(i)))
            assert travel < .005 and turn < .003


def test_slow_motion_and_reversals_remain_visible_between_anchors():
    motion = MotionSinceMatch()
    motion.reset((0., 0., 0.))
    # Each sample is well below both resolutions; the complete route is not.
    for i in [*range(1001), *range(999, -1, -1)]:
        travel, turn = motion.update((i*.001, 0., i*.0005))
    assert travel == pytest.approx(2., abs=.04)
    assert turn == pytest.approx(1., abs=.02)
    motion.reset((0., 0., 0.))
    assert motion.update((.005, 0., .003)) == pytest.approx((.005, .003))


def test_translation_and_rotation_anchors_are_independent_and_turns_unwrap():
    motion = MotionSinceMatch()
    motion.reset((0., 0., 0.))
    for i in range(8001):
        # Translation anchors advance often; slow rotation must not reset with them.
        yaw=i*4*math.pi/8000
        travel, turn = motion.update((i*.003, 0., math.atan2(math.sin(yaw), math.cos(yaw))))
    assert travel == pytest.approx(24.)
    assert turn == pytest.approx(4*math.pi)


@pytest.mark.parametrize('now_ns', [34_000_000_000, 1_790_000_000_000_000_000])
def test_status_future_limit_is_inclusive_to_one_nanosecond(now_ns):
    for offset, expected in [(200_000_000, True), (200_000_001, False)]:
        health = (10., now_ns + offset, True)
        assert observation_health_current(health, now_ns, 10., 2.5, future_tolerance=.2) is expected
