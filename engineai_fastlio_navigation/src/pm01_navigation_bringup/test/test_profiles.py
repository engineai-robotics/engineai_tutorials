from pathlib import Path
import yaml

CONFIG = Path(__file__).resolve().parents[3]/'config'


def normalized(value):
    if isinstance(value, dict):
        return {k: normalized(v) for k, v in value.items() if k != 'use_sim_time'}
    if isinstance(value, list):
        return [normalized(v) for v in value]
    return value


def test_navigation_and_ground_profiles_share_the_hardware_contract():
    for name in ('nav2.yaml', 'obstacles.yaml', 'online_map.yaml'):
        sim = yaml.safe_load((CONFIG/'sim'/name).read_text())
        hardware = yaml.safe_load((CONFIG/'hardware'/name).read_text())
        assert normalized(sim) == normalized(hardware), name
    nav = yaml.safe_load((CONFIG/'hardware/nav2.yaml').read_text())
    controller = nav['controller_server']['ros__parameters']['FollowPath']
    assert controller['max_vel_x'] == controller['max_speed_xy'] == .6
    assert nav['velocity_smoother']['ros__parameters']['max_velocity'] == [.6, .4, .7]
    for profile in ('hardware', 'sim'):
        for name, node in [('obstacles.yaml', 'navigation_obstacle_cloud'),
                           ('online_map.yaml', 'online_navigation_map')]:
            p = yaml.safe_load((CONFIG/profile/name).read_text())[node]['ros__parameters']
            assert p['use_shared_ground'] and 'ground_z' not in p


def test_lio_only_differs_in_scan_transport_and_physical_mounting():
    sim = yaml.safe_load((CONFIG/'sim/lio.yaml').read_text())['/**']['ros__parameters']
    hardware = yaml.safe_load((CONFIG/'hardware/lio.yaml').read_text())['/**']['ros__parameters']
    assert sim['common'].pop('lidar_is_snapshot') is True
    assert sim['common'].pop('lidar_qos_best_effort') is True
    for values in (sim, hardware):
        for key in ('initial_xyz', 'initial_rpy'):
            values['mapping'].pop(key)
    assert sim == hardware
