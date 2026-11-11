from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[2]
CONFIG=ROOT/'config'


def test_existing_user_override_files_remain_at_public_paths():
    for platform in ('hardware','sim'):
        for name in ('nav2.yaml','online_map.yaml','obstacles.yaml'):
            assert isinstance(yaml.safe_load((CONFIG/platform/name).read_text()),dict)
    for name in ('deployment.example.yaml','livox_driver.yaml','livox_network.json','lio.yaml'):
        assert (CONFIG/'hardware'/name).is_file()
    params=yaml.safe_load((CONFIG/'matching/slam.yaml').read_text())['slam_toolbox']['ros__parameters']
    assert params['local_match_validation'] and params['do_loop_closing'] and params['use_scan_matching']
    assert params['transform_publish_period']==0.0  # adapter is the only SLAM correction publisher


def test_existing_navigation_plugin_identifiers_remain_loadable_names():
    for platform in ('sim','hardware'):
        text=(CONFIG/platform/'nav2.yaml').read_text()
        assert 'pm01_fastlio_nav::LatestTfRotationShimController' in text
        assert 'pm01_fastlio_nav::SettledGoalChecker' in text
