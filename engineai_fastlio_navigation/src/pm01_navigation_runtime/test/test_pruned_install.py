from pm01_navigation_runtime.build.manifest import prepare_pruned_layout


def test_rebuilds_only_retired_slam_install_once(tmp_path):
    stale = tmp_path/'install/slam_toolbox/share/ament_index/resource_index/rclcpp_components/slam_toolbox'
    stale.parent.mkdir(parents=True)
    stale.write_text('obsolete_component')
    other = tmp_path/'install/pm01_navigation_core/keep'
    other.parent.mkdir(parents=True)
    other.write_text('keep')
    prepare_pruned_layout(tmp_path, 'desktop')
    assert stale.exists()
    prepare_pruned_layout(tmp_path, 'hardware', 'driver')
    assert stale.exists()
    prepare_pruned_layout(tmp_path, 'hardware', 'navigation')
    assert not stale.exists()
    assert other.read_text() == 'keep'
    stale.parent.mkdir(parents=True)
    stale.write_text('current_component')
    prepare_pruned_layout(tmp_path, 'hardware', 'navigation')
    assert stale.read_text() == 'current_component'
