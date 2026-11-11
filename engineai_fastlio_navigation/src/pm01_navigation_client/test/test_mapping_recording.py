from pm01_navigation_client.diagnostics.mapping_recording import command, TOPICS
import pytest


def test_recording_captures_backend_context_without_actuator_channels(tmp_path):
    args = command(tmp_path/'recording', True)
    assert args[:3] == ['ros2', 'bag', 'record']
    assert all(topic in args for topic in TOPICS)
    assert not any('cmd_vel' in topic or 'body_vel_cmd' in topic for topic in args)
    with pytest.raises(ValueError): command(tmp_path)


def test_navigation_capture_only_adds_passive_rosbag_subscriptions(tmp_path):
    from pm01_navigation_client.diagnostics.mapping_recording import NAVIGATION_TOPICS
    args = command(tmp_path/'navigation', include_navigation=True)
    assert args[:3] == ['ros2', 'bag', 'record']
    assert all(topic in args for topic in NAVIGATION_TOPICS)
