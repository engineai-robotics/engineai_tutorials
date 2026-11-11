"""Evaluate launch parameter overrides as ROS launch does, including old YAML."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

pytest.importorskip('launch_ros')
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


@pytest.fixture
def launch_module():
    source = Path(__file__).resolve().parents[1] / 'launch/nav2_stack.launch.py'
    spec = importlib.util.spec_from_file_location('pm01_nav2_launch', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('custom', [None, ['user_controller_bt', 'pm01_navigation_ready_wait_bt_node']])
def test_selected_legacy_yaml_adds_guard_without_losing_plugins(launch_module, tmp_path, monkeypatch, custom):
    (tmp_path / 'params').mkdir()
    defaults = {'bt_navigator': {'ros__parameters': {'plugin_lib_names': ['nav2_follow_path_action_bt_node']}}}
    (tmp_path / 'params/nav2_params.yaml').write_text(yaml.safe_dump(defaults))
    user = {'bt_navigator': {'ros__parameters': {'use_sim_time': True}}}
    if custom is not None:
        user['bt_navigator']['ros__parameters']['plugin_lib_names'] = custom
    source = tmp_path / 'user.yaml'
    source.write_text(yaml.safe_dump(user))
    original = source.read_text()
    monkeypatch.setattr(launch_module, 'get_package_share_directory', lambda package: str(tmp_path))
    context = LaunchContext()
    context.launch_configurations.update(params_file=str(source), namespace='')
    value = ParameterValue(launch_module.NavigationBtPlugins(
        LaunchConfiguration('params_file'), LaunchConfiguration('namespace'))).evaluate(context)
    assert value == (custom if custom is not None else
                     ['nav2_follow_path_action_bt_node', 'pm01_navigation_ready_wait_bt_node'])
    assert source.read_text() == original


def test_namespaced_plugin_list_is_extended_once(launch_module):
    config = {'robot': {'bt_navigator': {'ros__parameters': {'plugin_lib_names': ['custom']}}}}
    assert launch_module.navigation_bt_plugins(config, {}, '/robot') == [
        'custom', 'pm01_navigation_ready_wait_bt_node']


def test_custom_tree_preserves_main_behavior_and_includes(launch_module, tmp_path):
    source = tmp_path / 'custom.xml'
    source.write_text('''<root main_tree_to_execute="Navigate" BTCPP_format="3">
      <include path="helpers/subtrees.xml"/>
      <BehaviorTree ID="Helper"><AlwaysSuccess/></BehaviorTree>
      <BehaviorTree ID="Navigate"><Sequence name="custom_order">
        <CustomPlanner path="{path}"/><FollowPath path="{path}"/>
      </Sequence></BehaviorTree></root>''')
    original = source.read_text()
    result = tmp_path / 'guarded.xml'
    launch_module.guard_navigation_tree(source, result)
    root = ET.parse(result).getroot()
    assert root.attrib == {'main_tree_to_execute': 'Navigate', 'BTCPP_format': '3'}
    assert root.find('include').get('path') == str(tmp_path / 'helpers/subtrees.xml')
    assert root.find("BehaviorTree[@ID='Helper']/AlwaysSuccess") is not None
    guard = root.find("BehaviorTree[@ID='Navigate']/NavigationReadyWait")
    assert guard.get('timeout') == '30.0'
    sequence = guard.find('Sequence')
    assert sequence.get('name') == 'custom_order'
    assert [item.tag for item in sequence] == ['CustomPlanner', 'FollowPath']
    assert sequence[0].get('path') == '{path}'
    assert source.read_text() == original


def test_existing_guard_is_not_duplicated(launch_module, tmp_path):
    source = tmp_path / 'custom.xml'
    source.write_text('<root><BehaviorTree ID="Main"><NavigationReadyWait timeout="30">'
                      '<AlwaysSuccess/></NavigationReadyWait></BehaviorTree></root>')
    result = tmp_path / 'guarded.xml'
    launch_module.guard_navigation_tree(source, result)
    assert len(ET.parse(result).getroot().findall('.//NavigationReadyWait')) == 1


def test_selected_custom_tree_uses_temporary_copy_and_cleanup(launch_module, tmp_path):
    source = tmp_path / 'custom.xml'
    source.write_text('<root><BehaviorTree ID="Main"><AlwaysSuccess/></BehaviorTree></root>')
    config = tmp_path / 'params.yaml'
    config.write_text(yaml.safe_dump({'bt_navigator': {'ros__parameters': {
        'default_nav_to_pose_bt_xml': str(source)}}}))
    context = LaunchContext()
    context.launch_configurations.update(params_file=str(config), namespace='')
    trees = launch_module.NavigationBtTrees(LaunchConfiguration('params_file'),
                                            LaunchConfiguration('namespace'), '/installed/defaults')
    selected = ParameterValue(launch_module.NavigationBtTree(trees, 'navigate_to_pose'),
                              value_type=str).evaluate(context)
    assert Path(selected).exists() and selected != str(source)
    assert trees.selected(context, 'navigate_through_poses') == '/installed/defaults/navigate_through_poses.xml'
    trees.cleanup()
    assert not Path(selected).exists()
    assert source.exists()


def test_ambiguous_main_tree_is_rejected(launch_module, tmp_path):
    source = tmp_path / 'ambiguous.xml'
    source.write_text('<root><BehaviorTree ID="A"><AlwaysSuccess/></BehaviorTree>'
                      '<BehaviorTree ID="B"><AlwaysFailure/></BehaviorTree></root>')
    with pytest.raises(ValueError, match='unique main'):
        launch_module.guard_navigation_tree(source, tmp_path / 'guarded.xml')
