"""Startup conditions and cancellation fences, without actuator hardware."""
import pytest
pytest.importorskip('rclpy')
from pm01_navigation_client.startup import startup_decision
from pm01_navigation_client.startup_wait import StartupWait, wait_for_startup


def ready(**values):
    return dict(session_id='session-a', manager_id='manager-a', release_generation=0,
                ready=True, phase='ready', reason='ready', **values)


def test_saved_prompts_for_pose_before_ready():
    conditions = dict(observations={'ready': False}, nodes={'amcl': True, 'map_server': True}, motion=True,
                      map_loaded=True, localized=False)
    assert startup_decision('saved', **conditions)[0] == 'initial_pose'
    conditions['nodes']['planner_server'] = False
    assert startup_decision('saved', **conditions)[0] == 'initial_pose'
    conditions['nodes']['planner_server'] = True
    conditions['localized'] = True
    assert startup_decision('saved', **conditions)[0] == 'waiting_data'
    conditions['observations']['ready'] = True
    assert startup_decision('saved', **conditions)[0] == 'ready'
    conditions['nodes']['amcl'] = False
    assert startup_decision('saved', **conditions)[0] == 'waiting_nodes'


@pytest.mark.parametrize('mode', ['fresh', 'saved', 'mapping'])
def test_readiness_does_not_enable_output(mode):
    result = startup_decision(mode, observations={'ready': True}, nodes={}, motion=True,
                              map_loaded=True, localized=True)
    assert result[0] == 'ready'
    assert startup_decision(mode, observations={'ready': False, 'reason': 'scan_stale'},
        nodes={}, motion=True, map_loaded=True, localized=True)[0] == 'waiting_data'


@pytest.mark.parametrize('change', [{'session_id': 'session-b'}, {'manager_id': 'manager-b'},
                                   {'release_generation': 1}])
def test_release_or_restart_cancels_pending_enable(change):
    wait = StartupWait()
    first = ready(); first['ready'] = False
    assert not wait.update(first, 10., 10.)
    with pytest.raises(RuntimeError, match='canceled'):
        wait.update(dict(ready(), **change), 11., 11.)


def test_lost_heartbeat_never_enables_from_retained_status():
    wait = StartupWait()
    assert wait.update(ready(), 1., 1.)
    with pytest.raises(RuntimeError, match='heartbeat'):
        wait.update(ready(), 1., 4.)


def test_old_session_cannot_satisfy_explicit_wait():
    with pytest.raises(RuntimeError, match='Session changed'):
        StartupWait('new-session').update(ready(), 1., 1.)


def test_wait_timeout_and_interrupt_do_not_submit_motion(monkeypatch):
    from types import SimpleNamespace
    from pm01_navigation_client import startup_wait
    clock = [1.]
    monkeypatch.setattr(startup_wait.time, 'monotonic', lambda: clock[0])
    nav = SimpleNamespace(startup={}, startup_received=None)
    def poll(_): clock[0] += .1
    nav.poll = poll
    with pytest.raises(TimeoutError, match='Startup timeout'):
        wait_for_startup(nav, 2., lambda _: None)
    def interrupt(_): raise KeyboardInterrupt
    nav.poll = interrupt
    with pytest.raises(KeyboardInterrupt):
        wait_for_startup(nav, 3., lambda _: None)


def test_enable_waits_for_motion_service_after_robot_reports_ready(monkeypatch):
    from types import SimpleNamespace
    from pm01_navigation_client import startup_wait
    clock = [1.]
    monkeypatch.setattr(startup_wait.time, 'monotonic', lambda: clock[0])
    nav = SimpleNamespace(startup=ready(), startup_received=1.)
    nav.motion_request_client = SimpleNamespace(service_is_ready=lambda: clock[0] >= 1.5)
    def poll(_):
        clock[0] += .1
        nav.startup_received = clock[0]
    nav.poll = poll
    reports = []
    result = wait_for_startup(nav, 3., reports.append, require_motion=True)
    assert result.session_id == 'session-a' and clock[0] >= 1.5
    assert any('motion request service' in message for message in reports)


def test_failed_launch_file_ends_interactive_wait_with_cause(tmp_path):
    import json
    import time
    from types import SimpleNamespace
    path = tmp_path/'startup.json'
    path.write_text(json.dumps(dict(session_id='owned', phase='failed', reason='Robot spawn failed')))
    with pytest.raises(RuntimeError, match='Robot spawn failed'):
        wait_for_startup(SimpleNamespace(), time.monotonic()+1., lambda _: None,
                         session_id='owned', status_file=path)


def test_optional_servers_do_not_block_single_goal_startup():
    nodes={'controller_server':True,'planner_server':True,'bt_navigator':True,
           'smoother_server':False,'waypoint_follower':False}
    args=dict(observations={'ready':True},nodes=nodes,motion=True,map_loaded=True,localized=True)
    assert startup_decision('fresh',**args)[0]=='ready'
    nodes['controller_server']=False
    assert startup_decision('fresh',**args)[0]=='waiting_nodes'


def test_running_session_loss_is_not_reported_as_initial_startup_timeout(monkeypatch):
    import json
    import rclpy
    from types import SimpleNamespace
    from pm01_navigation_client import startup
    rclpy.init(); node = startup.StartupMonitor()
    try:
        node.lifecycle = {}
        node.since = 0.
        monkeypatch.setattr(startup.time, 'monotonic', lambda: 200.)
        decision = ['waiting_data', 'scan_stale']
        monkeypatch.setattr(startup, 'startup_decision', lambda *a, **kw: tuple(decision))
        statuses = []
        node.startup_pub = SimpleNamespace(publish=statuses.append)
        def phase():
            node.update_startup()
            return {v.key: json.loads(v.value) for v in statuses[-1].status[0].values}['phase']
        assert phase() == 'startup_delayed'
        decision[:] = ['ready', 'ready']
        assert phase() == 'ready'
        decision[:] = ['waiting_data', 'scan_stale']
        assert phase() == 'waiting_recovery'
    finally:
        node.destroy_node(); rclpy.shutdown()
