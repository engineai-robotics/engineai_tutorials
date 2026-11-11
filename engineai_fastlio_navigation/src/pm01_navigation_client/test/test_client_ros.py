"""Isolated ActionServer tests; no physical robot or simulator is used."""
import threading
import time
import pytest
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from pm01_navigation_client import Navigator

class FakeServer:
    def __init__(self):
        self.node=rclpy.create_node('fake_navigation_server')
        self.cancel_delay=0.
        self.long_goal_duration=2.
        self.server=ActionServer(self.node,NavigateToPose,'/navigate_to_pose',
            execute_callback=self.execute,goal_callback=self.goal,cancel_callback=self.cancel,
            callback_group=ReentrantCallbackGroup())
        self.executor=MultiThreadedExecutor(num_threads=4);self.executor.add_node(self.node)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
    def goal(self,request):
        if request.pose.pose.position.x==-1:return GoalResponse.REJECT
        if request.pose.pose.position.x==2:time.sleep(.5)
        return GoalResponse.ACCEPT
    def cancel(self,handle):
        time.sleep(self.cancel_delay)
        return CancelResponse.ACCEPT
    def execute(self,handle):
        x=handle.request.pose.pose.position.x
        start=time.monotonic()
        while time.monotonic()-start<(self.long_goal_duration if x in (2,3) else .2):
            if handle.is_cancel_requested:
                handle.canceled();return NavigateToPose.Result()
            feedback=NavigateToPose.Feedback();feedback.distance_remaining=1.
            handle.publish_feedback(feedback);time.sleep(.02)
        handle.succeed();return NavigateToPose.Result()
    def close(self):
        self.executor.shutdown();self.thread.join();self.server.destroy();self.node.destroy_node()

@pytest.fixture
def pair(monkeypatch):
    monkeypatch.setenv('PM01_USE_SIM_TIME','false')
    rclpy.init();server=FakeServer();nav=Navigator()
    assert nav.wait_until_ready(5.,require_map=False)
    yield server,nav
    nav.destroy_node();server.close();rclpy.shutdown()

def test_success_and_continuous_goals(pair):
    _,nav=pair
    a=nav.navigate_to_pose(0.,0.)
    assert a.state=='waiting_for_acceptance' and len(a.uuid)==32
    assert nav.wait(a,3.) and a.state=='succeeded'
    b=nav.navigate_to_pose(1.,0.)
    assert nav.wait(b,3.) and b.state=='succeeded' and a.uuid!=b.uuid
    assert a.feedback is not None

def test_rejected(pair):
    _,nav=pair;t=nav.navigate_to_pose(-1.,0.)
    assert nav.wait(t,3.) and t.state=='rejected'


def test_navigation_diagnostics_do_not_replace_action_result(pair):
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.qos import QoSProfile, DurabilityPolicy
    server, nav = pair
    server.long_goal_duration = 10.
    publishers = [server.node.create_publisher(DiagnosticArray, topic,
                  QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)) for topic in
                  ('/pm01/navigation_readiness', '/pm01/navigation_wait_status')]
    task = nav.navigate_to_pose(3., 0.)
    messages = [DiagnosticArray(status=[DiagnosticStatus(message=reason, values=[
        KeyValue(key='state', value=state), KeyValue(key='ready', value='false')])])
        for state, reason in [('blocked', 'scan_stale'), ('timed_out', 'navigation_observation_timeout')]]
    try:
        deadline = time.monotonic()+1.5
        while not nav.navigation_wait and time.monotonic() < deadline:
            for publisher, message in zip(publishers, messages): publisher.publish(message)
            nav.poll(.02)
        status = nav.get_status()
        assert status['navigation_readiness']['ready'] is False
        assert status['navigation_wait']['state'] == 'timed_out'
        assert status['navigation_readiness_receive_age_sec'] < 1.
        assert not task.done
        nav.cancel(task)
        assert nav.wait(task, 3.) and task.state == 'canceled'
    finally:
        for publisher in publishers: server.node.destroy_publisher(publisher)

def test_late_acceptance_after_timeout_and_early_cancel(pair):
    _,nav=pair;t=nav.navigate_to_pose(2.,0.)
    assert not nav.wait(t,.05) and t.state=='waiting_for_acceptance'
    nav.cancel(t)
    assert not t.done
    assert nav.wait(t,4.) and t.state=='canceled'
    assert bytes(t.goal_handle.goal_id.uuid).hex()==t.uuid

def test_cancel_timeout_is_not_success(pair):
    server,nav=pair;t=nav.navigate_to_pose(3.,0.)
    while t.goal_handle is None:nav.poll(.01)
    server.cancel_delay=.6;nav.cancel(t)
    assert not nav.wait(t,.05) and t.state=='canceling'
    assert nav.wait(t,4.) and t.state=='canceled'

def test_duplicate_inflight_refused(pair):
    _,nav=pair;t=nav.navigate_to_pose(3.,0.)
    with pytest.raises(RuntimeError):nav.navigate_to_pose(0.,0.)
    nav.cancel(t);assert nav.wait(t,4.)


def test_relocalization_ack_does_not_create_goal_or_claim_convergence(pair):
    from std_srvs.srv import Empty
    server, nav = pair
    calls = []
    def requested(request, response):
        calls.append(True)
        return response
    service = server.node.create_service(Empty, '/reinitialize_global_localization', requested)
    try:
        assert nav.relocalize_client.wait_for_service(timeout_sec=3.)
        future = nav.relocalize()
        deadline = time.monotonic()+3.
        while not future.done() and time.monotonic() < deadline: nav.poll(.05)
        assert future.done() and calls == [True]
        assert nav.tasks == {} and nav.localization == {} and nav.motion == {}
    finally:
        server.node.destroy_service(service)

def test_initialpose_and_save_map_use_standard_interfaces(pair):
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_srvs.srv import Trigger
    server,nav=pair
    received=[]
    sub=server.node.create_subscription(PoseWithCovarianceStamped,'/initialpose',received.append,10)
    def save(request,response):
        response.success=True;response.message='/tmp/example/map.yaml';return response
    srv=server.node.create_service(Trigger,'/pm01/save_navigation_map',save)
    try:
        end=time.monotonic()+3.
        while not nav.initial_pub.get_subscription_count() and time.monotonic()<end:nav.poll(.05)
        sent=nav.set_initial_pose(5.,3.,1.57)
        while not received and time.monotonic()<end:nav.poll(.05)
        assert received and received[0].pose.pose.position.x==5.
        assert sent.header.frame_id=='map' and sent.header.stamp.sec>0
        assert nav.trigger_clients['save_map'].wait_for_service(timeout_sec=3.)
        future=nav.save_map()
        while not future.done() and time.monotonic()<end:nav.poll(.05)
        assert future.done() and future.result().success
        assert future.result().message.endswith('map.yaml')
        # No AMCL node exists. Action-only readiness succeeds; map/TF readiness
        # accurately remains false instead of blocking indefinitely on AMCL.
        assert nav.wait_until_ready(.1,require_map=False)
        assert not nav.wait_until_ready(.1)
    finally:
        server.node.destroy_subscription(sub);server.node.destroy_service(srv)
