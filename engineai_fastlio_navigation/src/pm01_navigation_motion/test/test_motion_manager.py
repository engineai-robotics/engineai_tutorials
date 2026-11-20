"""Release must remove output while a cancel service is stalled."""
from pathlib import Path
import sys
import threading
import time
import rclpy
from action_msgs.srv import CancelGoal
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from pm01_navigation_motion.manager import MotionManager

def test_release_detaches_publisher_before_stalled_cancel(tmp_path):
    worker=tmp_path/'worker.py'
    worker.write_text('''import signal, rclpy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
rclpy.init();n=rclpy.create_node('fake_output_worker');p=n.create_publisher(Twist,'/pm01/walking_cmd_vel',1)
n.create_timer(.02,lambda:p.publish(Twist()))
def ready(request,response):response.success=True;return response
n.create_service(Trigger,n.declare_parameter('ready_service','~/ready').value,ready)
try: rclpy.spin(n)
except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
finally:
    n.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
''')
    rclpy.init(args=['--ros-args','-p','worker_executable:='+sys.executable,
        '-p','worker_arguments:=["'+str(worker)+'"]','-p','log_root:='+str(tmp_path)])
    manager=MotionManager();test=rclpy.create_node('motion_manager_test')
    def stalled(request,response):time.sleep(4.);return response
    server=test.create_service(CancelGoal,'/navigate_to_pose/_action/cancel_goal',stalled,callback_group=ReentrantCallbackGroup())
    executor=MultiThreadedExecutor(num_threads=4);executor.add_node(manager);executor.add_node(test)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    enable=test.create_client(Trigger,'/pm01/navigation/enable_motion');release=test.create_client(Trigger,'/pm01/navigation/release_motion')
    def await_future(f,seconds):
        end=time.monotonic()+seconds
        while not f.done() and time.monotonic()<end:time.sleep(.01)
        assert f.done();return f.result()
    try:
        assert enable.wait_for_service(timeout_sec=4.)
        assert release.wait_for_service(timeout_sec=4.)
        # Worker startup includes Python imports and DDS discovery. Use the
        # production readiness budget; the release latency assertion below
        # remains strict even on an emulated ARM64 test runner.
        assert await_future(enable.call_async(Trigger.Request()),11.).success
        end=time.monotonic()+5.
        while not test.count_publishers('/pm01/walking_cmd_vel') and time.monotonic()<end:time.sleep(.05)
        assert test.count_publishers('/pm01/walking_cmd_vel')==1
        assert not await_future(enable.call_async(Trigger.Request()),2.).success
        start=time.monotonic();response=await_future(release.call_async(Trigger.Request()),2.3)
        assert response.success and time.monotonic()-start<2.3
        assert not manager.worker.running
        end=time.monotonic()+2.
        while test.count_publishers('/pm01/walking_cmd_vel') and time.monotonic()<end:time.sleep(.05)
        assert test.count_publishers('/pm01/walking_cmd_vel')==0
        assert manager.cancel_state in ('pending','unknown_timeout')
    finally:
        manager.worker.stop();executor.shutdown();thread.join();manager.destroy_node();test.destroy_node();rclpy.shutdown()


def test_enable_waits_for_consumer_and_failed_attempt_can_retry(tmp_path):
    worker = tmp_path/'worker.py'
    ready_file = tmp_path/'consumer-ready'
    worker.write_text('''import rclpy,sys
from pathlib import Path
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
rclpy.init();n=rclpy.create_node('delayed_output_worker')
p=n.create_publisher(Twist,'/pm01/walking_cmd_vel',1)
n.create_timer(.02,lambda:p.publish(Twist()))
def ready(request,response):
    response.success=Path(sys.argv[1]).exists()
    response.message='waiting_for_motion_consumer'
    return response
n.create_service(Trigger,n.declare_parameter('ready_service','~/ready').value,ready)
try:rclpy.spin(n)
except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
finally:
    n.destroy_node()
    if rclpy.ok():rclpy.shutdown()
''')
    rclpy.init(args=['--ros-args','-p','worker_executable:='+sys.executable,
        '-p','worker_arguments:=["'+str(worker)+'", "'+str(ready_file)+'"]',
        '-p','enable_timeout_sec:=8.0','-p','log_root:='+str(tmp_path)])
    manager=MotionManager(); executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(manager)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        result=manager.enable(Trigger.Request(),Trigger.Response())
        assert not result.success and not manager.worker.running
        assert manager.state=='enable_failed' and not manager.transport_ready
        assert 'waiting_for_motion_consumer' in result.message
        # Old publisher discovery eventually expires; no session restart needed.
        end=time.monotonic()+4.
        while manager.output_publishers() and time.monotonic()<end:time.sleep(.05)
        ready_file.touch()
        result=manager.enable(Trigger.Request(),Trigger.Response())
        assert result.success and manager.transport_ready
    finally:
        manager.worker.stop();executor.shutdown();thread.join();manager.destroy_node();rclpy.shutdown()


def test_worker_immediate_exit_never_returns_enable_success(tmp_path):
    rclpy.init(args=['--ros-args','-p','worker_executable:=/bin/false',
                    '-p','log_root:='+str(tmp_path)])
    manager=MotionManager()
    try:
        result=manager.enable(Trigger.Request(),Trigger.Response())
        assert not result.success
        assert 'exited before ready' in result.message
        assert not manager.worker.running and not manager.transport_ready
    finally:
        manager.destroy_node();rclpy.shutdown()


def test_release_interrupts_an_enable_waiting_for_worker_ready(tmp_path):
    worker = tmp_path/'waiting.py'
    worker.write_text('import time\ntime.sleep(60)\n')
    rclpy.init(args=['--ros-args','-p','worker_executable:='+sys.executable,
                    '-p','worker_arguments:=["'+str(worker)+'"]',
                    '-p','log_root:='+str(tmp_path)])
    manager=MotionManager(); results=[]
    thread=threading.Thread(target=lambda:results.append(
        manager.enable(Trigger.Request(),Trigger.Response())))
    thread.start()
    try:
        end=time.monotonic()+2.
        while not manager.worker.running and time.monotonic()<end:time.sleep(.01)
        assert manager.worker.running
        start=time.monotonic()
        response=manager.release(Trigger.Request(),Trigger.Response())
        assert response.success and time.monotonic()-start<2.5
        thread.join(timeout=1.)
        assert results and not results[0].success
        assert not manager.worker.running and manager.state=='disabled'
    finally:
        manager.enable_cancel.set(); thread.join(timeout=3.)
        manager.destroy_node();rclpy.shutdown()


def test_session_release_fence_refuses_late_enable_and_cancel(tmp_path):
    from rcl_interfaces.srv import SetParametersAtomically
    from rclpy.parameter import Parameter
    rclpy.init(args=['--ros-args', '-p', 'worker_executable:=/bin/false',
                    '-p', 'log_root:='+str(tmp_path), '-p', 'session_id:=fixture'])
    manager = MotionManager()
    def request(values):
        return manager.request_motion(SetParametersAtomically.Request(parameters=[
            Parameter(k, value=v).to_parameter_msg() for k, v in values.items()]),
            SetParametersAtomically.Response()).result
    values = dict(session_id='fixture', manager_id=manager.manager_id, release_generation=0,
                  operation='enable', timeout_sec=1.)
    try:
        manager.release(None, Trigger.Response())
        result = request(values)
        assert not result.successful and manager.worker.process is None
        # An old cancel cannot tear down a later operation or increment its fence.
        generation = manager.release_generation
        assert request(dict(values, operation='cancel')).successful
        assert manager.release_generation == generation
        values.update(release_generation=generation, manager_id='previous-manager')
        assert not request(values).successful
        assert manager.worker.process is None
    finally:
        manager.destroy_node(); rclpy.shutdown()


def test_lost_ready_reply_requeries_same_worker_without_enabling_again(tmp_path, monkeypatch):
    from types import SimpleNamespace
    from rclpy.task import Future
    from pm01_navigation_motion import manager as module
    rclpy.init(args=['--ros-args', '-p', 'log_root:='+str(tmp_path)])
    manager = MotionManager()
    worker = manager.worker
    calls, removed = [], []
    clock = [0.]
    def query(request):
        future = Future()
        calls.append(future)
        if len(calls) == 2:
            future.set_result(Trigger.Response(success=True, message='ready'))
        return future
    client = SimpleNamespace(service_is_ready=lambda: True, call_async=query,
                             remove_pending_request=removed.append)
    try:
        manager.worker = SimpleNamespace(running=True)
        monkeypatch.setattr(manager, 'create_client', lambda *a, **k: client)
        monkeypatch.setattr(manager, 'destroy_client', lambda c: None)
        monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
        monkeypatch.setattr(module.time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0]+seconds))
        manager.await_worker_ready('/test/ready', 2.)
        assert len(calls) == 2 and removed == [calls[0]] and calls[0].cancelled()
        assert clock[0] < 2.
    finally:
        monkeypatch.undo()
        manager.worker = worker
        manager.destroy_node(); rclpy.shutdown()


def test_ready_timeout_preserves_the_workers_waiting_reason(tmp_path, monkeypatch):
    from types import SimpleNamespace
    from rclpy.task import Future
    from pm01_navigation_motion import manager as module
    import pytest
    rclpy.init(args=['--ros-args', '-p', 'log_root:='+str(tmp_path)])
    manager = MotionManager()
    worker, clock, calls = manager.worker, [0.], []
    def query(request):
        future = Future()
        calls.append(future)
        if len(calls) == 1:
            future.set_result(Trigger.Response(success=False, message='waiting_for_motion_consumer'))
        return future
    client = SimpleNamespace(service_is_ready=lambda: True, call_async=query,
                             remove_pending_request=lambda future: None)
    try:
        manager.worker = SimpleNamespace(running=True)
        monkeypatch.setattr(manager, 'create_client', lambda *a, **k: client)
        monkeypatch.setattr(manager, 'destroy_client', lambda client: None)
        monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
        monkeypatch.setattr(module.time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0]+seconds))
        with pytest.raises(TimeoutError, match='waiting_for_motion_consumer'):
            manager.await_worker_ready('/test/ready', 2.)
        assert len(calls) >= 2 and calls[-1].cancelled()
    finally:
        monkeypatch.undo()
        manager.worker = worker
        manager.destroy_node(); rclpy.shutdown()
