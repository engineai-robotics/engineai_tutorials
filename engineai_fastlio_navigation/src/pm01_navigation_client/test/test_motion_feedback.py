import json
from types import SimpleNamespace

import pytest

from pm01_navigation_client import startup_wait


def test_missing_receiver_has_actionable_error_and_confirmed_output_state():
    result = dict(last_enable_error='Output not ready: waiting_for_motion_consumer:'
                  'locomotion_interface_node; check DDS/consumer, then retry enable-motion',
                  output_process_running=False, output_publishers=0, output_detached=True)
    text = startup_wait.motion_failure_message(json.dumps(result))
    assert '未发现运控接收节点 locomotion_interface_node' in text
    assert '/motion/body_vel_cmd' in text and 'ROS' in text
    assert 'Motion output detached' in text
    result['output_process_running'] = True
    assert 'Motion output detached' not in startup_wait.motion_failure_message(json.dumps(result))


@pytest.mark.parametrize('message', ['Output worker exited before ready', '[]', 'null'])
def test_other_failure_reasons_are_preserved_without_claiming_cleanup(message):
    text = startup_wait.motion_failure_message(message)
    assert message in text and 'Motion enable failed' in text
    assert 'Motion output detached' not in text


def pending_request(monkeypatch, *, completes_at, success):
    clock = [0.]
    monkeypatch.setattr(startup_wait.time, 'monotonic', lambda: clock[0])
    response = SimpleNamespace(success=success, message='worker_ready_service_unavailable')
    future = SimpleNamespace(done=lambda: clock[0] >= completes_at, result=lambda: response)
    def poll(duration):
        clock[0] += duration
    return SimpleNamespace(poll=poll), future, clock


def test_transport_wait_replaces_startup_message_and_reports_progress(monkeypatch):
    nav, future, _ = pending_request(monkeypatch, completes_at=11., success=True)
    reports = []
    result = startup_wait.wait_for_motion(nav, future, 20., reports.append)
    assert result.success
    assert 'Navigation ready; connecting motion output' in reports[0]
    assert any('10s elapsed' in text for text in reports)
    assert all('startup' not in text.lower() for text in reports)


def test_transport_failure_is_distinct_from_startup_failure(monkeypatch):
    nav, future, _ = pending_request(monkeypatch, completes_at=1., success=False)
    with pytest.raises(RuntimeError, match='Motion enable failed: worker_ready_service_unavailable'):
        startup_wait.wait_for_motion(nav, future, 20., lambda _: None)


def test_transport_timeout_retains_unconfirmed_result(monkeypatch):
    nav, future, clock = pending_request(monkeypatch, completes_at=30., success=True)
    with pytest.raises(TimeoutError, match='Motion enable unconfirmed; canceling'):
        startup_wait.wait_for_motion(nav, future, 2., lambda _: None)
    assert clock[0] == pytest.approx(2.)


def test_transport_interrupt_propagates_to_existing_cancellation(monkeypatch):
    nav, future, _ = pending_request(monkeypatch, completes_at=30., success=True)
    def interrupt(_):
        raise KeyboardInterrupt
    nav.poll = interrupt
    with pytest.raises(KeyboardInterrupt):
        startup_wait.wait_for_motion(nav, future, 20., lambda _: None)
