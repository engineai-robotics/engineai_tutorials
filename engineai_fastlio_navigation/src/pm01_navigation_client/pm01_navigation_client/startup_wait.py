"""Cancelable wait bound to one startup session and one release generation."""
import time
import json
import re
from pathlib import Path


class StartupWait:
    def __init__(self, session_id=None):
        self.session_id = session_id
        self.manager = None
        self.reason = '正在读取当前导航状态 / Reading current navigation status'

    def update(self, status, received, now, *, allow_initial_pose=False):
        if received is None or now-received >= 2.5:
            if self.manager:
                raise RuntimeError('启动状态中断，接通已撤销 / Startup heartbeat lost; enable canceled')
            return False
        session = status.get('session_id')
        if not session:
            return False
        if self.session_id is not None and session != self.session_id:
            raise RuntimeError('导航会话已更换，接通已撤销 / Session changed; enable canceled')
        self.session_id = session
        manager = (status.get('manager_id'), status.get('release_generation'))
        if manager[0] and manager[1] is not None:
            if self.manager is not None and self.manager != manager:
                raise RuntimeError('输出已释放或管理节点已重启，接通已撤销 / Release or restart canceled enable')
            self.manager = manager
        self.reason = status.get('reason', self.reason)
        return bool(status.get('ready') and self.manager) or (
            allow_initial_pose and status.get('phase') == 'initial_pose')


def wait_for_startup(nav, deadline, report, *, session_id=None, allow_initial_pose=False, require_motion=False, status_file=None, require_action=False):
    wait = StartupWait(session_id)
    last_reason = None
    next_observation_detail = time.monotonic() + 5.
    while time.monotonic() < deadline:
        if status_file and session_id:
            try:
                stopped = json.loads(Path(status_file).read_text())
            except (OSError, ValueError):
                stopped = {}
            if stopped.get('session_id') == session_id and stopped.get('phase') in ('failed', 'stopped'):
                raise RuntimeError(stopped['reason'])
        if wait.update(nav.startup, nav.startup_received, time.monotonic(),
                       allow_initial_pose=allow_initial_pose):
            if allow_initial_pose and nav.startup.get('phase') == 'initial_pose':
                return wait
            if require_motion and not nav.motion_request_client.service_is_ready():
                wait.reason = '等待运动请求服务 / Waiting for motion request service'
            elif require_action and not nav.action.server_is_ready():
                wait.reason = '等待导航目标接口 / Waiting for navigation action server'
            else:
                return wait
        reason = wait.reason
        if require_motion and nav.startup.get('phase') in ('waiting_data', 'settling', 'waiting_recovery'):
            if time.monotonic() < next_observation_detail:
                if last_reason is None:
                    reason = '正在确认导航就绪状态 / Checking navigation readiness'
                else:
                    reason = last_reason
            else:
                next_observation_detail = time.monotonic() + 10.
        if reason != last_reason:
            report(reason)
            last_reason = reason
        nav.poll(min(.1, max(0., deadline-time.monotonic())))
    raise TimeoutError('启动等待超时 / Startup timeout: '+wait.reason)


def motion_failure_message(message):
    """Explain an enable result while keeping the service response unchanged."""
    try:
        status = json.loads(message)
    except (ValueError, TypeError):
        status = {}
    if not isinstance(status, dict):
        status = {}
    reason = str(status.get('last_enable_error') or message)
    missing = re.search(r'waiting_for_motion_consumer:([^;\s]+)', reason)
    if missing:
        node = missing.group(1)
        text = ('运动输出接通失败：未发现运控接收节点 '+node+'（/motion/body_vel_cmd）。'
                '请检查运控接口和 ROS 通信配置。\n'
                'Motion enable failed: receiver '+node+' was not discovered on /motion/body_vel_cmd; '
                'check the motion interface and ROS communication configuration.')
    else:
        text = '运动输出接通失败 / Motion enable failed: '+reason
    if (status.get('output_detached') is True and status.get('output_process_running') is False
            and status.get('output_publishers') == 0):
        text += '\n运动输出已撤下 / Motion output detached'
    return text


def wait_for_motion(nav, future, deadline, report):
    """Wait for the single submitted enable request, with transport-stage feedback."""
    started = time.monotonic()
    report('导航已就绪，正在连接运动输出；等待接收端确认 / '
           'Navigation ready; connecting motion output and waiting for the receiver')
    next_report = started+10.
    while not future.done() and time.monotonic() < deadline:
        nav.poll(min(.05, max(0., deadline-time.monotonic())))
        now = time.monotonic()
        if now >= next_report and not future.done():
            elapsed = int(now-started)
            report(f'正在等待运动输出连接，已等待 {elapsed} 秒 / '
                   f'Waiting for motion output connection: {elapsed}s elapsed')
            next_report = now+10.
    if not future.done():
        raise TimeoutError('运动输出接通结果未确认，正在撤销 / Motion enable unconfirmed; canceling')
    response = future.result()
    if not response.success:
        raise RuntimeError(motion_failure_message(response.message))
    return response
