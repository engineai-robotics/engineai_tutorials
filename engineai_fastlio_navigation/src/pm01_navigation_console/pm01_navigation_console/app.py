"""Native Ubuntu frontend. Session lifecycle uses SSH; navigation uses ROS 2."""
import argparse
import json
import os
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

from .config import Config
from .operations import Operations
from .session_client import SessionClient
from .status import detail


class Backend:
    def __init__(self, config):
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_action_status_default
        from action_msgs.srv import CancelGoal
        from action_msgs.msg import GoalStatusArray
        from pm01_navigation_client.navigator import Navigator
        from pm01_navigation_runtime.system.processes import OutputProcess
        self.config = config
        self.session = SessionClient(config)
        self.node = Navigator('pm01_navigation_console')
        self.cancel = self.node.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.diagnostics = {}
        self.goal_states = {}
        self.cancel_request = 'not_requested'
        self.cancel_ids = set()
        self.node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.goals,
                                      qos_profile_action_status_default)
        self.late = queue.Queue()
        self.rviz = OutputProcess()
        for topic in ('/lio/status', '/pm01/map_update_status', '/pm01/localization_status'):
            self.node.create_subscription(DiagnosticArray, topic,
                lambda msg, topic=topic: self.diagnostic(topic, msg), 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def connect(self):
        result = self.session.call('identity')
        if self.config.ssh_host:
            from pm01_navigation_runtime.system.clock_check import check
            result['clock'] = check(self.config.ssh_host, self.config.expected_machine_id)
        return result

    def diagnostic(self, topic, message):
        def decode(value):
            try:
                return json.loads(value)
            except ValueError:
                return value
        if message.status:
            entry = message.status[0]
            self.diagnostics[topic] = {'message': entry.message, 'received': time.monotonic(),
                'values': {v.key: decode(v.value) for v in entry.values}}

    def goals(self, message):
        self.goal_states = {bytes(s.goal_info.goal_id.uuid).hex(): int(s.status) for s in message.status_list}
        self.update_cancel()

    def update_cancel(self):
        if self.cancel_ids and self.cancel_ids.issubset(self.goal_states):
            states = [self.goal_states[key] for key in self.cancel_ids]
            if all(state == 5 for state in states):
                self.cancel_request = 'confirmed_canceled'
            elif all(state in (4, 5, 6) for state in states):
                self.cancel_request = 'terminal_result_observed'

    def cancel_acknowledged(self, result):
        self.cancel_ids = {bytes(g.goal_id.uuid).hex() for g in result.goals_canceling}
        self.cancel_request = ('accepted_waiting_terminal' if self.cancel_ids else 'no_goals_canceling') if result.return_code == 0 else 'rejected'
        self.update_cancel()

    def status(self):
        value = self.node.get_status()
        value.update(cancel_request=self.cancel_request, observed_goal_states=self.goal_states.copy())
        value['diagnostics'] = {topic: dict(entry, receive_age_sec=time.monotonic()-entry['received'])
                                for topic, entry in self.diagnostics.copy().items()}
        return value

    def call_ros(self, action):
        from action_msgs.srv import CancelGoal
        client = self.cancel if action == 'cancel' else self.node.trigger_clients[action]
        if not client.wait_for_service(timeout_sec=2.):
            raise RuntimeError('ROS 服务尚未发现：'+action)
        future = client.call_async(CancelGoal.Request()) if action == 'cancel' else getattr(self.node, action)()
        ready = threading.Event()
        future.add_done_callback(lambda f: ready.set())
        if not ready.wait(self.config.request_timeout):
            def late(f):
                try:
                    result = f.result()
                    if action == 'cancel':
                        self.cancel_acknowledged(result)
                    self.late.put('迟到回执 '+action+'：'+str(result))
                except Exception as error:
                    self.late.put('迟到错误 '+action+'：'+str(error))
            future.add_done_callback(late)
            if action == 'cancel':
                self.cancel_request = 'unknown_timeout'
            raise TimeoutError('请求结果未确认；刷新状态，勿将超时解释为停车或释放成功')
        result = future.result()
        if hasattr(result, 'success') and not result.success:
            raise RuntimeError(result.message)
        if action == 'cancel':
            self.cancel_acknowledged(result)
            return '取消请求返回；最终取消状态请查看任务状态：'+str(result)
        return result.message

    def restart_rviz(self):
        from ament_index_python.packages import get_package_share_directory
        self.rviz.stop(2.)
        config = self.config.rviz_config or str(Path(get_package_share_directory('pm01_navigation_bringup'))/'config/hardware/navigation.rviz')
        self.rviz.start(['rviz2', '-d', str(Path(config).expanduser()), '--ros-args',
                         '-p', 'use_sim_time:='+str(self.config.profile == 'sim').lower()])
        return 'RViz 已启动；机器人上的会话和目标保持原状态'

    def close(self):
        self.rviz.stop(2.)
        self.executor.shutdown(timeout_sec=2.)
        self.thread.join(timeout=2.)
        self.node.destroy_node()


class Panel:
    def __init__(self, backend):
        self.backend = backend
        self.operations = Operations()
        self.root = tk.Tk()
        self.root.title('PM01 Navigation Console ['+backend.config.profile+']')
        self.root.geometry('900x840')
        self.root.minsize(780, 700)
        self.pose = tk.BooleanVar(value=False)
        self.map_yaml = tk.StringVar()
        self.state = tk.StringVar(value='等待 ROS 状态')
        self.connection = tk.StringVar(value='请先连接并核对机器人身份')
        area = ttk.Frame(self.root, padding=18)
        area.pack(fill='both', expand=True)
        ttk.Label(area, text='PM01 导航控制台 · '+('真机' if backend.config.profile == 'hardware' else '仿真'), font=('', 22, 'bold')).pack(anchor='w')
        ttk.Label(area, textvariable=self.connection, wraplength=850).pack(anchor='w', pady=8)
        ttk.Label(area, textvariable=self.state, justify='left', wraplength=850).pack(anchor='w', pady=8)
        row = ttk.Frame(area); row.pack(fill='x')
        self.button(row, '连接并核对机器人', 'connect', backend.connect, 0)
        self.button(row, '新建地图并启动导航', 'fresh', lambda: backend.session.call('start', mode='fresh'), 1)
        self.button(row, '仅建图', 'mapping', lambda: backend.session.call('start', mode='mapping'), 2)
        ttk.Label(area, text='机器人上的地图 YAML：').pack(anchor='w', pady=(12, 0))
        self.maps = ttk.Combobox(area, textvariable=self.map_yaml)
        self.maps.pack(fill='x')
        row = ttk.Frame(area); row.pack(fill='x')
        self.button(row, '刷新地图列表', 'maps', lambda: backend.session.call('maps'), 0)
        self.button(row, '加载旧图并启动', 'saved', None, 1)
        self.button(row, '保存当前地图', 'save_map', lambda: backend.call_ros('save_map'), 2)
        ttk.Checkbutton(area, variable=self.pose,
            text='我已确认位置与朝向正确，机器人已站稳，现场停止手段可用').pack(anchor='w', pady=14)
        row = ttk.Frame(area); row.pack(fill='x')
        self.button(row, '接通导航运动输出', 'enable_motion', lambda: backend.call_ros('enable_motion'), 0)
        self.button(row, '取消当前目标', 'cancel', lambda: backend.call_ros('cancel'), 1)
        tk.Button(area, text='释放本项目导航输出', bg='#ba3030', fg='white', font=('', 14, 'bold'),
            command=lambda: self.request('release', lambda: backend.call_ros('release_motion'))).pack(fill='x', pady=10, ipady=9)
        row = ttk.Frame(area); row.pack(fill='x')
        self.button(row, '重启当前模式（重置 LIO）', 'restart', lambda: backend.session.call('restart'), 0)
        self.button(row, '重启 RViz', 'rviz', backend.restart_rviz, 1)
        self.button(row, '停止本项目会话', 'stop', lambda: backend.session.call('stop'), 2)
        ttk.Label(area, text='旧图用 2D Pose Estimate 定位，再用 Nav2 Goal 选点。到达或取消后可继续选点。\n'
                  '关闭窗口、开发电脑断网不会自动取消目标。释放响应与目标取消结果分别显示。',
                  wraplength=850).pack(anchor='w', pady=12)
        self.log = tk.Text(area, height=10, state='disabled', wrap='word')
        self.log.pack(fill='both', expand=True)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.after(100, self.tick)

    def button(self, parent, text, name, function, column):
        parent.columnconfigure(column, weight=1)
        ttk.Button(parent, text=text, command=lambda: self.request(name, function)).grid(
            row=0, column=column, sticky='ew', padx=3, pady=4, ipady=5)

    def request(self, name, function):
        if name == 'saved':
            path = self.map_yaml.get().strip()
            if not path:
                messagebox.showwarning('地图路径', '请先选择或填写机器人上的地图 YAML')
                return
            function = lambda: self.backend.session.call('start', path, mode='saved')
        if name == 'enable_motion':
            if not self.pose.get():
                messagebox.showwarning('现场确认', '请先核对定位与机器人状态，并勾选现场确认。')
                return
            if not messagebox.askyesno('接通输出', '接通后，RViz 目标可驱动机器人。确认接通？'):
                return
        if name in ('fresh', 'saved', 'mapping', 'restart'):
            if not messagebox.askyesno('启动或重置 LIO', '机器人需要站稳并保持静止。重启会撤下本项目输出并重置 LIO；旧图需要重新手动定位。继续？'):
                return
            self.pose.set(False)
        if self.operations.submit(name, function):
            self.append('正在执行：'+name)
        else:
            self.append('已有请求尚未结束；释放按钮仍可使用。')

    def append(self, message):
        self.log.configure(state='normal')
        self.log.insert('end', str(message)+'\n')
        self.log.see('end')
        self.log.configure(state='disabled')

    def tick(self):
        self.state.set(detail(self.backend.status()))
        try:
            while True:
                name, ok, result = self.operations.results.get_nowait()
                if not ok:
                    self.append('操作失败或结果未确认：'+str(result))
                    continue
                if name == 'connect':
                    ident = result['identity']
                    self.connection.set('已核对机器人身份：'+ident['hostname']+' / '+ident['architecture'])
                    if result.get('clock'):
                        self.connection.set(self.connection.get()+'　时钟差 {:+.3f} 秒'.format(result['clock']['robot_minus_developer_sec']))
                if isinstance(result, dict) and 'maps' in result:
                    self.maps.configure(values=result['maps'])
                    if result['maps']:
                        self.map_yaml.set(result['maps'][0])
                if name == 'save_map':
                    self.map_yaml.set(str(result))
                if name == 'connect':
                    self.append('机器人身份核对通过；会话'+('运行中' if result.get('running') else '未运行'))
                elif name == 'maps':
                    self.append('找到 {} 张可用地图'.format(len(result.get('maps', []))))
                else:
                    self.append(result.get('message', json.dumps(result, ensure_ascii=False)) if isinstance(result, dict) else result)
        except queue.Empty:
            pass
        try:
            while True:
                self.append(self.backend.late.get_nowait())
        except queue.Empty:
            pass
        self.root.after(200, self.tick)

    def close(self):
        if messagebox.askyesno('关闭控制台', '关闭控制台及本窗口启动的 RViz。机器人导航会话继续运行，不会取消目标或释放输出。确认关闭？'):
            self.operations.close()
            self.root.destroy()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    options = parser.parse_args(args)
    config = Config.load(options.config)
    # Apply after all shell overlays and before the first DDS participant exists.
    from pm01_navigation_runtime.system.network import write_profile
    profile = write_profile(Path(options.config).expanduser().parent/'fastdds-desktop.xml', config.local_addresses, config.peers)
    os.environ.update(RMW_IMPLEMENTATION='rmw_fastrtps_cpp', ROS_DOMAIN_ID=str(config.ros_domain_id),
                      ROS_LOCALHOST_ONLY='0', PM01_USE_SIM_TIME=str(config.profile == 'sim').lower(),
                      FASTRTPS_DEFAULT_PROFILES_FILE=profile)
    os.environ.pop('CYCLONEDDS_URI', None)
    os.environ.pop('FASTDDS_DEFAULT_PROFILES_FILE', None)
    import rclpy
    rclpy.init()
    backend = Backend(config)
    try:
        Panel(backend).root.mainloop()
    finally:
        backend.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
