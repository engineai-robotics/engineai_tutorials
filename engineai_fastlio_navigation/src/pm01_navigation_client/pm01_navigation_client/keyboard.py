"""Optional simulation keyboard UI; uses the existing managed /cmd_vel path.

The Qt and ROS imports are lazy so core/client development stays GUI independent.
"""
import argparse
import math
import os
import signal
import time


class KeyboardState:
    """Hold-to-move input, with a stop latch until all held keys are released."""
    KEYS = frozenset('wasd')

    def __init__(self, linear_speed=.40, angular_speed=.70):
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.held = set()
        self.space_held = False
        self.stopped = False

    def press(self, key):
        if key == 'space':
            self.space_held = True
            self.stopped = True
        elif key in self.KEYS and key not in self.held:
            if not self.held and not self.space_held:
                self.stopped = False
            self.held.add(key)

    def release(self, key):
        if key == 'space':
            self.space_held = False
        self.held.discard(key)

    def stop(self):
        self.stopped = True

    def reset(self):
        self.held.clear()
        self.space_held = False
        self.stopped = True

    def velocity(self):
        if self.stopped or self.space_held:
            return 0., 0.
        return (self.linear_speed * (('w' in self.held) - ('s' in self.held)),
                self.angular_speed * (('a' in self.held) - ('d' in self.held)))


def speed(value):
    result = float(value)
    if not math.isfinite(result) or not 0. < result <= .7:
        raise argparse.ArgumentTypeError('Speed must be finite and in (0, 0.7]')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description='WASD hold-to-move window for PM01 Gazebo mapping')
    parser.add_argument('--linear-speed', type=speed, default=.40, help='W/S speed in m/s (default: 0.40)')
    parser.add_argument('--angular-speed', type=speed, default=.70, help='A/D speed in rad/s (default: 0.70)')
    args = parser.parse_args(argv)
    try:
        from PyQt5 import QtCore, QtWidgets
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from geometry_msgs.msg import Twist
        from diagnostic_msgs.msg import DiagnosticArray
        from std_srvs.srv import Empty
    except ImportError as error:
        parser.exit(2, f'Simulation dependencies missing: {error}. Run scripts/install_dependencies.sh sim.\n')

    rclpy.init(args=[])
    node = Node('pm01_sim_keyboard_' + str(os.getpid()), parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    pub = node.create_publisher(Twist, '/cmd_vel', 1)
    reset_client = node.create_client(Empty, '/pm01/reset_stand')
    state = KeyboardState(args.linear_speed, args.angular_speed)
    output = {'enabled': False, 'received': None}

    def output_status(msg):
        for item in msg.status:
            if item.name == 'pm01_motion_output':
                values = {v.key: v.value for v in item.values}
                output.update(enabled=values.get('state') == '"enabled"' and
                              values.get('output_process_running') == 'true', received=time.monotonic())

    subscription = node.create_subscription(DiagnosticArray, '/pm01/navigation/motion_status', output_status, 1)
    app = QtWidgets.QApplication([])

    class Window(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.setWindowTitle('PM01 Simulation Keyboard')
            self.setFocusPolicy(QtCore.Qt.StrongFocus)
            self.setMinimumWidth(500)
            layout = QtWidgets.QVBoxLayout(self)
            title = QtWidgets.QLabel('PM01 仿真键盘控制')
            title.setStyleSheet('font-size: 24px; font-weight: bold;')
            layout.addWidget(title)
            layout.addWidget(QtWidgets.QLabel('仅用于 mapping 建图 · 点击本窗口，按住移动，松开停车'))
            grid = QtWidgets.QGridLayout()
            self.keys = {}
            for key, label, row, col in [('w', 'W  前进', 0, 1), ('a', 'A  左转', 1, 0),
                                         ('s', 'S  后退', 1, 1), ('d', 'D  右转', 1, 2)]:
                box = QtWidgets.QLabel(label)
                box.setAlignment(QtCore.Qt.AlignCenter)
                box.setMinimumSize(130, 65)
                self.keys[key] = box
                grid.addWidget(box, row, col)
            layout.addLayout(grid)
            stop = QtWidgets.QPushButton('空格 / 点击停车')
            stop.setFocusPolicy(QtCore.Qt.NoFocus)
            stop.setMinimumHeight(45)
            stop.clicked.connect(self.brake)
            layout.addWidget(stop)
            layout.addWidget(QtWidgets.QLabel(
                f'前后速度 {args.linear_speed:.2f} m/s · 转向速度 {args.angular_speed:.2f} rad/s'))
            layout.addWidget(QtWidgets.QLabel('失去窗口焦点或退出会归零；停车后松开方向键，再按下继续。'))
            self.status = QtWidgets.QLabel('等待仿真 /clock 和运动输出状态…')
            self.status.setWordWrap(True)
            layout.addWidget(self.status)
            self.started = time.monotonic()
            self.last_graph = 0.
            self.conflict = False
            self.simulation = False
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.tick)
            self.timer.start(50)
            app.applicationStateChanged.connect(self.application_state)

        def publish(self, vx=0., wz=0.):
            if rclpy.ok() and self.simulation and not self.conflict:
                msg = Twist()
                msg.linear.x, msg.angular.z = vx, wz
                pub.publish(msg)

        def brake(self):
            state.stop()
            self.publish()

        def application_state(self, active):
            if active != QtCore.Qt.ApplicationActive:
                state.reset()
                self.publish()

        def focusOutEvent(self, event):
            state.reset()
            self.publish()
            super().focusOutEvent(event)

        def changeEvent(self, event):
            if event.type() == QtCore.QEvent.ActivationChange and self.isActiveWindow():
                self.setFocus(QtCore.Qt.ActiveWindowFocusReason)
            super().changeEvent(event)

        def keyPressEvent(self, event):
            if not self.isActiveWindow():
                return
            # X11 can mark a fresh press as repeat if the previous release
            # occurred in another window. KeyboardState already deduplicates
            # held presses and preserves the space-stop latch.
            if event.key() == QtCore.Qt.Key_Space:
                state.press('space')
                self.publish()
            elif event.key() == QtCore.Qt.Key_Escape:
                self.close()
            elif event.key() == QtCore.Qt.Key_C and event.modifiers() & QtCore.Qt.ControlModifier:
                self.close()
            elif event.text().lower() in KeyboardState.KEYS:
                state.press(event.text().lower())
            else:
                super().keyPressEvent(event)

        def keyReleaseEvent(self, event):
            if event.isAutoRepeat():
                return
            key = 'space' if event.key() == QtCore.Qt.Key_Space else event.text().lower()
            state.release(key)
            if state.velocity() == (0., 0.):
                self.publish()
            super().keyReleaseEvent(event)

        def tick(self):
            if not rclpy.ok():
                self.close()
                return
            # Drain both /clock and status without blocking the GUI, even when
            # simulated clock messages arrive much faster than this timer.
            for _ in range(4):
                executor.spin_once(timeout_sec=0.)
            now = time.monotonic()
            if now - self.last_graph >= .25:
                self.last_graph = now
                peers = [p for p in node.get_publishers_info_by_topic('/cmd_vel')
                         if (p.node_name, p.node_namespace) != (node.get_name(), node.get_namespace())]
                self.conflict = bool(peers)
            simulation = node.get_clock().now().nanoseconds > 0 and reset_client.service_is_ready()
            self.simulation = simulation
            enabled = output['enabled'] and output['received'] is not None and now-output['received'] < 1.
            if self.conflict:
                # Do not keep sending zeros over another controller's commands.
                state.reset()
                self.status.setText('检测到其他 /cmd_vel 发布者。请退出键盘控制，使用仅建图模式。')
            elif not self.isActiveWindow():
                state.reset()
                self.publish()
                self.status.setText('窗口未获得焦点；点击本窗口后用 WASD 控制。')
            elif not simulation or not enabled or now-self.started < 1.:
                state.stop()
                self.publish()
                self.status.setText('等待 Gazebo 仿真…' if not simulation else
                                    '运动输出未接通；请先执行 pm01_nav enable-motion。')
            else:
                vx, wz = state.velocity()
                self.publish(vx, wz)
                self.status.setText(f'输出速度：前后 {vx:+.2f} m/s · 转向 {wz:+.2f} rad/s')
            for key, box in self.keys.items():
                color = '#c8e6c9' if key in state.held and not state.stopped else '#eeeeee'
                box.setStyleSheet(f'background: {color}; border: 1px solid #bbbbbb; border-radius: 6px; font-size: 18px;')

        def closeEvent(self, event):
            self.timer.stop()
            state.reset()
            self.publish()
            event.accept()

    window = None
    previous = {}
    try:
        window = Window()
        window.show()
        window.setFocus()
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.signal(sig, lambda *_: window.close())
        return app.exec_()
    finally:
        state.reset()
        if window:
            window.publish()
        # Give reliable DDS a bounded opportunity to send the final zero.
        if rclpy.ok():
            executor.spin_once(timeout_sec=.05)
        executor.shutdown()
        node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
