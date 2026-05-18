#!/usr/bin/env python3
"""
Fork pozisyon kontrolü (PD + gravity compensation).

Strateji:
  - /joint_states'ten fork pozisyonunu okur
  - Kullanıcı u/d ile target_position'ı değiştirir
  - 20 Hz kontrol döngüsü PD + gravity comp ile effort hesaplar
  - Her döngü: clear_joint_efforts → apply_joint_effort (continuous)

Klavye:
  u / U  → çatal yukarı (+2 cm hedef)
  d / D  → çatal aşağı  (−2 cm hedef)
  q      → çıkış
"""

import sys
import tty
import termios
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from gazebo_msgs.srv import ApplyJointEffort, JointRequest
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration, Time

JOINT = 'fork_carriage_joint'
STEP = 0.02
MIN_POS = 0.0
MAX_POS = 0.30

# PD kazançları (mass=3.1 kg, damping=200 ile uyumlu)
KP = 1500.0   # N/m
KD = 400.0    # N·s/m
GRAV_COMP = 32.0     # N – fork+carriage ağırlığı (~3.1 kg × 9.81)
EFFORT_MIN = -300.0
EFFORT_MAX = 500.0

CTRL_RATE_HZ = 50.0


class ForkTeleop(Node):
    def __init__(self):
        super().__init__('fork_teleop')
        cb = MutuallyExclusiveCallbackGroup()

        self.apply_cli = self.create_client(
            ApplyJointEffort, '/apply_joint_effort', callback_group=cb)
        self.clear_cli = self.create_client(
            JointRequest, '/clear_joint_efforts', callback_group=cb)

        self.create_subscription(
            JointState, '/joint_states', self._on_state, 10, callback_group=cb)

        self.position = 0.0
        self.velocity = 0.0
        self.target = 0.0
        self._first_apply_logged = False
        self._control_active = False

        self.get_logger().info('Servisler bekleniyor...')
        if not self.apply_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('apply_joint_effort yok – Gazebo çalışıyor mu?')
            return
        if not self.clear_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('clear_joint_efforts yok')

        # Kontrol döngüsü
        self.create_timer(1.0 / CTRL_RATE_HZ, self._control_step, callback_group=cb)
        self._control_active = True
        self.get_logger().info(
            f'Hazır. KP={KP}  KD={KD}  GRAV={GRAV_COMP}N. u=yukarı  d=aşağı  q=çıkış')

    # /joint_states callback
    def _on_state(self, msg: JointState):
        if JOINT not in msg.name:
            return
        i = msg.name.index(JOINT)
        self.position = msg.position[i] if i < len(msg.position) else 0.0
        self.velocity = msg.velocity[i] if i < len(msg.velocity) else 0.0

    # 50 Hz kontrol – clear, sonra apply (sürekli effort)
    def _control_step(self):
        if not self._control_active:
            return

        err = self.target - self.position
        effort = KP * err - KD * self.velocity + GRAV_COMP
        effort = max(EFFORT_MIN, min(EFFORT_MAX, effort))

        cr = JointRequest.Request()
        cr.joint_name = JOINT
        future = self.clear_cli.call_async(cr)
        future.add_done_callback(
            lambda f, e=effort: self._apply_after_clear(e))

    def _apply_after_clear(self, effort: float):
        req = ApplyJointEffort.Request()
        req.joint_name = JOINT
        req.effort = float(effort)
        req.start_time = Time(sec=0, nanosec=0)
        # duration < 0 → süresiz; her döngüde clear ile yenilenir
        req.duration = Duration(sec=-1, nanosec=0)
        future = self.apply_cli.call_async(req)
        if not self._first_apply_logged:
            future.add_done_callback(self._log_first_response)

    def _log_first_response(self, future):
        try:
            res = future.result()
            self.get_logger().info(
                f'İlk apply yanıtı: success={res.success}  msg="{res.status_message}"')
        except Exception as e:
            self.get_logger().error(f'apply yanıt hatası: {e}')
        self._first_apply_logged = True

    def up(self):
        self.target = min(self.target + STEP, MAX_POS)
        self.get_logger().info(
            f'target={self.target*100:5.1f}cm   pos={self.position*100:5.1f}cm')

    def down(self):
        self.target = max(self.target - STEP, MIN_POS)
        self.get_logger().info(
            f'target={self.target*100:5.1f}cm   pos={self.position*100:5.1f}cm')


def _get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def main(args=None):
    rclpy.init(args=args)
    node = ForkTeleop()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('\n=== Fork Teleop (PD + gravity comp) ===')
    print('  u / U  → çatal yukarı  (+2 cm)')
    print('  d / D  → çatal aşağı   (−2 cm)')
    print('  q      → çıkış\n')

    try:
        while rclpy.ok():
            key = _get_key()
            if key in ('u', 'U'):
                node.up()
            elif key in ('d', 'D'):
                node.down()
            elif key == 'q':
                break
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
