#!/usr/bin/env python3
"""Teleoperazione da tastiera dei giunti di Armando (j0..j3).

Pubblica una JointTrajectory a un punto su
/joint_trajectory_controller/joint_trajectory ad ogni pressione di tasto.
"""
import os
import select
import sys
import termios
import time
import tty

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = ['j0', 'j1', 'j2', 'j3']
LOWER, UPPER = -3.14, 3.14      # limiti dei giunti nell'URDF [rad]
STEP = 0.1                      # incremento per pressione di tasto [rad]
MOVE_TIME_NS = 300_000_000      # tempo per raggiungere il target [0.3 s]

# tasto -> (indice del giunto, direzione)
KEY_MAP = {
    'q': (0, +1), 'a': (0, -1),
    'w': (1, +1), 's': (1, -1),
    'e': (2, +1), 'd': (2, -1),
    'r': (3, +1), 'f': (3, -1),
}

HELP = """
Teleoperazione Armando
----------------------
  j0: q (+)  a (-)
  j1: w (+)  s (-)
  j2: e (+)  d (-)
  j3: r (+)  f (-)
  z : torna alla posa zero
  x : esci (anche Ctrl-C)
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('armando_keyboard_teleop')
        self.pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.target = [0.0] * len(JOINT_NAMES)
        self.synced = False

    def joint_state_cb(self, msg):
        # Legge la posa reale una sola volta, cosi' il primo comando
        # parte dalla configurazione attuale e non da zero.
        if self.synced:
            return
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINT_NAMES):
            self.target = [pos[j] for j in JOINT_NAMES]
            self.synced = True
            self.get_logger().info('Posa iniziale letta da /joint_states')

    def wait_for_state(self, timeout=3.0):
        start = time.monotonic()
        while rclpy.ok() and not self.synced:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start > timeout:
                self.get_logger().warn(
                    'Nessun /joint_states ricevuto: parto da zero')
                break

    def publish_target(self):
        point = JointTrajectoryPoint()
        point.positions = list(self.target)
        point.time_from_start = Duration(sec=0, nanosec=MOVE_TIME_NS)
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        msg.points = [point]
        self.pub.publish(msg)

    def move_joint(self, index, direction):
        new = self.target[index] + direction * STEP
        self.target[index] = min(max(new, LOWER), UPPER)
        self.publish_target()
        self.print_state()

    def go_home(self):
        self.target = [0.0] * len(JOINT_NAMES)
        self.publish_target()
        self.print_state()

    def print_state(self):
        values = '  '.join(f'{n}={v:+.2f}' for n, v in zip(JOINT_NAMES, self.target))
        print(values)


def read_key(timeout):
    """Legge un carattere da stdin senza bloccare oltre `timeout` secondi."""
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if ready:
        return os.read(sys.stdin.fileno(), 1).decode(errors='ignore')
    return ''


def main():
    rclpy.init()
    node = KeyboardTeleop()
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        node.wait_for_state()
        tty.setcbreak(fd)  # un tasto alla volta, senza Invio; Ctrl-C resta attivo
        print(HELP)
        node.print_state()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key(0.05)
            if key in KEY_MAP:
                index, direction = KEY_MAP[key]
                node.move_joint(index, direction)
            elif key == 'z':
                node.go_home()
            elif key == 'x':
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
