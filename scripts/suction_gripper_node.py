#!/usr/bin/env python3
"""Suction-cup gripper logic for the V4.4 Gazebo workcell (ROS 2 Humble).

Interface (for scripts now, MoveIt2 pick/place later):
  /pac/suction/vacuum  std_msgs/Bool    command: true = vacuum on, false = release
  /pac/suction/state   std_msgs/String  OFF | SEARCHING | GRIPPED (published at 5 Hz)

Gazebo side (bridged by run_suction_gripper.sh):
  /pac/suction/touched std_msgs/Bool  <- TouchPlugin: cup touched the box for 0.1 s
  TouchPlugin enable is a Gazebo *service* (/pac/suction/enable, Boolean -> no
  reply) that ros_gz_bridge (Humble) cannot bridge; it is called with `ign service`.
  /pac/gripper/attach, /pac/gripper/detach std_msgs/Empty -> DetachableJoint

A box is gripped only while vacuum is on AND the cup is in contact, like a
real suction cup; it is not attached at a distance.
"""
from __future__ import annotations

import signal
import subprocess
import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Empty, String


class SuctionGripper(Node):
    def __init__(self) -> None:
        super().__init__("pac_suction_gripper")
        self.state = "OFF"
        self.attach_pub = self.create_publisher(Empty, "/pac/gripper/attach", 10)
        self.detach_pub = self.create_publisher(Empty, "/pac/gripper/detach", 10)
        self.state_pub = self.create_publisher(String, "/pac/suction/state", 10)
        self.create_subscription(Bool, "/pac/suction/vacuum", self.on_vacuum, 10)
        self.create_subscription(Bool, "/pac/suction/touched", self.on_touched, 10)
        self.create_timer(0.2, self.publish_state)
        self.create_timer(1.0, self.keep_enabled)
        self.get_logger().info("Suction gripper ready (state OFF).")

    def _set(self, state: str) -> None:
        if state != self.state:
            self.state = state
            self.get_logger().info(f"Suction state -> {state}")
            print(f">>> [흡착] {dict(OFF='진공 꺼짐(해제)', SEARCHING='진공 켬, 박스 접촉 대기 중...', GRIPPED='박스 흡착 완료')[state]}", flush=True)
        self.publish_state()

    def publish_state(self) -> None:
        self.state_pub.publish(String(data=self.state))

    @staticmethod
    def touch_enable(on: bool) -> None:
        # One-way service: no reply, so `ign service` times out by design; do not wait.
        subprocess.Popen(
            ["ign", "service", "-s", "/pac/suction/enable", "--reqtype", "ignition.msgs.Boolean",
             "--reptype", "ignition.msgs.Empty", "--timeout", "300", "--req", f"data: {str(on).lower()}"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def keep_enabled(self) -> None:
        # TouchPlugin disables itself after reporting; re-arm while searching.
        if self.state == "SEARCHING":
            self.touch_enable(True)

    def on_vacuum(self, msg: Bool) -> None:
        if msg.data and self.state == "OFF":
            self.touch_enable(True)
            self._set("SEARCHING")
        elif not msg.data and self.state != "OFF":
            self.touch_enable(False)
            for _ in range(3):
                self.detach_pub.publish(Empty())
            self._set("OFF")

    def on_touched(self, msg: Bool) -> None:
        if msg.data and self.state == "SEARCHING":
            for _ in range(3):
                self.attach_pub.publish(Empty())
            self._set("GRIPPED")


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main() -> int:
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_interrupt)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = SuctionGripper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
