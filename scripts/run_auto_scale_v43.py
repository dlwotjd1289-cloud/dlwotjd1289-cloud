#!/usr/bin/env python3
"""V4.3 commissioning control node. Requires ROS 2 Humble + ros_gz_bridge.

This is a ground-truth *test harness*, not a production perception pipeline.
"""
from __future__ import annotations

import signal
import sys
import time
from pathlib import Path

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

sys.path.insert(0, str(Path(__file__).resolve().parent))
from scale_cycle_core_v43 import AutoScaleCycle, Config, Phase


# Human-readable progress shown once per phase change.
STATUS = {
    Phase.TARE: "[1/6] 빈 저울 영점 측정 중...",
    Phase.WAIT_BOX: "[2/6] 박스 투입 대기 중...",
    Phase.TO_SCALE: "[3/6] 박스를 저울로 이송 중...",
    Phase.SETTLING: "[4/6] 저울 위 정지, 무게 안정화·계량 중...",
    Phase.TO_PICK: "[5/6] 계량 완료, PICK 위치로 이송 중 (저울 영점 복귀 확인)...",
    Phase.DONE: "[6/6] 완료: PICK 위치 도착, 컨베이어 정지",
    Phase.ERROR: "[중단] 오류로 안전 정지",
}


class ScaleController(Node):
    def __init__(self) -> None:
        # Simulation time: timeouts must not expire when Gazebo runs slower than
        # real time (observed real-time factor ~0.2 under CPU load).
        super().__init__("pac_auto_scale_v43",
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.core = None  # created on the first /clock time
        self.publisher = self.create_publisher(Float64, "/pac/conveyor/roller_cmd_vel", 10)
        self.create_subscription(WrenchStamped, "/pac/scale/wrench", self.on_wrench, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/model/v43_scale_box_5kg/pose", self.on_pose, qos_profile_sensor_data)
        self.create_timer(0.1, self.tick)
        self.done = False
        self._last_velocity = None
        self._last_phase = None
        self.get_logger().info("Waiting for empty-scale tare, then demo-box pose. Gazebo must be UNPAUSED.")

    def sim_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_wrench(self, msg: WrenchStamped) -> None:
        if self.core is not None:
            self.core.update_wrench(msg.wrench.force.z, self.sim_now())

    def on_pose(self, msg: PoseStamped) -> None:
        if self.core is not None:
            self.core.update_pose(msg.pose.position.x, msg.pose.position.y, self.sim_now())

    def stop_rollers(self):
        msg = Float64()
        msg.data = 0.0
        for _ in range(4):
            self.publisher.publish(msg)

    def tick(self):
        now = self.sim_now()
        if now <= 0.0:
            return  # no /clock yet
        if self.core is None:
            self.core = AutoScaleCycle(Config(), now=now)
        velocity = self.core.tick(now)
        cmd = Float64()
        cmd.data = float(velocity)
        self.publisher.publish(cmd)  # periodically resend to survive discovery races
        if self.core.phase != self._last_phase:
            print(f">>> {STATUS[self.core.phase]}", flush=True)
            self._last_phase = self.core.phase
        if velocity != self._last_velocity:
            self.get_logger().info(f"ROLLER CMD: {velocity:.2f} rad/s")
            self._last_velocity = velocity
        for msg in self.core.messages:
            if self.core.phase == Phase.ERROR:
                self.get_logger().error(msg)
            else:
                self.get_logger().info(msg)
        self.core.messages.clear()
        if self.core.phase in (Phase.DONE, Phase.ERROR):
            self.done = True


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main() -> int:
    # Handle Ctrl+C / SIGTERM here so rollers are commanded zero while the
    # ROS context is still valid (the runner stops this node with SIGTERM).
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_interrupt)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = ScaleController()
    try:
        start = time.monotonic()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.core is None and time.monotonic() - start > 10.0:
                print("\nV4.3 FAIL: no /clock from Gazebo (10 s wall time).", flush=True)
                node.stop_rollers()
                return 1
        node.stop_rollers()
        if node.core is not None and node.core.phase == Phase.DONE:
            core = node.core
            print(f"\nV4.3 PASS: {core.measured_kg:.3f} kg; empty-scale restored "
                  f"(residual {core.unload_residual_kg:+.3f} kg, tare {core.baseline_n:.3f} N); "
                  f"PICK zone reached at x={core.pose_x_m:.3f} m.", flush=True)
            return 0
        print(f"\nV4.3 FAIL: {(node.core and node.core.error) or 'stopped'}", flush=True)
        return 1
    except KeyboardInterrupt:
        node.stop_rollers()
        print("\nStopped by user; rollers commanded zero.", flush=True)
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
