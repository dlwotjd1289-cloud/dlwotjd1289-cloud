#!/usr/bin/env python3
"""Reset after an interrupted V4.4 MoveIt run: release the suction gripper in
Gazebo, remove the test box from the MoveIt planning scene, plan back home.
Requires Gazebo V4.4 and hdr50_moveit_v44.launch.py running."""
import subprocess
import sys

import rclpy
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from rclpy.signals import SignalHandlerOptions

from moveit_pick_place_v44 import BOX_ID, BOX_SIZE, HOME, SCENE, TCP, Failure, MoveItPickPlace, box_object


def main():
    print(">>> [리셋 1/3] 흡착 그리퍼 해제 중...", flush=True)
    for _ in range(3):
        subprocess.run(["ign", "topic", "-t", "/pac/gripper/detach", "-m", "ignition.msgs.Empty", "-p", " "],
                       check=False, timeout=10)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MoveItPickPlace()
    try:
        if not node.move_ac.wait_for_server(timeout_sec=15.0) or not node.scene_cli.wait_for_service(timeout_sec=15.0):
            raise Failure("move_group not available (start hdr50_moveit_v44.launch.py)")
        print(">>> [리셋 2/3] planning scene에서 박스 제거 중...", flush=True)
        node.apply_scene(attached=[AttachedCollisionObject(
            link_name=TCP, object=box_object(BOX_ID, (0, 0, 0), BOX_SIZE, op=CollisionObject.REMOVE))], check=False)
        node.apply_scene(objects=[box_object(BOX_ID, (0, 0, 0), BOX_SIZE, op=CollisionObject.REMOVE)], check=False)
        node.apply_scene(objects=[box_object(n, c, s) for n, c, s in SCENE])
        print(">>> [리셋 3/3] 홈 자세로 복귀 중 (MoveIt)...", flush=True)
        node.plan_joints(HOME, "return home")
        print("RESET DONE: gripper released, robot at home.", flush=True)
        return 0
    except Failure as e:
        print(f"RESET FAIL: {e}", flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
