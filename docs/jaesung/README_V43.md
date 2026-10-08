# PAC2026 V4.3 — Auto weighing commissioning

**V4.2 Gazebo world remains unchanged.** V4.3 consists of a stand-alone ROS 2 control/test harness and a uniquely named test box with Gazebo PosePublisher. This is an automation smoke test, not a real perception system.

## Target behavior

`TARE(7kg) → BOX_DETECTED → ROLLERS_ON → SCALE_CENTER → ROLLERS_OFF → FORCE/TORQUE STABLE → MEASURE → ROLLERS_ON → SCALE_ZERO → PICK_ZONE → ROLLERS_OFF`

- Measured mass is derived only from the force sensor (`(Fz - tare)/g`), **not from box SDF mass**.
- The test compares the result with the known 5.0 kg fixture only as a QA assertion.
- Test-box position is **Gazebo ground truth for commissioning**, not a simulated visual perception claim.
- All missing data / failed settling causes a **safe stop**.
- Physical stopper and robot/EOAT are unchanged.
- Supports one test box, one active scale, one automatic cycle. Not a multi-box scheduler.

## Install into Ubuntu (Terminal 4)

Unzip the supplied archive into the *existing* project root. Do not replace any V4.2 file.

```bash
cd ~/AHEAD/pac2026_hdr50_proxy_scaffold
unzip -o ~/다운로드/PAC2026_V43_AutoScale.zip -d .
python3 -m unittest discover -s tests -p 'test_auto_scale_v43.py' -v
```

## Run

Terminal 1: stop Gazebo if already running; restart the **V4.2** world and press ▶ to unpause:

```bash
cd ~/AHEAD/pac2026_hdr50_proxy_scaffold/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pac_bringup hdr50_workcell_v4_2.launch.py
```

Ensure the scale is empty and `v43_scale_box_5kg` does not exist in this fresh simulation. Leave Terminal 1 running.

Terminal 4: launch bridge + auto-controller + spawn test box **with a single command**:

```bash
cd ~/AHEAD/pac2026_hdr50_proxy_scaffold
bash scripts/run_auto_scale_v43.sh
```

Expect logs like:

```
[TARE] Tare = 68.647 N
[WAIT_BOX] ...
[TO_SCALE] Incoming box detected
[SETTLING] Stop and stabilize
[TO_PICK] WEIGHED 5.000 kg
[TO_PICK] Scale unloaded
[DONE] PICK zone reached
V4.3 PASS
```

If a failure occurs, rollers are commanded to zero. Report the Terminal 4 output and the newest `logs/v43_scale_bridge_<timestamp>.log`.

Repeated runs in the same Gazebo session are supported: the runner removes only its own leftover `v43_scale_box_5kg` before the empty-scale tare. Ctrl+C stops the controller first, commands zero roller speed in Gazebo, then stops the bridge (no orphan processes).

Verified 2026-10-08 in Gazebo (Fortress 6.18): three consecutive PASS cycles with 5.000 kg measured from `/pac/scale/wrench`. The test box needs `publish_nested_model_pose=true`; with `false`, Gazebo advertises `/model/v43_scale_box_5kg/pose` but never publishes it.

## Gazebo / ROS bridge topics

| Direction | Gazebo topic | ROS message |
|---|---|---|
| GZ→ROS | `/pac/scale/wrench` (`ignition.msgs.Wrench`) | `geometry_msgs/msg/WrenchStamped` |
| GZ→ROS | `/model/v43_scale_box_5kg/pose` (`ignition.msgs.Pose`) | `geometry_msgs/msg/PoseStamped` |
| ROS→GZ | `/pac/conveyor/roller_cmd_vel` (`ignition.msgs.Double`) | `std_msgs/msg/Float64` |

The runner starts the `ros_gz_bridge` `parameter_bridge` executable directly (already used for `/clock` in the workcell). Bridge log: `logs/v43_scale_bridge_<timestamp>.log`.

### Known limitations and cautions

1. Weighing precision depends on the box fully resting on the instrumented four rollers. If the box stops too early or too late, it must **fail**, not pretend to measure 5.00 kg.
2. The position sensor is a debug-only truth source attached to the test model; replace with laser/photocell, encoder, or camera-based detection for real control.
3. If the model uses a different name/topic, the bridge must match; fail closed otherwise.
4. This exact end-to-end test has **not** been executed on your Ubuntu Gazebo yet. Offline state-machine unit tests cover logic only. Validate real ROS/Gazebo behavior before calling V4.3 complete.
5. The script uses the current V4.2 world. It intentionally does not modify Git PR #1 or the Hyundai dependencies and does not create a V4.3 SDF/launch.

Official component references: Gazebo 6 PosePublisher, Gazebo Fortress ROS bridge Wrench/Pose/Double conversions.
