#!/usr/bin/env python3
"""Write a Gazebo box model (SDF) for a generator SKU: size [m], mass [kg] -> OUT.

Same structure as test_data/scale_auto_box_5kg_v43.sdf (link box_link, cardboard colour,
friction 0.9, PosePublisher with publish_nested_model_pose=true); the model name is set at
spawn time (ros_gz_sim create -name box_NN). Prints the spawn z (box resting 3 cm above rollers).
  python3 scripts/make_box_sdf_v44.py --size 0.35 0.25 0.15 --mass 5.64 --out /tmp/box.sdf
"""
import argparse

ROLLER_TOP_Z = 0.895

TEMPLATE = """<?xml version="1.0"?>
<sdf version="1.8">
  <model name="generator_box">
    <static>false</static>
    <link name="box_link">
      <inertial>
        <mass>{m:.6f}</mass>
        <inertia><ixx>{ixx:.8f}</ixx><iyy>{iyy:.8f}</iyy><izz>{izz:.8f}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{x:.4f} {y:.4f} {z:.4f}</size></box></geometry>
        <surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface>
      </collision>
      <visual name="visual">
        <geometry><box><size>{x:.4f} {y:.4f} {z:.4f}</size></box></geometry>
        <material><ambient>0.55 0.32 0.12 1</ambient><diffuse>0.75 0.48 0.22 1</diffuse></material>
      </visual>
    </link>
    <!-- Dev/test-only ground-truth pose (perception uses the cameras). Gazebo 6 publishes the
         model's own pose only with publish_nested_model_pose=true. -->
    <plugin filename="ignition-gazebo-pose-publisher-system" name="gz::sim::systems::PosePublisher">
      <publish_model_pose>true</publish_model_pose>
      <publish_link_pose>false</publish_link_pose>
      <publish_collision_pose>false</publish_collision_pose>
      <publish_visual_pose>false</publish_visual_pose>
      <publish_nested_model_pose>true</publish_nested_model_pose>
      <update_frequency>30</update_frequency>
      <use_pose_vector_msg>false</use_pose_vector_msg>
    </plugin>
  </model>
</sdf>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"))
    ap.add_argument("--mass", type=float, required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    x, y, z = a.size
    m = a.mass
    open(a.out, "w").write(TEMPLATE.format(m=m, x=x, y=y, z=z, ixx=m * (y * y + z * z) / 12,
                                           iyy=m * (x * x + z * z) / 12, izz=m * (x * x + y * y) / 12))
    print(f"{ROLLER_TOP_Z + z / 2 + 0.03:.4f}")


if __name__ == "__main__":
    main()
