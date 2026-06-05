"""Play two retargeted X2 CSVs side-by-side in a single MuJoCo viewer window.

Usage:
    python scripts/compare_two_csvs.py \
        --csv-a scratch/csv/A021__h1.70_baseline.csv \
        --csv-b scratch/csv/A021__FINAL_v2.csv \
        --label-a OLD \
        --label-b NEW

The two robots are placed 1.5 m apart along the Y axis (left/right of the
camera) so you can scrub through the motion and visually compare. The CSV
format is `AgibotX2Ultra31DOF_CSVConfig` from soma_retargeter/assets/csv.py:
columns 1..6 = root translate (cm) + root euler (deg), columns 7.. = joint
DOFs in degrees, in the X2 joint order.
"""

from __future__ import annotations

import argparse
import math
import re
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation as R


CSV_JOINT_ORDER = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint",
    "head_yaw_joint", "head_pitch_joint",
]


def load_csv(path: Path) -> np.ndarray:
    """Load a CSV and return an (N, 7 + 31) array.

    Per-row layout: [tx_m, ty_m, tz_m, qw, qx, qy, qz, q0..q30] (radians).
    Translations come from cm; rotations come from XYZ euler in deg.
    """
    raw = np.loadtxt(path, delimiter=",", skiprows=1, dtype=np.float64)
    n = raw.shape[0]
    out = np.zeros((n, 7 + 31), dtype=np.float64)
    out[:, 0:3] = raw[:, 1:4] * 0.01  # cm -> m
    euler_xyz_deg = raw[:, 4:7]
    quats_wxyz = R.from_euler("xyz", euler_xyz_deg, degrees=True).as_quat()  # [x,y,z,w]
    out[:, 3] = quats_wxyz[:, 3]  # w
    out[:, 4:7] = quats_wxyz[:, 0:3]  # xyz
    out[:, 7:] = np.deg2rad(raw[:, 7:])
    return out


def build_world_xml(robot_xml_path: Path, offset_a_y: float, offset_b_y: float,
                    label_a: str, label_b: str) -> str:
    """Wrap two copies of the X2 MJCF into a single world with horizontal offsets.

    We do this by writing an inline scene MJCF that includes the X2 model
    twice with `<replicate>`-style position offsets, using prefix renaming.
    """
    robot_xml_path = robot_xml_path.resolve()
    xml = f"""<mujoco model="x2_compare">
  <compiler angle="radian" eulerseq="XYZ" meshdir="{robot_xml_path.parent / 'meshes'}" autolimits="true"/>
  <option timestep="0.002"/>
  <visual>
    <headlight diffuse="0.7 0.7 0.7" ambient="0.3 0.3 0.3" specular="0.5 0.5 0.5"/>
    <global azimuth="180" elevation="-15"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.3 0.52 0.63" rgb2="0.3 0.52 0.63" markrgb="1 1 1" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>
  <worldbody>
    <geom name="ground" type="plane" size="0 0 0.05" material="groundplane" condim="3"/>
    <light name="top" pos="0 0 3"/>
    <attach prefix="A_" body="A_pelvis" model="A_x2"/>
    <attach prefix="B_" body="B_pelvis" model="B_x2"/>
  </worldbody>
</mujoco>
"""
    return xml


def load_two_robots(robot_xml_path: Path, y_a: float, y_b: float):
    """Build a single MjModel containing two X2 robots side-by-side.

    The cleanest path is to read the original MJCF text, do a name-prefix
    rewrite for the second instance (so joints/bodies/sites are unique), and
    feed the merged XML to mujoco.MjModel.from_xml_string. We then use the
    `pos` attribute on each pelvis body to offset them.
    """
    xml_text = robot_xml_path.read_text()

    def rewrite(prefix: str, y_offset: float) -> str:
        text = xml_text
        # rename every body / joint / motor / geom / site / mesh by prefix
        for attr in ["name", "joint", "site"]:
            text = re.sub(rf'({attr}="[^"]+")',
                          lambda m: f'{attr}="{prefix}{m.group(1).split("=")[1][1:-1]}"', text)
        # offset the pelvis body
        text = re.sub(
            rf'(body name="{prefix}pelvis" pos=")[^"]+(")',
            lambda m: f'{m.group(1)}0 {y_offset} 0.68{m.group(2)}',
            text,
        )
        # also change model name
        text = re.sub(r'<mujoco model="[^"]+"',
                      f'<mujoco model="{prefix.rstrip("_")}_x2"', text)
        return text

    # Easier path: load each copy as its own MjModel, then merge via mjcf
    # python attach. But mujoco-py 3.x doesn't expose attach the same way.
    # Use the standalone approach: write two parallel MJCFs to disk.
    return None  # unused


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv-a", required=True, type=Path)
    parser.add_argument("--csv-b", required=True, type=Path)
    parser.add_argument("--label-a", default="A")
    parser.add_argument("--label-b", default="B")
    parser.add_argument("--mjcf",
                        default=str(Path(__file__).resolve().parents[1] /
                                    "soma_retargeter/robot_assets/agibot_x2_ultra/x2_ultra.xml"),
                        type=str)
    parser.add_argument("--fps", type=float, default=60.0)
    parser.add_argument("--side-by-side-y", type=float, default=1.4,
                        help="Y separation between the two robots (m)")
    args = parser.parse_args()

    data_a = load_csv(args.csv_a)
    data_b = load_csv(args.csv_b)
    n_frames = min(len(data_a), len(data_b))
    print(f"[{args.label_a}] {args.csv_a.name}: {len(data_a)} frames")
    print(f"[{args.label_b}] {args.csv_b.name}: {len(data_b)} frames")
    print(f"Playing {n_frames} frames at {args.fps} fps")

    # Build a merged MJCF: just write two copies with name-mangled prefixes.
    mjcf_text = Path(args.mjcf).read_text()
    meshdir = (Path(args.mjcf).parent / "meshes").resolve()

    def make_copy(prefix: str, y_offset: float) -> str:
        text = mjcf_text
        # Drop sensors and actuators blocks -- we only need kinematic playback.
        text = re.sub(r"<sensor>.*?</sensor>", "", text, flags=re.DOTALL)
        text = re.sub(r"<actuator>.*?</actuator>", "", text, flags=re.DOTALL)
        text = text.replace('meshdir="./meshes"', f'meshdir="{meshdir}"')

        # Prefix every name="..." / joint="..." / site="..." / mesh="..." /
        # childclass="..." / class="..." in both double- and single-quoted form,
        # tolerating optional whitespace around the '='.
        for attr in ["name", "joint", "site", "mesh", "childclass", "class"]:
            text = re.sub(
                rf'\b{attr}\s*=\s*"([^"\s]+)"',
                lambda m, a=attr: f'{a}="{prefix}{m.group(1)}"',
                text,
            )
            text = re.sub(
                rf"\b{attr}\s*=\s*'([^'\s]+)'",
                lambda m, a=attr: f"{a}='{prefix}{m.group(1)}'",
                text,
            )

        # Offset the pelvis body so the two robots are side-by-side.
        text = re.sub(
            rf'body name="{prefix}pelvis" pos="[^"]+"',
            f'body name="{prefix}pelvis" pos="0 {y_offset} 0.68"',
            text,
        )
        # rename model attribute
        text = re.sub(r'<mujoco model="[^"]+"',
                      f'<mujoco model="{prefix.rstrip("_")}_x2"', text)
        return text

    xml_a = make_copy("A_", -args.side_by_side_y / 2)
    xml_b = make_copy("B_", +args.side_by_side_y / 2)

    tmp_dir = Path(__file__).resolve().parents[1] / "scratch/compare_tmp"
    tmp_dir.mkdir(parents=True, exist_ok=True)
    p_a = tmp_dir / "A_x2.xml"
    p_b = tmp_dir / "B_x2.xml"
    p_a.write_text(xml_a)
    p_b.write_text(xml_b)

    # Build the world that loads both via <include>
    world_xml = f"""<mujoco model="x2_compare">
  <compiler angle="radian" eulerseq="XYZ" autolimits="true"/>
  <option timestep="0.002"/>
  <visual>
    <headlight diffuse="0.7 0.7 0.7" ambient="0.3 0.3 0.3" specular="0.5 0.5 0.5"/>
    <global azimuth="180" elevation="-15"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.3 0.52 0.63" rgb2="0.3 0.52 0.63" markrgb="1 1 1" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>
  <worldbody>
    <geom name="ground" type="plane" size="0 0 0.05" material="groundplane" condim="3"/>
    <light pos="0 0 3"/>
  </worldbody>
  <include file="{p_a}"/>
  <include file="{p_b}"/>
</mujoco>
"""
    world_path = tmp_dir / "world.xml"
    world_path.write_text(world_xml)

    model = mujoco.MjModel.from_xml_path(str(world_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    # Map csv joint name -> qpos index for each robot
    def qpos_indices(prefix: str):
        # The free joint occupies qpos[0:7] for that robot
        free_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{prefix}floating_base_joint")
        free_qadr = model.jnt_qposadr[free_jid]
        joint_addrs = {}
        for jn in CSV_JOINT_ORDER:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{prefix}{jn}")
            joint_addrs[jn] = int(model.jnt_qposadr[jid])
        return int(free_qadr), joint_addrs

    free_a, jaddr_a = qpos_indices("A_")
    free_b, jaddr_b = qpos_indices("B_")

    def apply_frame(frame_row, free_addr, jaddr):
        data.qpos[free_addr + 0] = frame_row[0]
        data.qpos[free_addr + 1] = frame_row[1]
        data.qpos[free_addr + 2] = frame_row[2]
        data.qpos[free_addr + 3] = frame_row[3]  # w
        data.qpos[free_addr + 4] = frame_row[4]
        data.qpos[free_addr + 5] = frame_row[5]
        data.qpos[free_addr + 6] = frame_row[6]
        for i, jn in enumerate(CSV_JOINT_ORDER):
            data.qpos[jaddr[jn]] = frame_row[7 + i]

    print(f"[{args.label_a}] at y = {-args.side_by_side_y/2:+.2f}")
    print(f"[{args.label_b}] at y = {+args.side_by_side_y/2:+.2f}")
    print("Use the MuJoCo viewer keyboard:")
    print("  SPACE = pause / resume")
    print("  Mouse drag = orbit, scroll = zoom")
    print()

    dt = 1.0 / args.fps
    with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
        frame = 0
        start = time.time()
        paused = False
        last_print = 0.0
        while viewer.is_running():
            if not paused:
                frame_a = data_a[frame % len(data_a)]
                # B starts at the same root y but shifted by side_by_side_y/2
                frame_b = data_b[frame % len(data_b)].copy()
                # the per-robot pelvis was already shifted via the XML; the CSV
                # root translation is in world-space relative to that pelvis offset
                # We need to add the offset so each robot stays in its own lane.
                # But applying the free joint qpos will overwrite the base offset.
                # So shift the y of each per-frame root by the offset.
                frame_a_local = frame_a.copy()
                frame_a_local[1] += -args.side_by_side_y / 2
                frame_b_local = frame_b.copy()
                frame_b_local[1] += +args.side_by_side_y / 2

                apply_frame(frame_a_local, free_a, jaddr_a)
                apply_frame(frame_b_local, free_b, jaddr_b)
                mujoco.mj_forward(model, data)

                viewer.sync()
                frame = (frame + 1) % n_frames
                if time.time() - last_print > 2.0:
                    print(f"frame {frame}/{n_frames}", end="\r")
                    last_print = time.time()
            time.sleep(max(0.0, dt - 0.001))


if __name__ == "__main__":
    main()
