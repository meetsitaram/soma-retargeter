"""G1 -> X2 Ultra retargeting via Cartesian-keypoint injection + arm joint-map.

A standalone retargeting method, alongside the SOMA->X2 recipes. It converts a
Unitree G1 retarget CSV (36 cols: root + 29 DOF) into an Agibot X2 Ultra CSV
(38 cols: root + 31 DOF), reusing the same Newton IK solver but driving it from
G1 forward-kinematics keypoints instead of a SOMA human source.

Pipeline (see docs/g1_to_x2.md for the full write-up + diagram):

  1. FK the G1 CSV -> world transforms of the 14 tracked bodies, in the X2
     ik_map effector order.
  2. Scale the keypoints about the clip-start floor point (pose + height +
     walk-trajectory) by `position_scale` (G1 1.70m -> X2 1.40m ~= 0.824).
  3. Inject them as `NewtonPipeline.input_targets` and run the X2 IK with a
     retargeter config whose ARM rotation weights are zeroed (arms are driven
     by the joint-map below, not weak IK rotation). This yields correct
     legs / torso / root and neutral arms.
  4. Overwrite the arm joints from the G1 CSV directly:
       * shoulder (pitch/roll/yaw) + elbow : baked per-joint linear fit X2 = a*G1 + b
       * wrist : FUNCTIONAL remap by physical axis, since the joint NAMES differ
                 (G1 wrist_roll == X2 wrist_yaw = pronation), with per-DOF sign,
                 clamped to X2 joint limits.

Configs (soma_retargeter/configs/agibot_x2_ultra/):
    g1_to_x2_ultra_retargeter_config.json  - IK config (arm r_weight = 0)
    g1_to_x2_ultra_calibration.json        - self-contained scale + fit + wrist remap
"""

from __future__ import annotations

import os
from pathlib import Path

import mujoco
import newton
import numpy as np
from scipy.spatial.transform import Rotation as R

import soma_retargeter.assets.bvh as bvh_utils
import soma_retargeter.assets.csv as csv_utils
import soma_retargeter.utils.io_utils as io_utils

from . import newton_pipeline

_PKG = Path(__file__).resolve().parents[1]  # soma_retargeter/
_CFG_DIR = _PKG / "configs" / "agibot_x2_ultra"
_SOMA_INIT_BVH = str(_PKG / "configs" / "soma" / "soma_zero_frame0.bvh")
_X2_MJCF = str(_PKG / "robot_assets" / "agibot_x2_ultra" / "x2_ultra.xml")

G1_NUM_DOF = 29
X2_NUM_DOF = 31

# The 14 X2 ik_map effectors, in order, and the G1 body whose FK world pose
# supplies each one (hands map to wrist_yaw on G1 vs wrist_roll on X2).
EFFECTOR_G1_BODIES = [
    "pelvis", "torso_link",
    "left_shoulder_roll_link", "left_elbow_link", "left_wrist_yaw_link",
    "right_shoulder_roll_link", "right_elbow_link", "right_wrist_yaw_link",
    "left_hip_roll_link", "left_knee_link", "left_ankle_roll_link",
    "right_hip_roll_link", "right_knee_link", "right_ankle_roll_link",
]


def _g1_mjcf_path() -> str:
    return str(newton.utils.download_asset("unitree_g1") / "mjcf" / "g1_29dof_rev_1_0.xml")


def _colmap(csv_path: str) -> dict:
    header = open(csv_path).readline().strip().split(",")
    return {n.replace("_dof", ""): i for i, n in enumerate(header)}


class G1ToX2Retargeter:
    """Retarget Unitree G1 CSVs to Agibot X2 Ultra CSVs. Build once, reuse per clip.

    Must run inside a warp device scope, e.g.:
        wp.init()
        with wp.ScopedDevice("cuda:0"):
            r = G1ToX2Retargeter()
            r.retarget_dir(g1_dir, out_dir)
    """

    def __init__(self, retargeter_config=None, calibration=None, g1_mjcf=None):
        rc = retargeter_config or (_CFG_DIR / "g1_to_x2_ultra_retargeter_config.json")
        cal = calibration or (_CFG_DIR / "g1_to_x2_ultra_calibration.json")
        self.calib = io_utils.load_json(str(cal))
        self.scale = float(self.calib["position_scale"])
        self.se_fit = self.calib["shoulder_elbow_fit"]           # {joint: [a, b]}
        self.wrist_remap = self.calib["wrist_remap"]             # {x2_joint: [g1_joint, sign]}
        # airborne/acrobatic post-process: per-frame vertical clamp to G1 (off by default)
        self.floor_clamp = bool(self.calib.get("floor_clamp", False))
        # constant root lift (cm) so X2's soles rest on the ground. The feet
        # stabilizer matches ankle-roll link ORIGINS, but X2's sole sits ~7.3 cm
        # below its ankle origin vs G1's ~3.5 cm, so X2's bigger foot otherwise
        # sinks ~5 cm. Applied on the normal path only (the floor-clamp handles
        # ground placement itself). See docs/g1_to_x2.md.
        self.foot_ground_offset_cm = float(self.calib.get("foot_ground_offset_cm", 0.0))

        # G1 FK model + tracked body ids
        self.g1_model = mujoco.MjModel.from_xml_path(g1_mjcf or _g1_mjcf_path())
        self.g1_data = mujoco.MjData(self.g1_model)
        self.g1_body_ids = [mujoco.mj_name2id(self.g1_model, mujoco.mjtObj.mjOBJ_BODY, b)
                            for b in EFFECTOR_G1_BODIES]

        # X2 FK model (joint-limit clamping of arm joints + optional floor clamp)
        self.x2_model = mujoco.MjModel.from_xml_path(_X2_MJCF)
        self.x2_data = mujoco.MjData(self.x2_model)
        self.x2_lim = {}
        for j in list(self.se_fit) + list(self.wrist_remap):
            jid = mujoco.mj_name2id(self.x2_model, mujoco.mjtObj.mjOBJ_JOINT, j)
            self.x2_lim[j] = tuple(np.degrees(self.x2_model.jnt_range[jid]))

        # X2 IK pipeline (arm rotation weights zeroed via the retargeter config)
        skel, _ = bvh_utils.BVHImporter().create_skeleton(_SOMA_INIT_BVH)
        self.pipe = newton_pipeline.NewtonPipeline(
            skel, "soma", "agibot_x2_ultra", retarget_config=io_utils.load_json(str(rc)))
        self.csv_cfg = csv_utils.AgibotX2Ultra31DOF_CSVConfig()

    # -- stage 1+2: G1 FK -> scaled (F,14,7) keypoints -----------------------
    def _keypoints(self, g1_csv: str) -> np.ndarray:
        with open(g1_csv) as f:
            header = f.readline()
        if "root_rotate" not in header and "root_translate" not in header:
            raise ValueError(
                f"{g1_csv}: not a G1 retarget CSV. Expected a header row with "
                "root_translate[cm]/root_rotate[euler deg] + named *_dof joints[deg]. "
                "A raw G1 qpos CSV (headerless, metres + quaternion + radians) will NOT "
                "work here -- convert it first with scripts/g1_qpos_to_soma_csv.py.")
        mat = np.loadtxt(g1_csv, delimiter=",", skiprows=1, dtype=np.float64)
        if mat.ndim == 1:
            mat = mat[None, :]
        T = mat.shape[0]
        tg = np.zeros((T, len(self.g1_body_ids), 7), dtype=np.float64)
        for t in range(T):
            row = mat[t]
            self.g1_data.qpos[0:3] = row[1:4] * 0.01
            q = R.from_euler("xyz", np.deg2rad(row[4:7])).as_quat()
            self.g1_data.qpos[3:7] = [q[3], q[0], q[1], q[2]]
            self.g1_data.qpos[7:7 + G1_NUM_DOF] = np.deg2rad(row[7:7 + G1_NUM_DOF])
            mujoco.mj_forward(self.g1_model, self.g1_data)
            for e, bid in enumerate(self.g1_body_ids):
                tg[t, e, 0:3] = self.g1_data.xpos[bid]
                w, x, y, z = self.g1_data.xquat[bid]
                tg[t, e, 3:7] = [x, y, z, w]
        # scale keypoints to X2 proportions. Two centering modes:
        #  - "clip_start_floor" (default): about a FIXED floor point under the
        #    start pelvis -> shrinks pose + height + walk trajectory together.
        #    Correct for upright, feet-on-ground locomotion.
        #  - "pelvis": about EACH frame's own pelvis -> shrinks only the pose
        #    (limb lengths), preserving the root's vertical/rotational travel.
        #    Correct for airborne / inverted motion (jumps, cartwheels) where a
        #    fixed-floor center would vertically squash the flight phase.
        mode = self.calib.get("position_scale_center", "clip_start_floor")
        if mode == "pelvis":
            c = tg[:, 0:1, 0:3]                                   # (T,1,3) per-frame pelvis
            tg[:, :, 0:3] = c + self.scale * (tg[:, :, 0:3] - c)
        elif mode == "contact_floor":
            # Per-frame center at the pelvis XY but at the height of the LOWEST
            # keypoint (the ground-contact point: feet in stance, HANDS in a
            # handstand). Shrinks the body to X2 proportions while keeping the
            # planted point on the floor -> no squash AND no float. For
            # continuous-contact acrobatics (cartwheels, floor breaking).
            c = np.empty((T, 1, 3))
            c[:, 0, 0:2] = tg[:, 0, 0:2]                          # pelvis XY
            c[:, 0, 2] = tg[:, :, 2].min(axis=1)                  # lowest keypoint Z
            tg[:, :, 0:3] = c + self.scale * (tg[:, :, 0:3] - c)
        else:
            center = tg[0, 0, 0:3].copy()
            center[2] = 0.0
            tg[:, :, 0:3] = center + self.scale * (tg[:, :, 0:3] - center)
        return tg

    # -- stage 3: inject -> X2 IK -> CSV -------------------------------------
    def _inject(self, targets: np.ndarray, out_csv: str) -> None:
        targets = np.ascontiguousarray(targets, dtype=np.float32)
        n = self.pipe.num_initialization_frames + self.pipe.num_stabilization_frames
        padded = np.concatenate([np.repeat(targets[:1], n, axis=0), targets], axis=0).astype(np.float32)
        self.pipe.clear()
        self.pipe.input_targets = [padded]
        self.pipe.input_sample_rates = [120.0]
        self.pipe.max_frames = len(padded)
        csv_utils.save_csv(out_csv, self.pipe.execute()[0], csv_config=self.csv_cfg)

    # -- stage 4: overwrite arm joints from the G1 CSV -----------------------
    def _arm_jointmap(self, out_csv: str, g1_csv: str) -> None:
        x2 = np.loadtxt(out_csv, delimiter=",", skiprows=1, dtype=np.float64)
        g1 = np.loadtxt(g1_csv, delimiter=",", skiprows=1, dtype=np.float64)
        T = min(len(x2), len(g1))
        x2, g1 = x2[:T], g1[:T]
        xc, gc = _colmap(out_csv), _colmap(g1_csv)
        for j, (a, b) in self.se_fit.items():                    # shoulder + elbow
            lo, hi = self.x2_lim[j]
            x2[:, xc[j]] = np.clip(a * g1[:, gc[j]] + b, lo, hi)
        for xj, (gj, sign) in self.wrist_remap.items():          # wrist functional remap
            lo, hi = self.x2_lim[xj]
            x2[:, xc[xj]] = np.clip(sign * g1[:, gc[gj]], lo, hi)
        header = open(out_csv).readline().strip()
        with open(out_csv, "w") as f:
            f.write(header + "\n")
            np.savetxt(f, x2, delimiter=",", fmt="%.6f")

    # -- stage 5 (optional): floor clamp for airborne / acrobatic motion -----
    @staticmethod
    def _min_body_z(model, data, mat: np.ndarray, ndof: int) -> np.ndarray:
        """Per-frame lowest body-origin Z (m) from a soma-format pose matrix."""
        out = np.empty(len(mat))
        for t, row in enumerate(mat):
            data.qpos[0:3] = row[1:4] * 0.01
            q = R.from_euler("xyz", np.deg2rad(row[4:7])).as_quat()   # xyzw
            data.qpos[3:7] = [q[3], q[0], q[1], q[2]]
            data.qpos[7:7 + ndof] = np.deg2rad(row[7:7 + ndof])
            mujoco.mj_forward(model, data)
            out[t] = data.xpos[1:, 2].min()      # skip world body 0
        return out

    def _floor_clamp(self, out_csv: str, g1_csv: str, smooth_k: int = 5) -> None:
        """Shift the X2 root Z per frame so X2's lowest body tracks G1's lowest
        body. This plants whatever limb is actually on the floor (feet in stance,
        HANDS in a handstand, nothing mid-flight) without assuming feet-on-ground
        -- the fix for cartwheels / backflips where a fixed floor sinks or floats
        the robot. Lightly smoothed to avoid vertical jitter."""
        g1 = np.loadtxt(g1_csv, delimiter=",", skiprows=1, dtype=np.float64)
        x2 = np.loadtxt(out_csv, delimiter=",", skiprows=1, dtype=np.float64)
        T = min(len(g1), len(x2))
        gz = self._min_body_z(self.g1_model, self.g1_data, g1[:T], G1_NUM_DOF)
        xz = self._min_body_z(self.x2_model, self.x2_data, x2[:T], X2_NUM_DOF)
        offset_cm = (gz - xz) * 100.0
        if smooth_k > 1:
            ker = np.ones(smooth_k) / smooth_k
            offset_cm = np.convolve(np.pad(offset_cm, smooth_k // 2, mode="edge"), ker, mode="valid")[:T]
        x2 = x2[:T]
        x2[:, 3] += offset_cm                     # root_translateZ (cm)
        header = open(out_csv).readline().strip()
        with open(out_csv, "w") as f:
            f.write(header + "\n")
            np.savetxt(f, x2, delimiter=",", fmt="%.6f")

    def _lift_root_z(self, out_csv: str, cm: float) -> None:
        """Add a constant offset (cm) to the X2 root_translateZ column."""
        x2 = np.loadtxt(out_csv, delimiter=",", skiprows=1, dtype=np.float64)
        x2[:, 3] += cm
        header = open(out_csv).readline().strip()
        with open(out_csv, "w") as f:
            f.write(header + "\n")
            np.savetxt(f, x2, delimiter=",", fmt="%.6f")

    # -- public API ----------------------------------------------------------
    def retarget(self, g1_csv: str, out_csv: str) -> None:
        """Retarget a single G1 CSV to an X2 CSV written at out_csv."""
        self._inject(self._keypoints(g1_csv), out_csv)
        self._arm_jointmap(out_csv, g1_csv)
        if self.floor_clamp:
            self._floor_clamp(out_csv, g1_csv)
        elif self.foot_ground_offset_cm:
            self._lift_root_z(out_csv, self.foot_ground_offset_cm)

    def retarget_batch(self, items) -> int:
        """Retarget many clips in ONE batched IK call (num_envs = len(items)).

        items: list of (g1_csv, out_csv). Far faster than per-clip retarget() for
        bulk runs -- the X2 IK solves all clips in parallel on the GPU. Clips of
        different lengths are fine (execute returns each at its own length); for
        best GPU efficiency batch clips of similar length together.
        Returns the number of clips written.
        """
        n = self.pipe.num_initialization_frames + self.pipe.num_stabilization_frames
        padded = []
        for g1_csv, _ in items:
            kp = self._keypoints(g1_csv).astype(np.float32)
            p = np.concatenate([np.repeat(kp[:1], n, axis=0), kp], axis=0) if n else kp
            padded.append(np.ascontiguousarray(p, dtype=np.float32))
        self.pipe.clear()
        self.pipe.input_targets = padded
        self.pipe.input_sample_rates = [120.0] * len(items)
        self.pipe.max_frames = max(len(p) for p in padded)
        results = self.pipe.execute()
        for (g1_csv, out_csv), res in zip(items, results):
            csv_utils.save_csv(out_csv, res, csv_config=self.csv_cfg)
            self._arm_jointmap(out_csv, g1_csv)
            if self.floor_clamp:
                self._floor_clamp(out_csv, g1_csv)
            elif self.foot_ground_offset_cm:
                self._lift_root_z(out_csv, self.foot_ground_offset_cm)
        return len(items)

    def retarget_dir(self, g1_dir: str, out_dir: str) -> int:
        """Retarget every G1 CSV in g1_dir; returns the count converted."""
        os.makedirs(out_dir, exist_ok=True)
        n = 0
        for f in sorted(os.listdir(g1_dir)):
            if f.endswith(".csv"):
                self.retarget(os.path.join(g1_dir, f), os.path.join(out_dir, f))
                n += 1
        return n
