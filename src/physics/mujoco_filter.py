"""MuJoCo physics filter for hand retargeting.

Wraps a MuJoCo simulation step so that:
  - Input:  joint angles in URDF/Pinocchio order (output of Retargeter.retarget)
  - Output: actual joint angles after MuJoCo physics resolves dynamics,
            collisions and joint limits — returned in the same joint order.

The difference between input and output reflects physical effects:
  - Joint limit clamping (ctrlrange / forcerange)
  - Self-collision contact forces pushing joints to non-penetrating poses
  - PD actuator dynamics (finite kp/kv means the joint may lag or overshoot)
  - Inertia / damping effects

Quick usage
-----------
    from physics.mujoco_filter import MuJoCoPhysicsFilter

    filt = MuJoCoPhysicsFilter.from_retargeter(retargeter)
    qpos_physical = filt.step(qpos_from_retargeter)

Standalone demo
---------------
    python mujoco_physics_filter.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Optional

import mujoco
import numpy as np

from config_path import PROJECT_ROOT

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

_LOG_TAG = "MuJoCoPhysics"


def _plog(msg: str) -> None:
    try:
        from utils.log_control import emit
        emit(msg, _LOG_TAG)
    except Exception:
        print(f"[{_LOG_TAG}] {msg}", flush=True)


def _apply_hard_contact_settings(model: mujoco.MjModel) -> int:
    """收紧碰撞 geom 的接触参数，使接触更接近硬约束。返回修改的 geom 数量。"""
    n = 0
    for gid in range(model.ngeom):
        if model.geom_contype[gid] == 0 and model.geom_conaffinity[gid] == 0:
            continue
        # solref: 时间常数越小越硬；solimp: dmin 越高允许穿透越少
        # 微调（2026-07）：略软于原 0.002/0.99，仍明显硬于 MJCF 默认 0.01
        model.geom_solref[gid] = [0.003, 1.05]
        model.geom_solimp[gid] = [0.985, 0.997, 0.0007, 0.5, 2.0]
        model.geom_friction[gid] = [0.9, 0.008, 0.0001]
        n += 1
    return n


def _build_reorder_perm(src_names: list[str], dst_names: list[str]) -> Optional[np.ndarray]:
    """Index array ``perm`` so ``x[perm]`` reorders src-order -> dst-order.

    Returns None when the name sets don't match (caller treats it as identity).
    """
    idx = {n: i for i, n in enumerate(src_names)}
    try:
        perm = np.array([idx[n] for n in dst_names], dtype=int)
    except KeyError:
        return None
    return perm


class MuJoCoPhysicsFilter:
    """Apply one or more MuJoCo physics steps to a vector of target joint angles.

    Parameters
    ----------
    mjcf_path:
        Path to the MJCF XML file for the hand model.
        Mutually exclusive with ``xml_string``.
    xml_string:
        Raw MJCF XML string.  Used when the MJCF is generated in-memory.
        Mutually exclusive with ``mjcf_path``.
    urdf_joint_names:
        Joint names in the order the *input* qpos vector uses
        (typically ``retargeter.optimizer.robot.dof_joint_names``).
        Pass ``None`` to skip reordering (input assumed to already be in
        MJCF/actuator order).
    warmup_steps:
        Number of ``mj_step`` calls during initialisation to let the
        simulation settle from the rest pose before use.
    default_n_steps:
        Default number of physics steps per ``step()`` call.
        Larger values improve tracking accuracy at the cost of compute time.
    """

    def __init__(
        self,
        mjcf_path: Optional[str | Path] = None,
        *,
        xml_string: Optional[str] = None,
        urdf_joint_names: Optional[list[str]] = None,
        warmup_steps: int = 50,
        default_n_steps: int = 25,
        hard_contact: bool = True,
        contact_log: bool = False,
        contact_log_interval_s: float = 0.15,
        retarget_pull_steps: int = 3,
    ):
        if mjcf_path is not None and xml_string is not None:
            raise ValueError("Provide either mjcf_path or xml_string, not both.")
        if mjcf_path is not None:
            mjcf_path = Path(mjcf_path)
            if not mjcf_path.exists():
                raise FileNotFoundError(f"MJCF model not found: {mjcf_path}")
            self.model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        elif xml_string is not None:
            self.model = mujoco.MjModel.from_xml_string(xml_string)
        else:
            raise ValueError("Either mjcf_path or xml_string must be provided.")

        self.default_n_steps = default_n_steps
        self.hard_contact = hard_contact
        self.contact_log = contact_log
        self.contact_log_interval_s = contact_log_interval_s
        # hard_contact 下：无碰撞全程拉追；有碰撞后最多 settle 这么多步
        self.retarget_pull_steps = max(0, retarget_pull_steps)
        self._last_contact_log_t = 0.0
        self.data = mujoco.MjData(self.model)
        self._collision_geom_pairs: list[tuple[int, int]] = []
        for first in range(self.model.ngeom):
            for second in range(first + 1, self.model.ngeom):
                first_type = int(self.model.geom_contype[first])
                first_affinity = int(self.model.geom_conaffinity[first])
                second_type = int(self.model.geom_contype[second])
                second_affinity = int(self.model.geom_conaffinity[second])
                if (first_type & second_affinity) or (
                    second_type & first_affinity
                ):
                    self._collision_geom_pairs.append((first, second))

        if hard_contact:
            n_geom = _apply_hard_contact_settings(self.model)
            _plog(f"硬接触已启用: 收紧 {n_geom} 个碰撞 geom 的 solref/solimp")

        # ── joint name lists ──────────────────────────────────────────────
        # MJCF actuator order (what data.ctrl expects)
        self._act_joint_names: list[str] = [
            mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_JOINT,
                self.model.actuator_trnid[i, 0]
            )
            for i in range(self.model.nu)
        ]
        # MJCF qpos order (what data.qpos stores)
        self._mjcf_joint_names: list[str] = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(self.model.njnt)
        ]

        # ── permutation: URDF order -> actuator order (for ctrl write) ───
        self._perm_in: Optional[np.ndarray] = None
        # ── permutation: MJCF qpos order -> URDF order (for qpos read) ───
        self._perm_out: Optional[np.ndarray] = None

        if urdf_joint_names is not None:
            self._perm_in = _build_reorder_perm(urdf_joint_names, self._act_joint_names)
            # inverse: MJCF qpos order -> URDF order
            perm_mjcf_to_urdf = _build_reorder_perm(self._mjcf_joint_names, urdf_joint_names)
            self._perm_out = perm_mjcf_to_urdf

        # ── initialise ctrl to mid-range and warm up ──────────────────────
        for i in range(self.model.nu):
            if self.model.actuator_ctrllimited[i]:
                lo, hi = self.model.actuator_ctrlrange[i]
                self.data.ctrl[i] = (lo + hi) / 2.0
            else:
                self.data.ctrl[i] = 0.0

        for _ in range(warmup_steps):
            mujoco.mj_step(self.model, self.data)

        # ── store the warmed-up state as the "reset" reference ────────────
        self._init_qpos = self.data.qpos.copy()
        self._init_qvel = self.data.qvel.copy()
        self._init_ctrl = self.data.ctrl.copy()

    # ─────────────────────────────────────────────────────────────────────
    @classmethod
    def from_xml_string(
        cls,
        xml_string: str,
        urdf_joint_names: Optional[list[str]] = None,
        warmup_steps: int = 50,
        default_n_steps: int = 25,
    ) -> "MuJoCoPhysicsFilter":
        """Convenience constructor: build directly from an MJCF XML string.

        Useful when the MJCF is generated in-memory (no file required).

        Parameters
        ----------
        xml_string:
            Complete MJCF XML (with ``<actuator>`` section).
        urdf_joint_names:
            If given, the input/output vector is reordered accordingly.
        warmup_steps:
            Physics warm-up steps at rest pose.
        default_n_steps:
            Default substeps per ``step()`` call.
        """
        return cls(
            xml_string=xml_string,
            urdf_joint_names=urdf_joint_names,
            warmup_steps=warmup_steps,
            default_n_steps=default_n_steps,
        )

    # ─────────────────────────────────────────────────────────────────────
    @classmethod
    def from_retargeter(
        cls,
        retargeter,
        mjcf_path: Optional[str | Path] = None,
        warmup_steps: int = 50,
    ) -> "MuJoCoPhysicsFilter":
        """Convenience constructor: build from a live ``Retargeter`` instance.

        Automatically extracts joint names and (if ``mjcf_path`` is not given)
        looks for the bundled MJCF for the retargeter's hand side.

        Parameters
        ----------
        retargeter:
            A ``wuji_retargeting.Retargeter`` instance.
        mjcf_path:
            Override the MJCF path.  When ``None`` the bundled WH110 model
            matching ``retargeter.hand_side`` is used.
        """
        urdf_joint_names: list[str] = retargeter.optimizer.robot.dof_joint_names

        if mjcf_path is None:
            hand_side = retargeter.hand_side
            mjcf_path = (
                PROJECT_ROOT
                / "wuji_retargeting" / "wuji-description" / "hand" / "body" / "mjcf"
                / f"{hand_side}.xml"
            )

        return cls(
            mjcf_path=mjcf_path,
            urdf_joint_names=urdf_joint_names,
            warmup_steps=warmup_steps,
        )

    # ─────────────────────────────────────────────────────────────────────
    def step(self, qpos_input: np.ndarray, n_steps: Optional[int] = None) -> np.ndarray:
        """Run one or more physics steps and return the resulting joint angles.

        Parameters
        ----------
        qpos_input:
            Target joint angles, shape ``(n_joints,)``, in URDF/Pinocchio
            order (i.e. the direct output of ``Retargeter.retarget``).
        n_steps:
            Number of physics substeps.  Defaults to ``self.default_n_steps``
            (set at construction, default = 25).
            More steps → joints track the target more closely but cost more CPU.

        Returns
        -------
        qpos_output:
            Actual joint angles after MuJoCo resolves physics, in the **same
            joint order** as the input.  The difference between input and
            output represents collision/limit/dynamics effects.
        """
        if n_steps is None:
            n_steps = self.default_n_steps

        # ── write ctrl (URDF order -> actuator order) ─────────────────────
        ctrl = qpos_input if self._perm_in is None else qpos_input[self._perm_in]
        n = min(len(ctrl), self.model.nu)
        self.data.ctrl[:n] = ctrl[:n]

        # ── physics steps ─────────────────────────────────────────────────
        for _ in range(max(1, n_steps)):
            mujoco.mj_step(self.model, self.data)

        # ── read qpos (MJCF order) ────────────────────────────────────────
        qpos_mjcf = self.data.qpos[: self.model.njnt].copy()

        # ── reorder MJCF qpos -> URDF order ──────────────────────────────
        if self._perm_out is not None:
            return qpos_mjcf[self._perm_out]
        return qpos_mjcf

    def step_n(self, qpos_input: np.ndarray, n_steps: int = 10) -> np.ndarray:
        """Alias for ``step(qpos_input, n_steps=n_steps)``."""
        return self.step(qpos_input, n_steps=n_steps)

    # ─────────────────────────────────────────────────────────────────────
    def _sync_ctrl_to_qpos(self) -> None:
        """将 ctrl 同步为当前 qpos，停止位置执行器继续朝 retarget 目标施力。"""
        for i in range(self.model.nu):
            jid = int(self.model.actuator_trnid[i, 0])
            adr = int(self.model.jnt_qposadr[jid])
            self.data.ctrl[i] = self.data.qpos[adr]

    def _geom_name(self, gid: int) -> str:
        return mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, gid) or f"geom{gid}"

    def _collect_contacts(self) -> list[dict]:
        """读取当前 data.contact，返回可序列化的接触摘要。"""
        rows: list[dict] = []
        frc = np.zeros(6, dtype=np.float64)
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            dist = float(c.dist)          # <0 表示穿透（米）
            pen_mm = max(0.0, -dist * 1000.0)
            mujoco.mj_contactForce(self.model, self.data, i, frc)
            fn = float(np.linalg.norm(frc[:3]))
            rows.append({
                "g1": self._geom_name(g1),
                "g2": self._geom_name(g2),
                "dist_mm": dist * 1000.0,
                "pen_mm": pen_mm,
                "force_n": fn,
            })
        rows.sort(key=lambda r: r["pen_mm"], reverse=True)
        return rows

    def contact_summary(self) -> list[dict]:
        """Return contacts for the currently synchronized pose.

        Call :meth:`sync_pose_from_dict` first when using this as a static
        collision query.  ``pen_mm`` is zero for grazing/separated contacts
        and positive for interpenetration.
        """
        return [dict(row) for row in self._collect_contacts()]

    def max_penetration_mm(self) -> float:
        """Maximum current collision penetration in millimetres."""
        return max(
            (float(row["pen_mm"]) for row in self._collect_contacts()),
            default=0.0,
        )

    def collision_pair_distance_mm(
        self,
        geom1: int,
        geom2: int,
        *,
        max_distance_mm: float = 25.0,
    ) -> float:
        """Return the signed surface distance for one enabled geom pair.

        Positive values are clearance, zero is touching, and negative values
        are penetration.  ``sync_pose_from_dict`` must be called first.  The
        upper search bound is deliberately finite because the safety filter is
        only interested in nearby pairs.
        """
        max_distance_m = max(float(max_distance_mm), 1.0e-6) / 1000.0
        distance_m = mujoco.mj_geomDistance(
            self.model,
            self.data,
            int(geom1),
            int(geom2),
            max_distance_m,
            None,
        )
        return float(distance_m) * 1000.0

    def collision_pair_distances(
        self,
        *,
        max_distance_mm: float = 25.0,
    ) -> list[dict]:
        """Return nearby signed distances for all MJCF-enabled self pairs.

        This uses MuJoCo's exact primitive distance query rather than waiting
        for ``data.contact``.  It therefore supplies the positive clearance
        needed by a predictive collision barrier before fingers touch.
        """
        limit_mm = max(float(max_distance_mm), 1.0e-6)
        rows: list[dict] = []
        for geom1, geom2 in self._collision_geom_pairs:
            distance_mm = self.collision_pair_distance_mm(
                geom1,
                geom2,
                max_distance_mm=limit_mm,
            )
            # mj_geomDistance returns distmax when the pair is farther away.
            if distance_mm >= limit_mm - 1.0e-9:
                continue
            rows.append(
                {
                    "geom1": int(geom1),
                    "geom2": int(geom2),
                    "g1": self._geom_name(geom1),
                    "g2": self._geom_name(geom2),
                    "distance_mm": float(distance_mm),
                    "dist_mm": float(distance_mm),
                    "pen_mm": max(0.0, -float(distance_mm)),
                }
            )
        rows.sort(key=lambda row: float(row["distance_mm"]))
        return rows

    def _log_step_dict_result(
        self,
        names: list[str],
        vals_in: np.ndarray,
        result: dict,
        elapsed_ms: float,
        n_pull: int,
        n_settle: int,
    ) -> None:
        if not self.contact_log:
            return
        now = time.perf_counter()
        if now - self._last_contact_log_t < self.contact_log_interval_s:
            return

        diffs: list[tuple[str, float]] = []
        for i, name in enumerate(names):
            if i >= len(vals_in):
                break
            d = float(result.get(name, vals_in[i])) - float(vals_in[i])
            if abs(d) > 1e-4:
                short = name.replace("left_hand_", "").replace("_joint", "")
                diffs.append((short, d))
        diffs.sort(key=lambda x: abs(x[1]), reverse=True)

        contacts = self._collect_contacts()
        ncon = len(contacts)
        max_pen = max((c["pen_mm"] for c in contacts), default=0.0)
        max_fn = max((c["force_n"] for c in contacts), default=0.0)
        max_dq = max((abs(d) for _, d in diffs), default=0.0)

        # 无接触且无修正时跳过，减少刷屏
        if ncon == 0 and max_dq < 0.002:
            return

        self._last_contact_log_t = now

        changed = ", ".join(f"{n}:{d:+.4f}" for n, d in diffs[:6])
        if len(diffs) > 6:
            changed += f" +{len(diffs) - 6}"

        parts = [
            f"ncon={ncon}",
            f"max_pen={max_pen:.2f}mm",
            f"max_fn={max_fn:.2f}N",
            f"max|dq|={max_dq:.4f}rad",
            f"pull={n_pull}",
            f"settle={n_settle}",
            f"t={elapsed_ms:.1f}ms",
        ]
        if changed:
            parts.append(f"corrected=[{changed}]")

        if contacts:
            top = contacts[:4]
            cstr = " | ".join(
                f"{c['g1']}<->{c['g2']} dist={c['dist_mm']:+.2f}mm "
                f"pen={c['pen_mm']:.2f}mm fn={c['force_n']:.1f}N"
                for c in top
            )
            parts.append(f"contacts=[{cstr}]")

        _plog(" ".join(parts))

    # ─────────────────────────────────────────────────────────────────────
    def step_dict(
        self,
        cfg: dict,
        n_steps: Optional[int] = None,
    ) -> dict:
        """Convenience wrapper: accept and return joint angles as a ``dict``.

        This is the easiest way to splice the filter into a pipeline that
        already works with ``{joint_name: angle_rad}`` dicts (e.g. the
        output of topo retarget + smoother).

        On the **first call** the method records the incoming key order and
        builds an internal name→index mapping so subsequent calls are fast.

        Parameters
        ----------
        cfg:
            ``{joint_name: angle_rad}`` dict — direct output of ``smooth()``.
        n_steps:
            Number of physics substeps.  Defaults to ``self.default_n_steps``.

        Returns
        -------
        dict
            Same keys as input; values are physics-corrected joint angles.
            Any joint name not found in the MJCF is passed through unchanged.
        """
        names = list(cfg.keys())
        self._ensure_step_dict_maps(names)
        vals = np.array([cfg[n] for n in names], dtype=np.float64)

        # ── Write ctrl (retarget 目标) ────────────────────────────────────
        for i, act_i in enumerate(self._sd_to_act):
            if act_i >= 0:
                self.data.ctrl[act_i] = vals[i]

        n = n_steps if n_steps is not None else self.default_n_steps
        n = max(1, n)

        # 跟踪优先：全程朝 retarget 拉；一旦出现接触，才 ctrl←qpos 做短 settle，
        # 避免无碰撞时 settle 把关节拖在旧姿态上（日志里常见 max|dq|≈0.5~0.8）。
        t0 = time.perf_counter()
        n_pull = 0
        n_settle = 0

        if not self.hard_contact:
            for _ in range(n):
                mujoco.mj_step(self.model, self.data)
            n_pull = n
        else:
            settle_budget = min(self.retarget_pull_steps, n)  # 接触后最多 settle 这么多步
            for i in range(n):
                mujoco.mj_step(self.model, self.data)
                n_pull += 1
                if self.data.ncon > 0:
                    remaining = n - n_pull
                    n_settle = min(remaining, settle_budget)
                    if n_settle > 0:
                        self._sync_ctrl_to_qpos()
                        for _ in range(n_settle):
                            mujoco.mj_step(self.model, self.data)
                    break

        elapsed_ms = (time.perf_counter() - t0) * 1000.0

        # ── Read back qpos ────────────────────────────────────────────────
        qpos_all = self.data.qpos[: self.model.njnt]
        result: dict = {}
        for i, name in enumerate(names):
            j = self._sd_to_qpos[i]
            result[name] = float(qpos_all[j]) if j >= 0 else float(vals[i])

        self._log_step_dict_result(names, vals, result, elapsed_ms, n_pull, n_settle)
        return result

    # ─────────────────────────────────────────────────────────────────────
    def get_capsule_render_data(self) -> list:
        """Return world-space capsule geometry for visualisation.

        Returns a list of ``(from_pos, to_pos, radius, finger_name)`` tuples:

        * ``from_pos`` / ``to_pos`` — ``(3,)`` float32 numpy arrays, world
          coordinates in **metres**.
        * ``radius`` — float, capsule radius in metres.
        * ``finger_name`` — one of ``'thumb'``, ``'index'``, ``'mid'``,
          ``'ring'``, ``'pinky'``, or ``'unknown'``.

        Call this **after** ``step_dict()`` so the kinematics are up to date.
        Pass the result directly to
        ``URDFGLWidget.update_capsule_wireframes()``.
        """
        _FINGER_NAMES = ("thumb", "index", "mid", "ring", "pinky")
        caps = []
        for gid in range(self.model.ngeom):
            if int(self.model.geom_type[gid]) != int(mujoco.mjtGeom.mjGEOM_CAPSULE):
                continue
            radius = float(self.model.geom_size[gid, 0])
            half_h = float(self.model.geom_size[gid, 1])
            center = np.array(self.data.geom_xpos[gid], dtype=np.float32)
            mat    = np.array(self.data.geom_xmat[gid], dtype=np.float32).reshape(3, 3)
            axis   = mat[:, 2]          # capsule axis = geom local z
            from_pt = (center - half_h * axis).astype(np.float32)
            to_pt   = (center + half_h * axis).astype(np.float32)
            gname   = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
            finger = next((f for f in _FINGER_NAMES if f in gname), "unknown")
            caps.append((from_pt, to_pt, radius, finger))
        return caps

    # ─────────────────────────────────────────────────────────────────────
    def _ensure_step_dict_maps(self, names: list[str]) -> None:
        """Build name→actuator/qpos index maps for ``step_dict`` / ``sync_pose_from_dict``."""
        if names == getattr(self, "_step_dict_names", None):
            return
        self._step_dict_names = names

        act_idx = {n: i for i, n in enumerate(self._act_joint_names)}
        qpos_idx = {n: i for i, n in enumerate(self._mjcf_joint_names)}
        canonical_names: list[str] = []
        for name in names:
            cname = name
            if cname not in act_idx and cname not in qpos_idx:
                if name.startswith("right_hand_"):
                    alt = "left_hand_" + name[len("right_hand_"):]
                    if alt in act_idx or alt in qpos_idx:
                        cname = alt
                elif name.startswith("left_hand_"):
                    alt = "right_hand_" + name[len("left_hand_"):]
                    if alt in act_idx or alt in qpos_idx:
                        cname = alt
            canonical_names.append(cname)
        self._step_dict_canonical_names = canonical_names

        self._sd_to_act = np.array(
            [act_idx.get(n, -1) for n in canonical_names], dtype=int)
        self._sd_to_qpos = np.array(
            [qpos_idx.get(n, -1) for n in canonical_names], dtype=int)

    def sync_pose_from_dict(self, cfg: dict) -> None:
        """将 qpos/ctrl 对齐到给定关节角（不步进仿真），用于碰撞开启时的暖启动。"""
        names = list(cfg.keys())
        self._ensure_step_dict_maps(names)

        qpos_all = self.data.qpos[: self.model.njnt]
        for i, name in enumerate(names):
            val = float(cfg[name])
            act_i = int(self._sd_to_act[i])
            if act_i >= 0:
                self.data.ctrl[act_i] = val
            j = int(self._sd_to_qpos[i])
            if j >= 0:
                qpos_all[j] = val

        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def reset(self):
        """Reset the simulation to the post-warmup initial state."""
        self.data.qpos[:] = self._init_qpos
        self.data.qvel[:] = self._init_qvel
        self.data.ctrl[:] = self._init_ctrl
        mujoco.mj_forward(self.model, self.data)

    def snapshot_state(self) -> tuple:
        """保存当前仿真状态（供碰撞预检后恢复）。"""
        return (
            self.data.qpos.copy(),
            self.data.qvel.copy(),
            self.data.ctrl.copy(),
        )

    def restore_state(self, snap: tuple) -> None:
        """恢复 ``snapshot_state()`` 保存的状态。"""
        qpos, qvel, ctrl = snap
        self.data.qpos[:] = qpos
        self.data.qvel[:] = qvel
        self.data.ctrl[:] = ctrl
        mujoco.mj_forward(self.model, self.data)

    # ─────────────────────────────────────────────────────────────────────
    @property
    def joint_names(self) -> list[str]:
        """Joint names in URDF/input order (or MJCF order if no URDF given)."""
        if self._perm_out is not None:
            # invert _perm_out to recover URDF order from MJCF names
            inv = np.argsort(self._perm_out)
            return [self._mjcf_joint_names[i] for i in inv]
        return self._mjcf_joint_names[: self.model.nu]

    @property
    def n_joints(self) -> int:
        return self.model.nu

    @property
    def timestep(self) -> float:
        return float(self.model.opt.timestep)
