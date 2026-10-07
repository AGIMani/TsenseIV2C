"""Natural grasp demo: fingers curl in place, no thumb opposition, smooth kinematic."""
from __future__ import annotations

import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

HERE = Path(__file__).resolve().parent
XML = HERE / "A3_hand.xml"

# Resolve by functional name: the four-finger chain is MCP, swing, PIP, DIP.
# The shell's thumb rotation zero poses differ from the bare A3.0 CAD.
FINGERS = ("index", "mid", "ring", "pinky")


def grasp_qpos(t: float, model: mujoco.MjModel) -> np.ndarray:
    q = np.zeros(model.nq)
    # slow open-close, ~5s cycle
    phase = 0.5 * (1.0 - np.cos(t * (2 * np.pi / 5.0)))

    # Natural fist: mainly PIP/DIP; tiny MCP; no swing; no thumb-to-fingers
    mcp = 0.25 * phase
    pip = 1.35 * phase
    dip = 1.1 * phase

    def set_joint(name, value):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(f"Missing joint {name}; rebuild A3_hand.xml")
        q[int(model.jnt_qposadr[joint_id])] = value

    for finger in FINGERS:
        for part, value in (("MCP", mcp), ("PIP", pip), ("DIP", dip)):
            set_joint(f"left_hand_{finger}_{part}_joint", value)
    set_joint("left_hand_thumb_MCP_joint", 1.2 * phase)
    set_joint("left_hand_thumb_PIP_joint", .95 * phase)
    # Both thumb base rotations and all four swing joints stay at zero.

    for i in range(model.njnt):
        adr = int(model.jnt_qposadr[i])
        if model.jnt_limited[i]:
            lo, hi = model.jnt_range[i]
            q[adr] = float(np.clip(q[adr], lo, hi))
    return q


def main() -> None:
    if not XML.exists():
        raise SystemExit(f"Missing {XML}. Run build_mjcf.py first.")

    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)

    print("自然抓握：各指独立弯曲，无对掌、无侧摆、无碰撞")
    print("  关窗口退出")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        t0 = time.time()
        while viewer.is_running():
            step_start = time.time()
            data.qpos[:] = grasp_qpos(time.time() - t0, model)
            data.qvel[:] = 0
            mujoco.mj_forward(model, data)
            viewer.sync()
            dt = 1.0 / 60.0 - (time.time() - step_start)
            if dt > 0:
                time.sleep(dt)

    print("done.")


if __name__ == "__main__":
    main()
