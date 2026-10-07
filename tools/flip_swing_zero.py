"""四指 swing 零点反转:

  旧约定: q=0 = 旧几何零位, axis=-1 0 0, q 增大朝旧方向
  新约定: q=0 = 旧 q=0.4537856 姿态(init), axis=+1 0 0, 方向反转
          q_new = 0.4537856... - q_old, 限位仍 [0, 0.4537856]

实现: rpy_x' = rpy_x - UPPER, axis 取反。对 URDF 与胶囊 MJCF 同步。
"""
import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LEFT = ROOT / "assets/hands/TsenseI_hand/urdf/TsenseI__hand_left_v1.urdf"
MJCF = ROOT / "assets/hands/TsenseI_hand/mjcf/mjmodel.xml"

UPPER = 0.4537856055185257
FINGERS = ("index", "mid", "ring", "pinky")


def patch_left_urdf() -> None:
    text = LEFT.read_text(encoding="utf-8")
    for f in FINGERS:
        joint = f"left_hand_{f}_swing_joint"
        # joint block: from name=... to </joint>
        m = re.search(
            r'(name="%s"[\s\S]*?</joint>)' % re.escape(joint), text
        )
        assert m, joint
        block = m.group(1)
        rm = re.search(r'rpy="([-\d.eE]+) ([-\d.eE]+) ([-\d.eE]+)"', block)
        assert rm, joint
        rx, ry, rz = (float(rm.group(i)) for i in (1, 2, 3))
        new_rx = rx - UPPER
        new_rpy = f'rpy="{new_rx!r} {ry!r} {rz!r}"'
        new_block = block.replace(rm.group(0), new_rpy)
        old_axis = 'xyz="-1 0 0"'
        assert old_axis in new_block, joint
        new_block = new_block.replace(old_axis, 'xyz="1 0 0"')
        text = text.replace(block, new_block)
        print(f"URDF  {joint}: rpy_x {rx:+.6f} -> {new_rx:+.6f}, axis -1->+1")
    LEFT.write_text(text, encoding="utf-8")


def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quat_from_rpy(r, p, y):
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def patch_mjcf() -> None:
    text = MJCF.read_text(encoding="utf-8")
    qx_neg = (math.cos(UPPER * 0.5), -math.sin(UPPER * 0.5), 0.0, 0.0)
    for f in FINGERS:
        body = f"left_hand_{f}_swing_link"
        joint = f"left_hand_{f}_swing_joint"
        bm = re.search(
            r'<body name="%s" pos="([^"]+)" quat="([^"]+)">' % re.escape(body),
            text,
        )
        assert bm, body
        q_old = tuple(float(x) for x in bm.group(2).split())
        q_new = _qmul(q_old, qx_neg)
        n = math.sqrt(sum(c * c for c in q_new))
        q_new = tuple(c / n for c in q_new)
        new_tag = '<body name="%s" pos="%s" quat="%s">' % (
            body,
            bm.group(1),
            " ".join(f"{c:.7f}" for c in q_new),
        )
        text = text.replace(bm.group(0), new_tag)
        jm_old = f'<joint name="{joint}" type="hinge" axis="-1 0 0"'
        assert jm_old in text, joint
        text = text.replace(
            jm_old, f'<joint name="{joint}" type="hinge" axis="1 0 0"'
        )
        print(f"MJCF  {body}: quat {bm.group(2)} -> "
              + " ".join(f"{c:.7f}" for c in q_new))
    MJCF.write_text(text, encoding="utf-8")


def verify() -> None:
    """URDF 新 rpy 直接算 quat,须与 MJCF 里 q_old*qx(-UPPER) 一致。"""
    text = LEFT.read_text(encoding="utf-8")
    mtext = MJCF.read_text(encoding="utf-8")
    for f in FINGERS:
        joint = f"left_hand_{f}_swing_joint"
        m = re.search(
            r'name="%s"[\s\S]*?rpy="([-\d.eE]+) ([-\d.eE]+) ([-\d.eE]+)"'
            % re.escape(joint),
            text,
        )
        rx, ry, rz = (float(m.group(i)) for i in (1, 2, 3))
        q_ref = _quat_from_rpy(rx, ry, rz)
        bm = re.search(
            r'<body name="left_hand_%s_swing_link" pos="[^"]+" quat="([^"]+)">'
            % f,
            mtext,
        )
        q_mj = tuple(float(x) for x in bm.group(1).split())
        err = max(abs(a - b) for a, b in zip(q_ref, q_mj))
        status = "OK" if err < 1e-5 else f"MISMATCH err={err:.2e}"
        print(f"verify {f}: {status}")


if __name__ == "__main__":
    patch_left_urdf()
    patch_mjcf()
    verify()
