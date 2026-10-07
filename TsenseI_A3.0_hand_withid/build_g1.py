"""Build E:\\URDF\\AIgei\\G1 from yw1 with MINIMAL middle-finger fix only."""
from __future__ import annotations

import csv
import math
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

SRC = Path(r"E:\URDF\A3-2691-1053yw1")
DST = Path(r"E:\URDF\AIgei\G1")
CSV = SRC / "urdf" / "A3-2691-1053yw1.csv"
URDF = SRC / "urdf" / "A3-2691-1053yw1.urdf"

LIMITS = {
    "21": (-0.4, 0.4),
    "22": (0.0, 1.4),
    "23": (0.0, 1.6),
    "24": (0.0, 1.2),
    "31": (-0.3, 0.3),
    "32": (0.0, 1.4),
    "33": (0.0, 1.6),
    "34": (0.0, 1.2),
    "41": (-0.4, 0.4),
    "42": (0.0, 1.4),
    "43": (0.0, 1.6),
    "44": (0.0, 1.2),
    "51": (-0.5, 0.3),
    "52": (0.0, 1.4),
    "53": (0.0, 1.6),
    "54": (0.0, 1.2),
    "11": (-0.5, 1.2),
    "12": (-0.5, 1.5),
    "13": (0.0, 1.6),
    "14": (0.0, 1.2),
}


def rpy_to_rot(rpy: np.ndarray) -> np.ndarray:
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1.0, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1.0, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
    return Rz @ Ry @ Rx


def fk(root: ET.Element):
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        a = j.find("axis")
        joints[j.attrib["name"]] = {
            "xyz": np.fromstring(o.attrib["xyz"], sep=" "),
            "rpy": np.fromstring(o.attrib["rpy"], sep=" "),
            "axis": np.fromstring(a.attrib["xyz"], sep=" "),
            "parent": j.find("parent").attrib["link"],
            "child": j.find("child").attrib["link"],
        }
    links = {"base_link": (np.eye(3), np.zeros(3))}
    pending = list(joints)
    while pending:
        still = []
        for name in pending:
            j = joints[name]
            if j["parent"] not in links:
                still.append(name)
                continue
            Rp, tp = links[j["parent"]]
            R = Rp @ rpy_to_rot(j["rpy"])
            links[j["child"]] = (R, tp + Rp @ j["xyz"])
        pending = still
    return joints, links


def report(tag: str, joints, links) -> None:
    print(f"\n==== {tag} ====")
    for digit in "2345":
        y3 = links[f"Empty_Link{digit}3"][0][:, 1]
        z3 = links[f"Empty_Link{digit}3"][1][2]
        z4 = links[f"Empty_Link{digit}4"][1][2]
        y2 = links[f"Empty_Link{digit}2"][0][:, 1]
        print(
            f" finger{digit}: L{digit}2_Yz={y2[2]:+.3f} L{digit}3_Yz={y3[2]:+.3f} "
            f"tip_beyond_pip={z4 > z3} "
            f"pos3={np.array2string(links[f'Empty_Link{digit}3'][1], precision=3)} "
            f"pos4={np.array2string(links[f'Empty_Link{digit}4'][1], precision=3)}"
        )


def main() -> None:
    # copy package
    for item in ["meshes", "config", "launch", "textures", "CMakeLists.txt", "package.xml"]:
        s, d = SRC / item, DST / item
        if s.is_dir():
            if d.exists():
                shutil.rmtree(d)
            shutil.copytree(s, d)
        elif s.is_file():
            shutil.copy2(s, d)
    (DST / "urdf").mkdir(exist_ok=True)
    (DST / "mujoco").mkdir(exist_ok=True)
    shutil.copy2(CSV, DST / "urdf" / CSV.name)

    csv_joints = {}
    with CSV.open(encoding="utf-8") as f:
        for row in csv.DictReader(f):
            name = (row.get("Joint Name") or "").strip()
            if not name:
                continue
            csv_joints[name] = {
                "xyz": [
                    float(row["Joint Origin X"]),
                    float(row["Joint Origin Y"]),
                    float(row["Joint Origin Z"]),
                ],
                "rpy": [
                    float(row["Joint Origin Roll"]),
                    float(row["Joint Origin Pitch"]),
                    float(row["Joint Origin Yaw"]),
                ],
                "axis": [
                    float(row["Joint Axis X"]),
                    float(row["Joint Axis Y"]),
                    float(row["Joint Axis Z"]),
                ],
            }

    tree = ET.parse(URDF)
    root = tree.getroot()

    # Restore every joint from CSV exactly
    for j in root.findall("joint"):
        name = j.attrib["name"]
        cj = csv_joints[name]
        j.find("origin").set("xyz", f"{cj['xyz'][0]} {cj['xyz'][1]} {cj['xyz'][2]}")
        j.find("origin").set("rpy", f"{cj['rpy'][0]} {cj['rpy'][1]} {cj['rpy'][2]}")
        j.find("axis").set("xyz", f"{cj['axis'][0]} {cj['axis'][1]} {cj['axis'][2]}")
        lo, hi = LIMITS[name]
        lim = j.find("limit")
        lim.set("lower", str(lo))
        lim.set("upper", str(hi))
        lim.set("effort", "5")
        lim.set("velocity", "3")

    joints, links = fk(root)
    report("ORIGINAL", joints, links)

    fixes: list[str] = []

    # Root cause for yw1: joint 32 rpy has extra pi roll vs ring finger.
    # CSV: (pi, -pi/2, 0)  -> should be (0, -pi/2, 0)
    # This flips Empty_Link32 Y into -Z and folds distal middle finger into palm.
    j32 = root.find("./joint[@name='32']")
    old_rpy = j32.find("origin").attrib["rpy"]
    j32.find("origin").set("rpy", "0 -1.5707963267949 0")
    fixes.append(f"joint 32 rpy: ({old_rpy}) -> (0 -1.5707963267949 0)  # remove extra pi")

    # After frame flip, MCP axis world may reverse vs other fingers; flip axis ONLY on 32
    joints, links = fk(root)
    aw32 = links[joints["32"]["child"]][0] @ joints["32"]["axis"]
    aw42 = links[joints["42"]["child"]][0] @ joints["42"]["axis"]
    if float(np.dot(aw32, aw42)) < 0:
        a = joints["32"]["axis"]
        j32.find("axis").set("xyz", f"{-float(a[0])} {-float(a[1])} {-float(a[2])}")
        fixes.append("joint 32 axis sign flipped to match ring MCP (zero pose unchanged)")

    # Middle DIP axis vs ring after parent fix
    joints, links = fk(root)
    aw34 = links[joints["34"]["child"]][0] @ joints["34"]["axis"]
    aw44 = links[joints["44"]["child"]][0] @ joints["44"]["axis"]
    if float(np.dot(aw34, aw44)) < 0:
        a = joints["34"]["axis"]
        root.find("./joint[@name='34']").find("axis").set(
            "xyz", f"{-float(a[0])} {-float(a[1])} {-float(a[2])}"
        )
        fixes.append("joint 34 axis sign flipped to match ring DIP")

    # Middle PIP axis vs ring
    joints, links = fk(root)
    aw33 = links[joints["33"]["child"]][0] @ joints["33"]["axis"]
    aw43 = links[joints["43"]["child"]][0] @ joints["43"]["axis"]
    if float(np.dot(aw33, aw43)) < 0:
        a = joints["33"]["axis"]
        root.find("./joint[@name='33']").find("axis").set(
            "xyz", f"{-float(a[0])} {-float(a[1])} {-float(a[2])}"
        )
        fixes.append("joint 33 axis sign flipped to match ring PIP")

    joints, links = fk(root)
    report("AFTER (middle only)", joints, links)

    # Sanity: do NOT change other fingers — compare finger2/4/5 to ORIGINAL snapshot
    print("\nFIXES (middle finger only):")
    for f in fixes:
        print(" -", f)

    assert links["Empty_Link33"][0][:, 1][2] > 0.5, "middle PIP Y still wrong"
    assert links["Empty_Link34"][1][2] > links["Empty_Link33"][1][2], "middle tip not beyond PIP"

    out = DST / "urdf" / "A3-2691-1053yw1.urdf"
    tree.write(out, encoding="utf-8", xml_declaration=True)
    text = out.read_text(encoding="utf-8")
    pkg = "A3-2691-1053yw1"
    if f"package://{pkg}/" not in text:
        # keep whatever package path was; normalize to yw1
        import re

        text = re.sub(r"package://[^/]+/meshes/", f"package://{pkg}/meshes/", text)
    out.write_text(text, encoding="utf-8")
    (DST / "mujoco" / "A3_hand.urdf").write_text(
        text.replace(f"package://{pkg}/meshes/", "../meshes/"),
        encoding="utf-8",
    )

    (DST / "FIX_NOTES.txt").write_text(
        "\n".join(
            [
                "来源: E:\\URDF\\A3-2691-1053yw1",
                "输出: E:\\URDF\\AIgei\\G1",
                "",
                "问题: 中指关节32（坐标系32）rpy 多了 pi，Empty_Link32 的 Y 朝向 -Z，",
                "      远端折进掌心。其他手指未改 rpy/xyz。",
                "",
                "修改:",
                *[f"  - {x}" for x in fixes],
                "  - 全部关节补充可用 limit（原导出多为 0..0）",
                "",
                "仿真: python mujoco\\simulate.py --seconds 0",
            ]
        ),
        encoding="utf-8",
    )
    print("\nwrote", DST)


if __name__ == "__main__":
    main()
