"""Build MuJoCo MJCF for manual posing: no contact interference, position actuators."""
from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
URDF = HERE / "A3_hand.urdf"
OUT = HERE / "A3_hand.xml"


def preserve_joint_precision(xml: str) -> str:
    """MjSpec.to_xml rounds quaternions to six significant digits.

Restore the URDF joint poses at full precision so the exported MJCF has the
same rotation axes as the reference. All moving parents in this package are
either the root (possibly through an identity fixed joint) or a moving body.
"""
    root = ET.fromstring(xml)
    urdf = ET.parse(URDF).getroot()
    for joint in urdf.findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        body = root.find(f".//body[@name='{joint.find('child').get('link')}']")
        if body is None:
            raise ValueError(f"Missing body for {joint.get('name')}")
        source_origin = joint.find('origin')
        body.set('pos', source_origin.get('xyz'))
        q = Rotation.from_euler('xyz', np.fromstring(source_origin.get('rpy'), sep=' ')).as_quat()
        body.set('quat', ' '.join(f'{v:.17g}' for v in q[[3, 0, 1, 2]]))
        target_joint = body.find(f"joint[@name='{joint.get('name')}']")
        target_joint.set('axis', joint.find('axis').get('xyz'))
        source_limit = joint.find('limit')
        target_joint.set('range', source_limit.get('lower') + ' ' + source_limit.get('upper'))
    ET.indent(root, space='  ')
    return ET.tostring(root, encoding='unicode')


def main() -> None:
    spec = mujoco.MjSpec.from_file(str(URDF))

    # Light damping so joints hold still when not driven
    for j in spec.joints:
        j.damping = [0.2, 0.0, 0.0]
        j.armature = 0.001

    for j in spec.joints:
        lo, hi = (float(j.range[0]), float(j.range[1])) if j.limited else (-1.5, 1.5)
        # Strong position servo so slider poses hold like a hand pose
        spec.add_actuator(
            name=f"act_{j.name}",
            target=j.name,
            trntype=mujoco.mjtTrn.mjTRN_JOINT,
            biastype=mujoco.mjtBias.mjBIAS_AFFINE,
            gainprm=[80, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            biasprm=[0, -80, -8, 0, 0, 0, 0, 0, 0, 0],
            ctrllimited=True,
            ctrlrange=[lo, hi],
            forcelimited=True,
            forcerange=[-20, 20],
        )

    xml = spec.to_xml()

    if 'meshdir="' in xml:
        xml = xml.replace('meshdir="', 'meshdir="../meshes" ', 1)
        xml = xml.replace('meshdir="../meshes" ../meshes"', 'meshdir="../meshes"')
    else:
        xml = xml.replace("<compiler", '<compiler meshdir="../meshes"', 1)

    # No gravity, no contacts — free manual posing without mesh interference
    opt_block = (
        '  <option timestep="0.002" gravity="0 0 0">\n'
        '    <flag contact="disable"/>\n'
        "  </option>\n"
        "  <visual>\n"
        '    <global offwidth="1024" offheight="1024"/>\n'
        "  </visual>\n"
    )
    if "<option" not in xml:
        import re

        if re.search(r"<compiler[^>]*/>", xml):
            xml = re.sub(r"(<compiler[^>]*/>)", r"\1\n" + opt_block, xml, count=1)
        else:
            xml = xml.replace("</compiler>", "</compiler>\n" + opt_block, 1)

    # Disable collision on all geoms (belt-and-suspenders with contact flag)
    xml = xml.replace("<geom ", '<geom contype="0" conaffinity="0" ')

    if "<light" not in xml:
        xml = xml.replace(
            "<worldbody>",
            (
                "<worldbody>\n"
                '    <light pos="0.15 -0.1 0.4" dir="-0.2 0.3 -1" diffuse="1 1 1"/>\n'
                '    <geom name="floor" type="plane" size="0.25 0.25 0.01" '
                'contype="0" conaffinity="0" rgba="0.2 0.25 0.3 1" pos="0 0 -0.05"/>'
            ),
            1,
        )

    # Clean mesh file= paths if double-prefixed later
    OUT.write_text(xml, encoding="utf-8")
    text = OUT.read_text(encoding="utf-8").replace('file="../meshes/', 'file="')
    OUT.write_text(preserve_joint_precision(text), encoding="utf-8")

    model = mujoco.MjModel.from_xml_path(str(OUT))
    print(f"Wrote {OUT}")
    print(f"nq={model.nq} nv={model.nv} nu={model.nu}  contact disabled, gravity off")


if __name__ == "__main__":
    main()
