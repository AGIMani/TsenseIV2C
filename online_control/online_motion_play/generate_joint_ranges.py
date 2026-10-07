"""Regenerate the default UI/wire range table without opening a serial port."""
from pathlib import Path
import tempfile

import server
from a30_session import A30Session
from a30_kinematics.downlink import MIT_GEAR_RATIOS


def main():
    lines = ["# Online Motion Play：UI 与下发范围", "",
             "由当前 UI 限位与下发转换生成，默认标定。页面随型号、左右手及 V2 行程系数更新。", "",
             "UI/动作文件单位为 rad；直线目标为 -abs(行程) mm，零行程为 0。旋转单位为 °；"
             f"A3.0 外接两路为电机端角度，已经包含 M1 ×{MIT_GEAR_RATIOS[0]:g} / M2 ×{MIT_GEAR_RATIOS[1]:g}。", "",
             "下发范围按数值从小到大排列；最后一列依次对应 UI 最小值、最大值。"
             "直线机构转换为非线性，不能按范围做线性插值。数值保留四位小数；下发仍使用完整精度。", ""]
    fmt = lambda value: f"{value:.4f}".rstrip("0").rstrip(".") if abs(value) >= .00005 else "0"
    with tempfile.TemporaryDirectory() as directory:
        bridge = server.Bridge(directory)
        try:
            for model in ("TsenseI_v2", "TsenseI_v3_0", "TsenseI_v3_0_withid"):
                for side in ("left", "right"):
                    if model == "TsenseI_v2":
                        spec = bridge.snapshot()["controls"]["handSide"]
                        bridge.dispatch("widget", {"id": spec["id"], "value": int(side == "right")})
                        joints = bridge.snapshot()["joints"]
                    else:
                        session = A30Session(directory, side, model_id=model)
                        try:
                            joints = session.snapshot()["joints"]
                        finally:
                            session.close()
                    lines += [f"## {model} / {side}", "",
                              "| 关节 | UI 范围 (rad) | 槽位 | 下发范围 | UI 两端对应下发值 |",
                              "|---|---|---|---|---|"]
                    for joint in joints:
                        spec = joint["downlink"]
                        unit = "mm" if spec["unit"] == "mm" else "°"
                        lines.append(f"| {joint['short']} | [{fmt(joint['min'])}, {fmt(joint['max'])}] | {spec['slot']} | "
                                     f"[{fmt(spec['min'])}, {fmt(spec['max'])}] {unit} | "
                                     f"{fmt(spec['at_ui_min'])} → {fmt(spec['at_ui_max'])} {unit} |")
                    lines.append("")
        finally:
            bridge.close()
    path = Path(__file__).with_name("joint_ranges.md")
    path.write_text("\n".join(lines), encoding="utf-8")
    print(path)


if __name__ == "__main__":
    main()
