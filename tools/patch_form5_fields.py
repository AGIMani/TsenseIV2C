"""Insert extra telemetry fields into each motor groupBox in form5.ui."""

from __future__ import annotations

import re
from pathlib import Path

UI = Path(__file__).resolve().parents[1] / "form5.ui"

MOTOR_KEYS = [
    "lm_L12", "lm_L13", "rm_R04", "rm_R05",
    "lm_L06", "lm_L07", "lm_L08", "rm_R00",
    "lm_L09", "lm_L10", "lm_L11", "rm_R01",
    "lm_L00", "lm_L01", "lm_L02", "rm_R02",
    "lm_L03", "lm_L04", "lm_L05", "rm_R03", "rm_R06",
]

FIELDS = (
    ("target", "目标(°)"),
    ("err", "误差"),
    ("temp", "MCU°C"),
    ("iq", "iq_meas"),
    ("home", "回零"),
)
ROW_Y0, ROW_STEP = 42, 22
EXTRA_H = len(FIELDS) * ROW_STEP + 8


def fields_xml(key: str) -> str:
    parts = []
    for i, (fk, lbl) in enumerate(FIELDS):
        y = ROW_Y0 + i * ROW_STEP
        parts.append(
            f"""    <widget class="QLabel" name="label_{key}_{fk}">
     <property name="geometry">
      <rect><x>6</x><y>{y}</y><width>52</width><height>18</height></rect>
     </property>
     <property name="text"><string>{lbl}</string></property>
    </widget>
    <widget class="QLineEdit" name="lineEdit_{key}_{fk}">
     <property name="geometry">
      <rect><x>58</x><y>{y - 2}</y><width>66</width><height>20</height></rect>
     </property>
     <property name="readOnly"><bool>true</bool></property>
     <property name="alignment">
      <set>Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter</set>
     </property>
     <property name="placeholderText"><string>—</string></property>
    </widget>"""
        )
    return "\n".join(parts)


def patch_key(text: str, key: str) -> str:
    if f"lineEdit_{key}_target" in text:
        return text
    angle_tag = f'name="lineEdit_{key}_angle"'
    pos = text.find(angle_tag)
    if pos < 0:
        print(f"skip missing {key}")
        return text
    close = text.find("    </widget>", pos)
    if close < 0:
        return text
    close_end = close + len("    </widget>")
    text = text[:close_end] + "\n" + fields_xml(key) + text[close_end:]

    gpat = rf'(<widget class="QGroupBox" name="groupBox_{key}">.*?<height>)(\d+)(</height>)'
    def repl(m: re.Match) -> str:
        h = int(m.group(2))
        return f"{m.group(1)}{max(h, 58) + EXTRA_H}{m.group(3)}"
    text, n = re.subn(gpat, repl, text, count=1, flags=re.DOTALL)
    if n == 0:
        print(f"height not updated for {key}")
    return text


RX_BOX = """
   <widget class="QGroupBox" name="groupBox_rx_rate">
    <property name="geometry">
     <rect><x>8</x><y>500</y><width>200</width><height>58</height></rect>
    </property>
    <property name="title"><string>遥测 RX</string></property>
    <widget class="QLabel" name="label_rx_rate_caption">
     <property name="geometry">
      <rect><x>8</x><y>22</y><width>72</width><height>18</height></rect>
     </property>
     <property name="text"><string>速率(Hz)</string></property>
    </widget>
    <widget class="QLineEdit" name="lineEdit_rx_rate">
     <property name="geometry">
      <rect><x>84</x><y>20</y><width>100</width><height>22</height></rect>
     </property>
     <property name="readOnly"><bool>true</bool></property>
     <property name="alignment">
      <set>Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter</set>
     </property>
     <property name="placeholderText"><string>0.00</string></property>
    </widget>
   </widget>
"""


def main() -> None:
    text = UI.read_text(encoding="utf-8")
    for key in MOTOR_KEYS:
        text = patch_key(text, key)
    if "groupBox_rx_rate" not in text:
        text = text.replace("   <zorder>label_image</zorder>", RX_BOX + "   <zorder>label_image</zorder>")
    UI.write_text(text, encoding="utf-8")
    print("done, target count", text.count("_target"))


if __name__ == "__main__":
    main()
