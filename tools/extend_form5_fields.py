"""Extend form5.ui: add telemetry rows under each motor groupBox + RX rate box."""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
UI = ROOT / "form5.ui"

EXTRA_FIELDS = (
    ("target", "目标(°)"),
    ("err", "误差"),
    ("temp", "MCU°C"),
    ("iq", "iq_meas"),
    ("home", "回零"),
)
ROW_Y0 = 42
ROW_STEP = 22
BOX_EXTRA_H = len(EXTRA_FIELDS) * ROW_STEP + 8


def _fields_xml(key: str) -> str:
    lines = []
    for i, (fk, lbl) in enumerate(EXTRA_FIELDS):
        y = ROW_Y0 + i * ROW_STEP
        lines.append(f"""    <widget class="QLabel" name="label_{key}_{fk}">
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
    </widget>""")
    return "\n".join(lines)


def extend_groupbox(match: re.Match) -> str:
    block = match.group(0)
    gname = match.group(1)
    if gname == "groupBox_rx_rate":
        return block
    key = gname.replace("groupBox_", "")
    if f"lineEdit_{key}_target" in block:
        return block
    h = int(match.group(2))
    new_h = max(h, 58) + BOX_EXTRA_H
    block = re.sub(r"<height>\d+</height>", f"<height>{new_h}</height>", block, count=1)
    marker = f'name="lineEdit_{key}_angle">'
    pos = block.find(marker)
    if pos < 0:
        return block
    close = block.find("    </widget>", pos)
    if close < 0:
        return block
    close_end = close + len("    </widget>")
    return block[:close_end] + "\n" + _fields_xml(key) + block[close_end:]


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
    pattern = re.compile(
        r'<widget class="QGroupBox" name="(groupBox_[^"]+)">.*?<height>(\d+)</height>.*?</widget>',
        re.DOTALL,
    )
    text = pattern.sub(extend_groupbox, text)
    if "groupBox_rx_rate" not in text:
        text = text.replace("   <zorder>label_image</zorder>", RX_BOX + "   <zorder>label_image</zorder>")
    UI.write_text(text, encoding="utf-8")
    print(f"Extended {UI}")


if __name__ == "__main__":
    main()
