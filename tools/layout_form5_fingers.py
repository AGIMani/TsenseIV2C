"""Generate form5.ui with per-finger columns; motors stacked vertically without overlap."""

from __future__ import annotations

from pathlib import Path

UI = Path(__file__).resolve().parents[1] / "form5.ui"

WIN_W, WIN_H = 1067, 579
IMG_W, IMG_H = 371, 351

MOTOR_W = 140
MOTOR_H = 158
MOTOR_GAP = 6
FINGER_TOP = 8
FINGER_INNER_X = 4
FINGER_TITLE_H = 22

FIELDS = (
    ("angle", "角度(°)"),
    ("target", "目标(°)"),
    ("err", "误差"),
    ("temp", "MCU°C"),
    ("iq", "iq_meas"),
    ("home", "回零"),
)
ROW_Y0, ROW_STEP = 22, 22

MOTOR_TITLES = {
    "lm_L12": "拇指 MCP",
    "lm_L13": "拇指 PIP",
    "rm_R04": "拇指 rota",
    "rm_R05": "拇指 swing",
    "lm_L06": "食指 MCP",
    "lm_L07": "食指 PIP",
    "lm_L08": "食指 DIP",
    "rm_R00": "食指 swing",
    "lm_L09": "中指 MCP",
    "lm_L10": "中指 PIP",
    "lm_L11": "中指 DIP",
    "rm_R01": "中指 swing",
    "lm_L00": "无名指 MCP",
    "lm_L01": "无名指 PIP",
    "lm_L02": "无名指 DIP",
    "rm_R02": "无名指 swing",
    "lm_L03": "小指 MCP",
    "lm_L04": "小指 PIP",
    "lm_L05": "小指 DIP",
    "rm_R03": "小指 swing",
    "rm_R06": "小指 rota",
}

FINGERS = (
    ("groupBox_finger_thumb", "拇指", 370, ("lm_L13", "lm_L12", "rm_R05", "rm_R04")),
    ("groupBox_finger_index", "食指", 500, ("lm_L07", "lm_L08", "lm_L06", "rm_R00")),
    ("groupBox_finger_mid", "中指", 640, ("lm_L10", "lm_L11", "lm_L09", "rm_R01")),
    ("groupBox_finger_ring", "无名指", 790, ("lm_L01", "lm_L02", "lm_L00", "rm_R02")),
    ("groupBox_finger_pinky", "小指", 920, ("lm_L05", "lm_L04", "lm_L03", "rm_R03", "rm_R06")),
)


def _esc(text: str) -> str:
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def _field_rows_xml(key: str) -> str:
    parts = []
    for i, (fk, lbl) in enumerate(FIELDS):
        y = ROW_Y0 + i * ROW_STEP
        parts.append(
            f"""     <widget class="QLabel" name="label_{key}_{fk}">
      <property name="geometry">
       <rect><x>6</x><y>{y}</y><width>56</width><height>18</height></rect>
      </property>
      <property name="text"><string>{_esc(lbl)}</string></property>
     </widget>
     <widget class="QLineEdit" name="lineEdit_{key}_{fk}">
      <property name="geometry">
       <rect><x>64</x><y>{y - 2}</y><width>68</width><height>20</height></rect>
      </property>
      <property name="readOnly"><bool>true</bool></property>
      <property name="alignment">
       <set>Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter</set>
      </property>
      <property name="placeholderText"><string>—</string></property>
     </widget>"""
        )
    return "\n".join(parts)


def motor_box_xml(key: str, x: int, y: int) -> str:
    title = MOTOR_TITLES[key]
    return f"""    <widget class="QGroupBox" name="groupBox_{key}">
     <property name="geometry">
      <rect><x>{x}</x><y>{y}</y><width>{MOTOR_W}</width><height>{MOTOR_H}</height></rect>
     </property>
     <property name="title"><string>{_esc(title)}</string></property>
{_field_rows_xml(key)}
    </widget>"""


def finger_box_xml(name: str, title: str, x: int, keys: tuple[str, ...]) -> str:
    n = len(keys)
    fh = FINGER_TITLE_H + n * MOTOR_H + max(0, n - 1) * MOTOR_GAP + 8
    fw = MOTOR_W + FINGER_INNER_X * 2
    motors = []
    for i, key in enumerate(keys):
        y = FINGER_TITLE_H + i * (MOTOR_H + MOTOR_GAP)
        motors.append(motor_box_xml(key, FINGER_INNER_X, y))
    motors_xml = "\n".join(motors)
    return f"""   <widget class="QGroupBox" name="{name}">
    <property name="geometry">
     <rect><x>{x}</x><y>{FINGER_TOP}</y><width>{fw}</width><height>{fh}</height></rect>
    </property>
    <property name="title"><string>{_esc(title)}</string></property>
{motors_xml}
   </widget>"""


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
    finger_blocks = [finger_box_xml(n, t, x, keys) for n, t, x, keys in FINGERS]
    z_fingers = [f"   <zorder>{n}</zorder>" for n, _, _, _ in FINGERS]

    xml = "\n".join(
        [
            '<?xml version="1.0" encoding="UTF-8"?>',
            '<ui version="4.0">',
            " <class>MainWindow</class>",
            ' <widget class="QMainWindow" name="MainWindow">',
            "  <property name=\"geometry\">",
            f"   <rect><x>0</x><y>0</y><width>{WIN_W}</width><height>{WIN_H}</height></rect>",
            "  </property>",
            "  <property name=\"windowTitle\"><string>遥测回传</string></property>",
            '  <widget class="QWidget" name="centralwidget">',
            f"""   <widget class="QLabel" name="label_image">
    <property name="geometry">
     <rect><x>0</x><y>0</y><width>{IMG_W}</width><height>{IMG_H}</height></rect>
    </property>
    <property name="text"><string></string></property>
    <property name="pixmap">
     <pixmap resource="rsc.qrc">:/hand/right_snapshot.png</pixmap>
    </property>
    <property name="scaledContents"><bool>true</bool></property>
    <property name="alignment">
     <set>Qt::AlignmentFlag::AlignCenter</set>
    </property>
   </widget>""",
            *finger_blocks,
            RX_BOX,
            "   <zorder>label_image</zorder>",
            *z_fingers,
            "   <zorder>groupBox_rx_rate</zorder>",
            "  </widget>",
            '  <widget class="QStatusBar" name="statusbar"/>',
            " </widget>",
            " <resources>",
            "  <include location=\"rsc.qrc\"/>",
            " </resources>",
            " <connections/>",
            "</ui>",
        ]
    )
    UI.write_text(xml, encoding="utf-8")
    print(f"Wrote {UI}")


if __name__ == "__main__":
    main()
