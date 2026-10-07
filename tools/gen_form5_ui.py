"""Generate form5.ui — hand-overlay telemetry widgets (14 linear + 7 rotary).

Controls are placed directly on centralwidget (no scroll area), roughly
matching finger positions on right_snapshot.png. Each motor shows one
converted joint-angle field (deg); user can rearrange in Qt Designer.
"""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "form5.ui"

WIN_W, WIN_H = 1120, 829

# (kind, id, title, x, y) — kind: lm | rm
# Positions approximate palm view: thumb left, four fingers upward.
MOTOR_PLACEMENTS = (
    # 拇指（图左侧）
    ("lm", "L12", "拇指 MCP", 70, 560),
    ("lm", "L13", "拇指 PIP", 70, 480),
    ("rm", "R04", "拇指 rota", 210, 620),
    ("rm", "R05", "拇指 swing", 210, 540),
    # 食指
    ("lm", "L06", "食指 MCP", 330, 650),
    ("lm", "L07", "食指 PIP", 330, 570),
    ("lm", "L08", "食指 DIP", 330, 490),
    ("rm", "R00", "食指 swing", 470, 530),
    # 中指
    ("lm", "L09", "中指 MCP", 520, 650),
    ("lm", "L10", "中指 PIP", 520, 570),
    ("lm", "L11", "中指 DIP", 520, 490),
    ("rm", "R01", "中指 swing", 660, 530),
    # 无名指
    ("lm", "L00", "无名指 MCP", 710, 650),
    ("lm", "L01", "无名指 PIP", 710, 570),
    ("lm", "L02", "无名指 DIP", 710, 490),
    ("rm", "R02", "无名指 swing", 850, 530),
    # 小指
    ("lm", "L03", "小指 MCP", 900, 650),
    ("lm", "L04", "小指 PIP", 900, 570),
    ("lm", "L05", "小指 DIP", 900, 490),
    ("rm", "R03", "小指 swing", 1040, 530),
    ("rm", "R06", "小指 rota", 1040, 610),
)


def _esc(text: str) -> str:
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def _motor_box(kind: str, motor_id: str, title: str, x: int, y: int) -> list[str]:
    key = f"{kind}_{motor_id}"
    return [
        f'   <widget class="QGroupBox" name="groupBox_{key}">',
        "    <property name=\"geometry\">",
        f"     <rect><x>{x}</x><y>{y}</y><width>132</width><height>58</height></rect>",
        "    </property>",
        f"    <property name=\"title\"><string>{_esc(title)}</string></property>",
        f'    <widget class="QLabel" name="label_{key}_caption">',
        "     <property name=\"geometry\">",
        "      <rect><x>8</x><y>22</y><width>48</width><height>18</height></rect>",
        "     </property>",
        "     <property name=\"text\"><string>角度(°)</string></property>",
        "    </widget>",
        f'    <widget class="QLineEdit" name="lineEdit_{key}_angle">',
        "     <property name=\"geometry\">",
        "      <rect><x>56</x><y>20</y><width>68</width><height>22</height></rect>",
        "     </property>",
        "     <property name=\"readOnly\"><bool>true</bool></property>",
        "     <property name=\"alignment\">",
        "      <set>Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter</set>",
        "     </property>",
        "     <property name=\"placeholderText\"><string>—</string></property>",
        "    </widget>",
        "   </widget>",
    ]


def main() -> None:
    motor_blocks: list[str] = []
    for kind, motor_id, title, x, y in MOTOR_PLACEMENTS:
        motor_blocks.extend(_motor_box(kind, motor_id, title, x, y))

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
            '   <widget class="QLabel" name="label_image">',
            "    <property name=\"geometry\">",
            f"     <rect><x>0</x><y>0</y><width>{WIN_W}</width><height>{WIN_H - 49}</height></rect>",
            "    </property>",
            "    <property name=\"pixmap\">",
            "     <pixmap resource=\"rsc.qrc\">:/hand/right_snapshot.png</pixmap>",
            "    </property>",
            "    <property name=\"scaledContents\"><bool>true</bool></property>",
            "    <property name=\"alignment\">",
            "     <set>Qt::AlignmentFlag::AlignCenter</set>",
            "    </property>",
            "    <property name=\"text\"><string></string></property>",
            "   </widget>",
            *motor_blocks,
            "  </widget>",
            '  <widget class="QStatusBar" name="statusbar"/>',
            " </widget>",
            " <resources/>",
            " <connections/>",
            "</ui>",
        ]
    )
    OUT.write_text(xml, encoding="utf-8")
    print(f"Wrote {OUT} ({len(MOTOR_PLACEMENTS)} motor widgets)")


if __name__ == "__main__":
    main()