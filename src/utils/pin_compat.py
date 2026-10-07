"""Pinocchio import helper (conda-forge 4.x vs legacy / wrong pip package)."""

from __future__ import annotations


def import_pinocchio():
    """Return the pinocchio module with robotics APIs available."""
    try:
        import pinocchio as pin
    except ImportError as exc:
        raise ImportError(
            "未安装 Pinocchio。请使用 conda-forge 安装：\n"
            "  conda install -n hand_dex pinocchio pinocchio-python -c conda-forge\n"
            "不要用 pip install pinocchio（那是无关的测试插件包）。"
        ) from exc

    if not hasattr(pin, "buildModelFromUrdf"):
        raise ImportError(
            "当前环境中的 pinocchio 不是机器人学库（缺少 buildModelFromUrdf）。\n"
            "若曾执行过 pip install pinocchio，请先卸载：\n"
            "  pip uninstall pinocchio -y\n"
            "再安装 conda-forge 版本：\n"
            "  conda install -n hand_dex pinocchio pinocchio-python -c conda-forge --force-reinstall"
        )
    return pin
