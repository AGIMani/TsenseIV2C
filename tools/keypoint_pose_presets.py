"""Geometric source-skeleton presets, independent of retargeted joint output.

Kapandji target order: https://pubmed.ncbi.nlm.nih.gov/3963909/ (1986).
Skin creases/radial surfaces are proxies derived from the 21-point skeleton.
These are editable calibration trials, not measured or scored clinical poses.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


KAPANDJI_LABELS = (
    '食指中节桡侧', '食指末节桡侧', '食指尖', '中指尖', '无名指尖', '小指尖',
    '小指 DIP 掌侧纹', '小指 PIP 掌侧纹', '小指根部掌侧纹', '掌部远侧横纹',
)
COMMON_PRESETS = (
    ('relaxed', '半握拳'), ('fist', '握拳'), ('hook', '钩拳'), ('neutral', '标定零位'),
    ('pinch_index', '拇–食对指'), ('pinch_middle', '拇–中对指'),
    ('pinch_ring', '拇–无名对指'), ('pinch_little', '拇–小对指'),
)


@dataclass
class PosePreset:
    points: np.ndarray
    label: str
    target: np.ndarray | None = None
    gap_m: float = 0.0


def _unit(vector):
    length = float(np.linalg.norm(vector))
    if length < 1e-10:
        raise ValueError('标定骨架存在重合点，无法生成姿态')
    return np.asarray(vector, dtype=float) / length


def model_flexion_hint(model, rows):
    """Use the model's flexion direction, so CAD axes and mirrors are respected."""
    joint = next((name for name in model.joint_names() if 'index_MCP' in name), None)
    if joint is None:
        raise ValueError('当前模型缺少食指 MCP，无法确定掌侧方向')
    row = next(row for row in rows if row['id'] == 8)
    local = np.r_[row['local_xyz'], 1.0]
    zero = model.forward_kinematics({})[row['link']] @ local
    flexed = model.forward_kinematics({joint: .25})[row['link']] @ local
    return _unit((flexed - zero)[:3])


def _curl(points, base, normal, angles):
    """Bend a three-segment finger in its flexion plane without stretching it."""
    result = points.copy()
    forward = _unit(points[base + 3] - points[base])
    inward = _unit(normal - forward * np.dot(normal, forward))
    lengths = np.linalg.norm(np.diff(points[base:base + 4], axis=0), axis=1)
    if np.any(lengths < 1e-8):
        raise ValueError('标定骨节长度过小，无法生成姿态')
    for offset, (length, angle) in enumerate(zip(lengths, np.cumsum(np.deg2rad(angles))), 1):
        result[base + offset] = result[base + offset - 1] + length * (
            forward * np.cos(angle) + inward * np.sin(angle))
    return result


def _thumb_to_target(points, target, normal):
    """FABRIK with fixed CMC and segment lengths; unreachable targets leave a gap."""
    chain = points[1:5].copy()
    lengths = np.linalg.norm(np.diff(chain, axis=0), axis=1)
    if np.any(lengths < 1e-8):
        raise ValueError('拇指标定骨节长度过小，无法生成对指')
    root = chain[0].copy()
    reach = lengths.sum()
    distance = float(np.linalg.norm(target - root))
    if distance >= reach:
        direction = _unit(target - root)
        chain[1:] = root + np.cumsum(lengths)[:, None] * direction
        return chain
    # Seed an arc on the palm side to avoid a straight-chain singularity.
    direction = _unit(target - root) if distance > 1e-10 else _unit(points[5] - root)
    bend = normal - direction * np.dot(normal, direction)
    if np.linalg.norm(bend) < 1e-8:
        bend = points[5] - points[17]
        bend -= direction * np.dot(bend, direction)
    bend = _unit(bend)
    t = np.r_[0, np.cumsum(lengths) / reach]
    chain = root + t[:, None] * (target - root) + np.sin(np.pi * t)[:, None] * bend * reach * .3
    for _ in range(160):
        chain[-1] = target
        for j in range(2, -1, -1):
            chain[j] = chain[j + 1] + _unit(chain[j] - chain[j + 1]) * lengths[j]
        chain[0] = root
        for j in range(3):
            chain[j + 1] = chain[j] + _unit(chain[j + 1] - chain[j]) * lengths[j]
        if np.linalg.norm(chain[-1] - target) < reach * 1e-6:
            break
    return chain


def kapandji_target(points, stage, radial, normal, width):
    if stage == 1:
        return (points[6] + points[7]) / 2 + radial * width * .045
    if stage == 2:
        return (points[7] + points[8]) / 2 + radial * width * .04
    if 3 <= stage <= 6:
        return points[(8, 12, 16, 20)[stage - 3]].copy()
    if 7 <= stage <= 9:
        return points[{7: 19, 8: 18, 9: 17}[stage]] + normal * width * .045
    if stage == 10:
        # Approximate the ulnar end of the distal palmar crease, proximal to MCP.
        return points[17] + (points[0] - points[17]) * .15 + normal * width * .045
    raise ValueError('Kapandji 位置必须为 K1–K10')


def make_preset(reference, preset_id, flexion_hint):
    reference = np.asarray(reference, dtype=float)
    if reference.shape != (21, 3) or not np.isfinite(reference).all():
        raise ValueError('预设需要 21 个有效三维标定点')
    points = reference.copy()
    if preset_id == 'neutral':
        return PosePreset(points, '标定零位')
    radial = _unit(points[5] - points[17])
    longitudinal = _unit(points[[5, 9, 13, 17]].mean(axis=0) - points[0])
    normal = _unit(np.cross(radial, longitudinal))
    if np.dot(normal, flexion_hint) < 0:
        normal = -normal
    width = float(np.linalg.norm(points[5] - points[17]))
    curls = {'relaxed': (25, 40, 20), 'fist': (70, 85, 55), 'hook': (0, 85, 55)}
    if preset_id in curls:
        for base in (5, 9, 13, 17):
            points = _curl(points, base, normal, curls[preset_id])
        return PosePreset(points, dict(COMMON_PRESETS)[preset_id])
    aliases = dict(zip(('pinch_index', 'pinch_middle', 'pinch_ring', 'pinch_little'), range(3, 7)))
    if preset_id in aliases:
        stage = aliases[preset_id]
        label = dict(COMMON_PRESETS)[preset_id]
    elif preset_id.startswith('kapandji_') and preset_id.removeprefix('kapandji_').isdigit():
        stage = int(preset_id.removeprefix('kapandji_'))
        if not 1 <= stage <= 10:
            raise ValueError('Kapandji 位置必须为 K1–K10')
        label = f'K{stage} · {KAPANDJI_LABELS[stage - 1]}'
    else:
        raise ValueError(f'未知骨架预设：{preset_id}')
    reach = np.linalg.norm(np.diff(points[1:5], axis=0), axis=1).sum()
    base = 5 if stage <= 3 else {4: 9, 5: 13}.get(stage, 17)
    # Bend only the contact finger to bring the target within thumb reach.
    # Rebuild from the reference every time; changing presets never accumulates.
    if stage <= 8:
        candidates = []
        for angle in np.linspace(0, 85, 86):
            trial = _curl(reference, base, normal, (angle, angle, angle * .6))
            target = kapandji_target(trial, stage, radial, normal, width)
            distance = np.linalg.norm(target - trial[1])
            score = abs(distance - reach * .85) + reach * .02 * angle / 85
            candidates.append((score, trial, target))
        _, points, target = min(candidates, key=lambda item: item[0])
    else:
        target = kapandji_target(points, stage, radial, normal, width)
    points[1:5] = _thumb_to_target(points, target, normal)
    return PosePreset(points, label, target.copy(), float(np.linalg.norm(points[4] - target)))
