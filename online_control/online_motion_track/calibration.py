"""Per-browser calibration drafts using the desktop editor's solver and presets."""
from copy import deepcopy
from dataclasses import replace
from datetime import datetime
import hashlib
import os
import tempfile
import threading
import time

import numpy as np
import yaml

from hand_profiles import A30_MODEL_IDS, A32_MODEL_IDS, load_hand_profile
from keypoint_preview_solver import PreviewSolver
from keypoint_mediapipe_input import align_world_landmarks, palm_frame
from keypoint_pose_presets import COMMON_PRESETS, KAPANDJI_LABELS, make_preset, model_flexion_hint
from online_control.online_motion_track.landmark_filter import ResponsiveLandmarkFilter


def coordinates(value):
    arr = np.asarray(value, dtype=float)
    if arr.shape != (21, 3) or not np.isfinite(arr).all() or np.abs(arr).max() > 10:
        raise ValueError('需要有限的 21×3 三维坐标，单位为米')
    palm_frame(arr)
    return arr.copy()


class CalibrationSession:
    """No control adapter or hardware target; all edits remain in this draft."""
    file_lock = threading.Lock()

    def __init__(self, side='left', *, model_id='TsenseI_v3_2_withid', input_source='mediapipe'):
        if model_id not in (*A32_MODEL_IDS, *A30_MODEL_IDS) or side not in ('left', 'right'):
            raise ValueError('动态标定支持 A3.0 / A3.2（含 with ID）的左右手')
        self.model_id, self.side = model_id, side
        self.input_source, self.input_side = input_source, side
        self.profile = load_hand_profile(model_id)
        profile = replace(self.profile, retarget_options={**self.profile.retarget_options, 'kinematics_backend': 'numpy'})
        self.config = profile.create_retargeter(side).config
        self.engine = PreviewSolver(self.config)
        self.zero_poses = self.engine.solver.fk.model.forward_kinematics({})
        self.path = profile.path('keypoints', side)
        self.lock = threading.RLock()
        self.closed = False
        self.created_at = self.last_seen = time.monotonic()
        self.input_active, self.input_epoch = False, 0
        self.last_sequence, self.last_timestamp = -1, -1.
        self.filter = ResponsiveLandmarkFilter()
        self._read()
        self.input_points = self.world_points()
        self.reference = self.input_points.copy()
        self.result = None
        self._solve()

    def _read(self):
        raw = self.path.read_bytes()
        data = yaml.safe_load(raw)
        self._validated_rows(data['keypoints'], data['keypoints'])
        data['keypoints'].sort(key=lambda row: row['id'])
        self.data = data
        self.saved_hash = hashlib.sha256(raw).hexdigest()

    def _validated_rows(self, rows, baseline):
        if not isinstance(rows, list) or len(rows) != 21:
            raise ValueError('标定必须包含 0–20 共 21 个点')
        if any(type(row.get('id')) is not int for row in rows) or sorted(row['id'] for row in rows) != list(range(21)):
            raise ValueError('标定点编号必须为不重复的 0–20')
        result = []
        original = {row['id']: row for row in baseline}
        for row in sorted(rows, key=lambda row: row['id']):
            xyz = np.asarray(row.get('local_xyz'), dtype=float)
            if row.get('link') not in self.zero_poses or xyz.shape != (3,) or not np.isfinite(xyz).all() or np.abs(xyz).max() > 10:
                raise ValueError(f"点 {row['id']} 的连杆或局部坐标无效")
            new = deepcopy(original[row['id']])
            if new['link'] != row['link'] or new['local_xyz'] != xyz.tolist():
                new['annotation_status'] = 'manually_adjusted'
            new.update(link=row['link'], local_xyz=xyz.tolist())
            result.append(new)
        coordinates(self.world_points(result))
        return result

    def world_points(self, rows=None):
        rows = self.data['keypoints'] if rows is None else rows
        return np.asarray([(self.zero_poses[row['link']] @ np.r_[row['local_xyz'], 1])[:3]
                           for row in sorted(rows, key=lambda row: row['id'])])

    def _apply_rows(self, data):
        if 'calibration_rows' in data:
            self.data['keypoints'] = self._validated_rows(data['calibration_rows'], self.data['keypoints'])

    def _touch(self):
        if self.closed:
            raise ValueError('标定会话已结束，请刷新页面')
        self.last_seen = time.monotonic()

    def _solve(self):
        try:
            self.result = self.engine.solve(self.input_points, self.data)
        except Exception:
            self.engine = PreviewSolver(self.config)
            raise
        return dict(pose={**self.result['joint_values'], **self.profile.fixed_joints},
                    result_points=self.result['points'].tolist(), input_points=self.input_points.tolist(),
                    solve_ms=self.result['elapsed_ms'],
                    delta=self.result['delta'] if np.isfinite(self.result['delta']) else None)

    def description(self):
        return dict(model_id=self.model_id, side=self.side, mode='calibration',
                    calibration_rows=deepcopy(self.data['keypoints']), calibration_points=self.world_points().tolist(),
                    input_points=self.input_points.tolist(), pose=self.result['joint_values'],
                    result_points=self.result['points'].tolist(), path=str(self.path),
                    presets=[dict(id=key, label=label) for key, label in COMMON_PRESETS] +
                            [dict(id=f'kapandji_{i}', label=f'K{i} · {label}') for i, label in enumerate(KAPANDJI_LABELS, 1)])

    def set_input(self, data):
        with self.lock:
            self._touch()
            source, side = data.get('source', self.input_source), data.get('side', self.input_side)
            if source not in ('mediapipe', 'webxr', 'wuji_glove', 'leapmotion', 'realsense_rgbd') or side not in ('left', 'right'):
                raise ValueError('输入源或手别无效')
            if type(data.get('enabled')) is not bool:
                raise ValueError('enabled 必须为布尔值')
            self._apply_rows(data)
            self.input_source, self.input_side = source, side
            self.input_epoch += 1
            self.input_active = data['enabled']
            self.reference = self.world_points()
            self.last_sequence, self.last_timestamp = -1, -1.
            self.filter.reset()
            return dict(input_epoch=self.input_epoch, active=self.input_active)

    def process(self, packet):
        with self.lock:
            self._touch()
            if not self.input_active or packet.get('input_epoch') != self.input_epoch:
                raise ValueError('输入已冻结，丢弃旧帧')
            sequence, timestamp = int(packet['sequence']), float(packet['timestamp_ms'])
            if not np.isfinite(timestamp) or sequence <= self.last_sequence or timestamp <= self.last_timestamp:
                raise ValueError('丢弃过期或重复输入帧')
            self.last_sequence, self.last_timestamp = sequence, timestamp
            self._apply_rows(packet)
            hand = packet.get('hand')
            expected = self.input_side.capitalize()
            valid = hand is not None and hand.get('label') == expected
            message = '21 点实时输入' if valid else f'未检测到{ "左" if self.input_side == "left" else "右"}手，保持最后姿态'
            if valid:
                raw = coordinates(hand['world'])
                filtered = (self.filter.filter(raw, timestamp / 1000.)
                            if self.input_source == "mediapipe" else raw)
                self.input_points = align_world_landmarks(filtered, self.reference, self.input_side, self.side)
            return dict(**self._solve(), tracking=valid, message=message, source=self.input_source,
                        sequence=sequence, edit_revision=packet.get('edit_revision'), input_epoch=self.input_epoch)

    def process_metric(self, packet, commit):
        """Keep an expired metric observation out of the editable draft too."""
        if not callable(commit):
            raise TypeError('metric processing requires a commit callback')
        with self.lock:
            previous_points = self.input_points.copy()
            previous_result = deepcopy(self.result)
            previous_data = deepcopy(self.data)
            previous_sequence, previous_timestamp = self.last_sequence, self.last_timestamp

            def restore():
                self.input_points, self.result, self.data = previous_points, previous_result, previous_data
                self.last_sequence, self.last_timestamp = previous_sequence, previous_timestamp
                # The expired solve can leave an IK warm start even if the
                # draft arrays are restored. Build a fresh estimator for the
                # next valid frame; do not re-solve the discarded observation.
                self.engine = PreviewSolver(self.config)
                self.filter.reset()

            try:
                response = self.process(packet)
                try:
                    accepted = commit(lambda: None)
                except ValueError:
                    accepted = False
            except Exception:
                restore()
                raise
            if accepted:
                return response
            restore()
            return dict(response, tracking=False, display_discarded=True,
                        discard_reason='source_cancelled_or_expired',
                        message='输入已取消或过期，保持上次标定结果',
                        pose={**self.result['joint_values'], **self.profile.fixed_joints},
                        result_points=self.result['points'].tolist(),
                        input_points=self.input_points.tolist(),
                        solve_ms=self.result['elapsed_ms'],
                        delta=self.result['delta'] if np.isfinite(self.result['delta']) else None)

    def update(self, data):
        with self.lock:
            self._touch()
            self._apply_rows(data)
            preset = None
            if 'preset' in data or 'input_points' in data:
                if self.input_active:
                    raise ValueError('请先冻结输入，再拖动骨架或应用预设')
                if 'preset' in data:
                    preset = make_preset(self.world_points(), data['preset'],
                                         model_flexion_hint(self.engine.solver.fk.model, self.data['keypoints']))
                    self.input_points = preset.points
                else:
                    self.input_points = coordinates(data['input_points'])
            result = dict(**self._solve(), edit_revision=data.get('edit_revision'))
            if preset:
                result['preset'] = dict(label=preset.label, target=None if preset.target is None else preset.target.tolist(),
                                        gap_mm=preset.gap_m * 1000)
            return result

    def export(self, data):
        with self.lock:
            self._touch()
            self._apply_rows(data)
            return dict(filename=self.path.name, yaml=yaml.safe_dump(self.data, sort_keys=False, allow_unicode=True, width=120))

    def save(self, data):
        with self.lock, self.file_lock:
            exported = self.export(data)
            raw = self.path.read_bytes()
            if hashlib.sha256(raw).hexdigest() != self.saved_hash:
                raise ValueError('标定文件已被其他页面或编辑器修改；请先导出当前草稿，再重新加载')
            backup = self.path.with_name(self.path.name + '.bak_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            backup.write_bytes(raw)
            fd, temporary = tempfile.mkstemp(prefix=self.path.name + '.', suffix='.tmp', dir=self.path.parent)
            try:
                with os.fdopen(fd, 'w', encoding='utf-8', newline='\n') as stream:
                    stream.write(exported['yaml'])
                os.replace(temporary, self.path)
            finally:
                if os.path.exists(temporary):
                    os.unlink(temporary)
            self.saved_hash = hashlib.sha256(self.path.read_bytes()).hexdigest()
            return dict(path=str(self.path), backup=str(backup), calibration_rows=deepcopy(self.data['keypoints']))

    def reload(self):
        with self.lock:
            self._touch()
            self._read()
            self._solve()
            return self.description()

    def close(self):
        with self.lock:
            self.closed = True
            self.input_active = False
