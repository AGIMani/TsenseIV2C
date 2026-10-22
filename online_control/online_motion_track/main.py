"""V2/V3.2 web UI: browser GPU detection, Python retarget and local V2 control.

python online_control/online_motion_track/main.py --port 8766
Install browser dependencies once: npm ci --prefix online_control/online_motion_track
"""
from __future__ import annotations

import argparse
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import ipaddress
import importlib.util
import mimetypes
import os
from pathlib import Path
import shutil
import signal
import socket
import ssl
import sys
import threading
import time
from types import SimpleNamespace
from urllib.parse import parse_qs, unquote, urlsplit
from uuid import uuid4
import webbrowser
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "src/tracking"), str(ROOT / "tools")]
from hand_profiles import A30_MODEL_IDS, A32_MODEL_IDS, load_hand_profile
from wuji_mediapipe import _OneEuroLandmarkFilter
from online_control.online_motion_track.landmark_filter import ResponsiveLandmarkFilter, A32LandmarkFilter
from online_control.online_motion_track.webxr_input import WebXRInputRelay
from online_control.online_motion_track.wuji_input import WujiInputRelay, resolve_wuji_config
from online_control.online_motion_track.leapmotion_input import LeapMotionInputRelay
from online_control.online_motion_track.leap_camera import LeapCameraSource, encode_frame
from online_control.online_motion_track.realsense_source import RealSenseSource
from online_control.online_motion_track.source_worker import SourceWorker
from online_control.online_motion_track.optimized_skeleton import optimized_skeleton
from online_control.online_motion_track.calibration import CalibrationSession
from online_control.shared.stream import serve_records
from online_control.shared.version import downlink_version
from online_control.online_motion_track.pipeline_v3_2.pipeline import MediaPipeHandPipeline
from online_control.online_motion_track.pipeline_v3_2.recording import SessionRecorder, json_safe
from utils.control_log_catalog import a30_catalog, a30_file, control_log_catalog, control_log_file, set_control_log_parent


def vector(element, key, default="0 0 0"):
    return [float(v) for v in (element.get(key, default) if element is not None else default).split()]


def origin(element):
    return {"xyz": vector(element, "xyz"), "rpy": vector(element, "rpy")}


MODEL_IDS = (*A32_MODEL_IDS, *A30_MODEL_IDS, "TsenseI_v2")
ASSET_PREFIXES = {"TsenseI_v3_2_withid": "/assets-v3-2-withid/", "TsenseI_v3_0_withid": "/assets-v3-0-withid/", "TsenseI_v3_2": "/assets/", "TsenseI_v3_0": "/assets-v3-0/", "TsenseI_v2": "/assets-v2/"}


def model_description(side, model_id="TsenseI_v3_2"):
    if side not in ("left", "right"):
        raise ValueError("手别必须为 left 或 right")
    if model_id not in MODEL_IDS:
        raise ValueError("未知手型")
    profile = load_hand_profile(model_id)
    urdf = profile.path("urdf", side)
    robot = ET.parse(urdf).getroot()
    links, joints = [], []
    for link in robot.findall("link"):
        visuals = []
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                raise ValueError("V3.2 viewer expects original mesh visuals")
            path = (urdf.parent / mesh.get("filename")).resolve()
            relative = path.relative_to(profile.root.resolve()).as_posix()
            color = visual.find("material/color")
            prefix = ASSET_PREFIXES[model_id]
            visuals.append({**origin(visual.find("origin")), "mesh": prefix + relative,
                            "scale": vector(mesh, "scale", "1 1 1"),
                            "color": vector(color, "rgba", "0.8 0.82 0.85 1")})
        links.append({"name": link.get("name"), "visuals": visuals})
    for joint in robot.findall("joint"):
        limit = joint.find("limit")
        joints.append({"name": joint.get("name"), "type": joint.get("type"),
                       "parent": joint.find("parent").get("link"),
                       "child": joint.find("child").get("link"),
                       **origin(joint.find("origin")), "axis": vector(joint.find("axis"), "xyz", "1 0 0"),
                       "lower": float(limit.get("lower", 0)) if limit is not None else 0,
                       "upper": float(limit.get("upper", 0)) if limit is not None else 0})
    children = {j["child"] for j in joints}
    return {"side": side, "model_id": model_id, "links": links, "joints": joints,
            "root": next(link["name"] for link in links if link["name"] not in children),
            "initial_pose": profile.init_pose(side), "calibration": profile.calibration_status(side)}


def points(value):
    arr = np.asarray(value, dtype=float)
    if arr.shape != (21, 3) or not np.isfinite(arr).all() or np.abs(arr).max() > 10:
        raise ValueError("需要有限的 21×3 三维关键点")
    return arr


class StaleFrameError(ValueError):
    """An obsolete observation, not a solver or control safety fault."""


class WebSession:
    model_id = "TsenseI_v3_2"
    def __init__(self, side="left", recording=True, log_dir=None, collision_mode="off", *, model_id="TsenseI_v3_2",
                 input_source="mediapipe"):
        if input_source not in ("mediapipe", "webxr", "wuji_glove", "leapmotion", "realsense_rgbd"):
            raise ValueError("关键点来源必须为 mediapipe、webxr、wuji_glove、leapmotion 或 realsense_rgbd")
        self.input_source = input_source
        self.mediapipe_filter_enabled = True
        self.direct_mode = False
        if model_id not in (*A32_MODEL_IDS, *A30_MODEL_IDS):
            raise ValueError("Unsupported preview model")
        self.model_id = model_id
        if side not in ("left", "right"):
            raise ValueError("手别无效")
        if model_id in A30_MODEL_IDS and collision_mode not in a30_collision_modes(model_id):
            raise ValueError("A3.0 碰撞模式不可用，请安装完整依赖并检查 MJCF 模型")
        if model_id in A32_MODEL_IDS and collision_mode not in a32_collision_modes(model_id):
            raise ValueError("A3.2 碰撞模式不可用，请检查依赖与模型资源")
        if collision_mode == "mink" and not mink_available(model_id):
            raise ValueError("Mink 未安装或缺少模型，请安装 requirements-mink.txt")
        pipeline_type = MediaPipeHandPipeline
        if model_id in A30_MODEL_IDS:
            from online_control.online_motion_track.pipeline_v3_0.pipeline import MediaPipeHandPipeline as pipeline_type
        self.pipeline = pipeline_type(side, collision_mode, model_id=model_id, retarget_options={"kinematics_backend": "numpy"})
        self.lock = threading.Lock()
        self.closed = False
        self.last_seen = time.monotonic()
        self.created_at = self.last_seen
        self.last_sequence = -1
        self.last_timestamp = -1.0
        self.filter = (A32LandmarkFilter() if model_id in A32_MODEL_IDS else
                       ResponsiveLandmarkFilter() if model_id in A30_MODEL_IDS else
                       _OneEuroLandmarkFilter(min_cutoff=1.5, beta=.25, derivative_cutoff=1.))
        self.normalized_filter = _OneEuroLandmarkFilter(min_cutoff=1.5, beta=.25, derivative_cutoff=1.)
        self.recorder = SessionRecorder(log_dir or HERE / "logs", self.pipeline,
            camera_source={"webxr": "quest3-webxr", "wuji_glove": "wuji-glove", "leapmotion": "leap-motion-2", "realsense_rgbd": "realsense-rgbd"}.get(input_source, "browser-webcam"),
            smoothing=input_source == "mediapipe", input_source=input_source) if recording else None

    def process(self, packet, *, touch_client=True):
        with self.lock:
            if self.closed:
                raise ValueError("会话已停止，请重新开始")
            sequence = int(packet["sequence"])
            timestamp = float(packet["timestamp_ms"])
            if not np.isfinite(timestamp):
                raise ValueError("帧时间戳必须为有限值")
            if timestamp <= self.last_timestamp or sequence <= self.last_sequence:
                raise StaleFrameError("丢弃过期或重复帧")
            input_source = packet.get("source", self.input_source)
            if input_source not in ("mediapipe", "webxr", "wuji_glove", "leapmotion", "realsense_rgbd"):
                raise ValueError("关键点来源必须为 mediapipe、webxr、wuji_glove、leapmotion 或 realsense_rgbd")
            filter_enabled = packet.get("mediapipe_filter_enabled", self.mediapipe_filter_enabled)
            if not isinstance(filter_enabled, bool):
                raise ValueError("mediapipe_filter_enabled 必须为布尔值")
            if filter_enabled != self.mediapipe_filter_enabled:
                self.filter.reset()
                self.normalized_filter.reset()
                self.mediapipe_filter_enabled = filter_enabled
            if input_source != self.input_source:
                self.filter.reset()
                self.normalized_filter.reset()
                self.pipeline.session.retargeter.reset()
                self.input_source = input_source
            capture_ns = int(packet["capture_unix_ms"] * 1_000_000)
            camera_size = packet.get("camera_size", [640, 480]) if input_source in ("mediapipe", "realsense_rgbd") else None
            observation = None
            hand = packet.get("hand")
            if hand is not None:
                raw = points(hand["world"])
                # WebXR supplies metric 3-D joints, with no image projection.
                normalized = points(hand["normalized"]) if input_source in ("mediapipe", "realsense_rgbd") else None
                expected = self.pipeline.expected_label
                label = hand.get("label")
                if label not in ("Left", "Right"):
                    raise ValueError("检测手别无效")
                # Unselected/lost hands never enter the filter or the solver.
                if label == expected:
                    if timestamp - self.last_timestamp > 300:
                        self.filter.reset()
                        self.normalized_filter.reset()
                    # Metric trackers already provide measured 3-D joints.
                    # MediaPipe smoothing is independently selectable per observation.
                    world = (self.filter.filter(raw, timestamp / 1000)
                             if input_source == "mediapipe" and filter_enabled else raw.copy())
                    filtered_normalized = (self.normalized_filter.filter(normalized, timestamp / 1000)
                                           if normalized is not None and (input_source != "mediapipe" or filter_enabled)
                                           else normalized.copy() if normalized is not None else None)
                else:
                    world = raw
                    filtered_normalized = normalized
                    self.filter.reset()
                    self.normalized_filter.reset()
                observation = SimpleNamespace(
                    raw_world_points=raw, world_points=world, points=world,
                    raw_normalized_points=normalized, normalized_points=filtered_normalized,
                    detected_label=label, expected_label=expected,
                    mediapipe_filter_enabled=input_source == "mediapipe" and filter_enabled,
                    source=input_source, source_metadata=packet.get("source_metadata"),
                    raw_webxr=packet.get("raw_webxr"),
                    held_last_valid=False, used_fallback_hand=False)
            else:
                self.filter.reset()
                self.normalized_filter.reset()
            result = self.pipeline.process(observation)
            if self.recorder:
                self.recorder.record_frame(sequence, observation, result,
                    capture_timestamp_ns=capture_ns, camera_size=camera_size, input_source=input_source,
                    mediapipe_filter_enabled=input_source == "mediapipe" and filter_enabled,
                    source_metadata=packet.get("source_metadata"), raw_webxr=packet.get("raw_webxr"))
            self.last_sequence, self.last_timestamp = sequence, timestamp
            if touch_client:
                self.last_seen = time.monotonic()
            return json_safe({"sequence": sequence, "tracking": result.tracking,
                              "direct_mode": self.direct_mode,
                              "mediapipe_filter_enabled": input_source == "mediapipe" and filter_enabled,
                              "source": input_source, "depth_source": result.depth_source,
                              "source_metadata": packet.get("source_metadata"),
                              "message": result.message, "pose": result.pose,
                              "retarget_joint_angles_rad": result.retarget_pose,
                              "mink_joint_angles_rad": result.mink_pose,
                              "direct_joint_angles_rad": result.direct_pose,
                              "collision_safe_target_joint_angles_rad": result.pose,
                              "optimized_points": optimized_skeleton(
                                  self.pipeline.session.retargeter, result.retarget_pose)
                                  if result.tracking else None,
                              "points": result.points, "solve_ms": result.solve_ms,
                              "actuation": result.actuation,
                              "collision": result.collision_diagnostics})

    def process_metric(self, packet, commit):
        """Preview-only commit check; controller sessions guard their target push."""
        result = self.process(packet)
        if not commit(lambda: None):
            return {**result, 'tracking': False, 'display_discarded': True,
                    'discard_reason': 'source_cancelled_or_expired',
                    'message': '输入已取消或过期，等待新数据'}
        return result

    def close(self):
        with self.lock:
            if not self.closed:
                self.closed = True
                if self.recorder:
                    self.recorder.close(reason="web_session_stopped")


def mink_available(model_id="TsenseI_v3_2"):
    return (all(importlib.util.find_spec(name) is not None for name in ("mink", "mujoco", "qpsolvers"))
            and (HERE / "mink_guard.py").is_file()
            and all(load_hand_profile(model_id).path("mjcf", side).is_file()
                    for side in ("left", "right")))


def a30_collision_modes(model_id="TsenseI_v3_0"):
    modes = ["off"]
    profile = load_hand_profile(model_id)
    if (importlib.util.find_spec("mujoco") is not None
            and (ROOT / "src/physics/mujoco_filter.py").is_file()
            and all(profile.path("mjcf", side).is_file() for side in ("left", "right"))):
        modes.append("monitor")
        if (ROOT / "online_control/online_motion_track/pipeline_v2/collision_guard.py").is_file():
            modes.append("stop")
        if mink_available(model_id):
            modes.append("mink")
    return modes


def v2_available():
    return (all(importlib.util.find_spec(name) is not None for name in ("mujoco", "ruckig", "serial"))
            and (ROOT / "online_control/online_motion_track/v2/session.py").is_file()
            and (ROOT / "assets/hands/TsenseI_hand/hand_profile.yaml").is_file())


def a30_control_available():
    return (importlib.util.find_spec("ruckig") is not None
            and (ROOT / "online_control/online_motion_track/a30/session.py").is_file()
            and "stop" in a30_collision_modes())


def a32_collision_modes(model_id="TsenseI_v3_2"):
    # The path checker consumes a profile and has no A30 wire assumptions.
    return a30_collision_modes(model_id)


def a32_control_available():
    if (importlib.util.find_spec("ruckig") is None
            or not (ROOT / "src/a32_driver/__init__.py").is_file()):
        return False
    if sys.platform == "win32":
        return True
    if sys.platform.startswith("linux"):
        # File discovery only: capability checks must never open a device.
        from a32_driver.native import resolve_native_library
        return resolve_native_library().is_file()
    return False


class Application:
    def __init__(self, log_dir=None, *, public=False, allowed_hosts=(), max_sessions=4, collision_mode="off",
                 wuji_source=None, wuji_url=None, wuji_address=None, wuji_serial=None):
        self.sessions = {}
        self.lock = threading.Lock()
        self.log_dir = log_dir
        self.public = public
        self.allowed_hosts = {h.lower() for h in allowed_hosts} | {"127.0.0.1", "localhost"}
        self.max_sessions = max_sessions
        self.owners = {}
        self.collision_mode = collision_mode
        self.mink_available = mink_available()
        self.v2_available = v2_available()
        self.hardware_lock = threading.RLock()
        self.hardware_owner = None
        self.recordings = {}
        self.recordings_ready = set()
        self.webxr = WebXRInputRelay()
        self.quest_connection = {}
        self.quest_adb_connection = {}
        self.wuji = WujiInputRelay(wuji_url, source=wuji_source, address=wuji_address, serial=wuji_serial)
        self.leapmotion = LeapMotionInputRelay()
        self.leap_camera = LeapCameraSource()
        self.realsense = RealSenseSource()
        self.leapmotion_frame_lock = threading.Lock()
        self.metric_workers = {}

    def _replace_metric_worker(self, token, session, relay):
        previous = self.metric_workers.pop(token, None)
        if previous:
            previous.cancel()
        if session.model_id in A32_MODEL_IDS and hasattr(session, 'control'):
            if session.input_source != 'wuji_glove' or self.wuji.supports_background:
                self.metric_workers[token] = SourceWorker(token, session, relay, session.input_source)
                return True
        return False

    def _metric_result(self, data, session):
        worker = self.metric_workers.get(data['session'])
        if worker is None or (session.input_source == 'wuji_glove' and not self.wuji.supports_background):
            return None
        worker.touch_client()
        return worker.read(data.get('after_revision'))

    def create(self, data, owner="local"):
        self.expire()
        input_source = data.get("input_source", data.get("source", "mediapipe"))
        camera_source = data.get('camera_source', 'realsense_rgbd' if input_source == 'realsense_rgbd' else 'browser')
        self._validate_camera_source(camera_source, input_source)
        if input_source not in ("mediapipe", "webxr", "wuji_glove", "leapmotion", "realsense_rgbd"):
            raise ValueError("输入源必须为 mediapipe、webxr、wuji_glove、leapmotion 或 realsense_rgbd")
        direct_mode = data.get("direct_mode", False)
        if not isinstance(direct_mode, bool):
            raise ValueError("direct_mode 必须为布尔值")
        model_id = data.get("model_id", "TsenseI_v3_2")
        if model_id not in MODEL_IDS:
            raise ValueError("未知手型")
        if direct_mode and model_id in (*A30_MODEL_IDS, *A32_MODEL_IDS):
            # A3 direct control is a complete mode: no target/sample collision
            # correction and no Ruckig, including preview-only deployments.
            data = {**data, "collision_mode": "off"}
        with self.lock:
            if len(self.sessions) >= self.max_sessions:
                raise ValueError("当前服务繁忙，请稍后重试")
            if self.public and list(self.owners.values()).count(owner) >= 2:
                raise ValueError("每个访问地址最多开启两个会话，请先停止其他页面")
            if data.get('mode') == 'calibration':
                session = CalibrationSession(data.get('side', 'left'), model_id=model_id, input_source=input_source)
            elif model_id == "TsenseI_v2":
                if not self.v2_available:
                    raise ValueError("V2 运行环境未安装，请安装 online_control/online_motion_track/v2/requirements.txt")
                from online_control.online_motion_track.v2.session import V2WebSession
                session = V2WebSession(data.get("side", "left"),
                    False if self.public else bool(data.get("recording", True)),
                    Path(self.log_dir) / "v2" if self.log_dir else None,
                    collision_mode=data.get("collision_mode", "avoid"), input_source=input_source)
            elif model_id in A30_MODEL_IDS and a30_control_available():
                from online_control.online_motion_track.a30.session import A30WebSession
                session = A30WebSession(data.get("side", "left"),
                    False if self.public else bool(data.get("recording", True)), self.log_dir,
                    collision_mode=data.get("collision_mode", "stop"), model_id=model_id, input_source=input_source)
            elif model_id in A32_MODEL_IDS and a32_control_available() and not self.public:
                from online_control.online_motion_track.a32.session import A32WebSession
                session = A32WebSession(data.get("side", "left"),
                    bool(data.get("recording", True)), self.log_dir,
                    collision_mode=data.get("collision_mode", self.collision_mode), model_id=model_id, input_source=input_source)
            else:
                session = WebSession(data.get("side", "left"),
                                     False if self.public else bool(data.get("recording", True)), self.log_dir,
                                     collision_mode=data.get("collision_mode", "off" if model_id in A30_MODEL_IDS else self.collision_mode),
                                     model_id=model_id, input_source=input_source)
            session.input_source = input_source
            session.camera_source = camera_source
            session.input_side = data.get("side", "left")
            if direct_mode:
                try:
                    if hasattr(session, "control"):
                        session.command("set_direct_mode", {"enabled": True})
                    elif isinstance(session, WebSession):
                        session.direct_mode = True
                        session.pipeline.collision_bypass = True
                except Exception:
                    session.close()
                    raise
            token = uuid4().hex
            self.sessions[token] = session
            self.owners[token] = owner
            if isinstance(session, CalibrationSession):
                return {'session': token, **session.description(), 'save_allowed': not self.public}
            if model_id == "TsenseI_v2":
                response = session.description()
                for suffix, path in (("control", session.recorder.path),
                                     ("frames", session.frame_logger.path if session.frame_logger else None)):
                    if path:
                        self.recordings[f"{token}-{suffix}"] = Path(path)
                response["download_id"] = f"{token}-control" if session.recorder.path else None
                return {"session": token, **response}
        if model_id in (*A30_MODEL_IDS, *A32_MODEL_IDS) and hasattr(session, "control"):
            return {"session": token, **session.description()}
        return {"session": token, "model_id": model_id, "pose": session.pipeline.session.pose,
                "direct_mode": session.direct_mode,
                "actuation": session.pipeline.stroke_mapper.description(),
                "collision_mode": session.pipeline.collision_mode,
                "recording": session.recorder is not None,
                "log_file": str(session.recorder.path) if session.recorder else None}

    def get(self, token):
        with self.lock:
            if token not in self.sessions:
                raise ValueError("会话已失效，请重新开始")
            return self.sessions[token]

    @staticmethod
    def _validate_camera_source(camera_source, input_source):
        if camera_source not in ('browser', 'leapmotion_camera', 'realsense_rgbd'):
            raise ValueError('未知相机来源')
        if camera_source == 'leapmotion_camera' and input_source != 'mediapipe':
            raise ValueError('Leap 相机图像仅用于 MediaPipe 输入')
        if camera_source == 'realsense_rgbd' and input_source != 'realsense_rgbd':
            raise ValueError('RealSense 深度图像仅用于 realsense_rgbd 输入')

    def calibration_action(self, data, action):
        session = self.get(data['session'])
        if not isinstance(session, CalibrationSession):
            raise ValueError('需要动态标定会话')
        if action == 'input':
            camera_source = data.get('camera_source', 'browser') if data.get('enabled') else 'browser'
            self._validate_camera_source(camera_source, data.get('source'))
            with self.lock, session.lock:
                result = session.set_input(data)
                session.camera_source = camera_source
                self.webxr.close(data['session'])
                self.wuji.close(data['session'])
                self.leapmotion.close(data['session'])
                self.realsense.close(data['session'])
                if not any(getattr(s, 'camera_source', None) == 'leapmotion_camera' for s in self.sessions.values()):
                    self.leap_camera.close(wait=False)
                return result
        if action == 'save' and self.public:
            raise ValueError('公共服务请使用导出 YAML，在本地应用标定')
        if action == 'reload':
            return session.reload()
        if action == 'touch':
            with session.lock:
                session._touch()
                return {'ok': True}
        return getattr(session, action)(data)

    def remove(self, token):
        with self.hardware_lock:
            with self.lock:
                session = self.sessions.pop(token, None)
                self.owners.pop(token, None)
                worker = self.metric_workers.pop(token, None)
                if worker:
                    worker.cancel()
                self.webxr.close(token)
                self.wuji.close(token)
                self.leapmotion.close(token)
                self.realsense.close(token)
                if not any(getattr(s, 'camera_source', None) == 'leapmotion_camera' for s in self.sessions.values()):
                    self.leap_camera.close(wait=False)
            try:
                if worker:
                    worker.close()
                if session:
                    session.close()
                    self.recordings_ready.update(k for k in self.recordings if k.startswith(token + "-"))
            finally:
                if self.hardware_owner == token:
                    self.hardware_owner = None

    def webxr_create(self, data):
        token = data["session"]
        transport = data.get("transport", "webxr")
        if transport not in ("webxr", "adb"):
            raise ValueError("Quest transport 必须为 webxr 或 adb")
        with self.lock:
            session = self.sessions.get(token)
            if session is None or session.closed:
                raise ValueError("会话已失效，请重新开始")
            if session.input_source != "webxr":
                raise ValueError("请先选择 Quest WebXR 输入并重新开始会话")
            session.last_seen = time.monotonic()
            previous = self.metric_workers.pop(token, None)
            if previous:
                previous.cancel()
            result = self.webxr.create(token, session.input_side)
            background = self._replace_metric_worker(token, session, self.webxr)
            connection = self.quest_connection
            if transport == "adb":
                result.update(entry_path="/q/adb", capture_path="/q/adb#code=123456")
                connection = self.quest_adb_connection
            return {**result, **connection, 'transport': transport,
                    'background_processing': background, 'combined_frame': background}

    def webxr_latest(self, data):
        token = data["session"]
        with self.lock:
            session = self.sessions.get(token)
            if session is None or session.closed:
                raise ValueError("会话已失效，请重新开始")
            background = self._metric_result(data, session)
            if background is not None:
                return background
            snapshot = self.webxr.latest(token)
            # The PC keeps its session alive while pairing. Quest publishes
            # never keep an abandoned control page alive.
            session.last_seen = time.monotonic()
            return snapshot

    def wuji_create(self, data):
        with self.lock:
            session = self.sessions.get(data["session"])
            if session is None or session.closed or session.input_source != "wuji_glove":
                raise ValueError("请先选择 Wuji 手套并重新开始会话")
            session.last_seen = time.monotonic()
            session._wuji_processed_revision = -1
            session._wuji_loss_at = -1.
            previous = self.metric_workers.pop(data['session'], None)
            if previous:
                previous.cancel()
            result = self.wuji.create(data['session'], session.input_side)
            background = self._replace_metric_worker(data['session'], session, self.wuji)
            return {**result, 'combined_frame': True, 'background_processing': background}

    def wuji_latest(self, data):
        session = self.get(data["session"])
        if session.input_source != "wuji_glove":
            raise ValueError("当前会话不是 Wuji 手套输入")
        background = self._metric_result(data, session)
        if background is not None:
            return background
        snapshot = self.wuji.latest(data["session"])
        session.last_seen = time.monotonic()
        return snapshot

    def wuji_frame(self, data):
        """Consume the trusted newest observation and solve before returning it.

        The browser never transports skeletons back to the server. Duplicate
        revisions do not repeat a solve or refresh a robot target's timestamp.
        """
        session = self.get(data["session"])
        if session.input_source != "wuji_glove":
            raise ValueError("当前会话不是 Wuji 手套输入")
        background = self._metric_result(data, session)
        if background is not None:
            return background
        with session.lock:
            snapshot = self.wuji_latest(data)
            snapshot["update"] = None
            now = time.perf_counter()
            if snapshot["hand"] is not None:
                if snapshot["revision"] <= getattr(session, '_wuji_processed_revision', -1):
                    return snapshot
            elif now - getattr(session, '_wuji_loss_at', -1.) < .1:
                return snapshot
            packet = {key: data[key] for key in ('session', 'sequence', 'timestamp_ms', 'capture_unix_ms')}
            packet.update(source="wuji_glove", wuji_revision=snapshot["revision"])
            snapshot["update"] = self.frame(packet)
            if snapshot["hand"] is not None:
                session._wuji_processed_revision = snapshot["revision"]
            else:
                session._wuji_loss_at = now
            snapshot["update"]["input_path"] = "server_wuji_frame"
            snapshot["update"]["source_age_at_solve_ms"] = snapshot["age_ms"]
            return snapshot

    def leap_camera_frame(self, data):
        after = data.get('after_revision', -1)
        if type(after) is not int or after < -1:
            raise ValueError('图像 revision 必须为 -1 或非负整数')
        with self.lock:
            session = self.sessions.get(data['session'])
            if session is None or session.closed:
                raise ValueError('相机会话已失效，请重新开始')
            if session.input_source != 'mediapipe' or getattr(session, 'camera_source', None) != 'leapmotion_camera':
                raise ValueError('请先创建 Leap Motion 2 相机 / MediaPipe 会话')
            session.last_seen = time.monotonic()
            # Start atomically with session validation; a stopped long-poll must
            # never reopen the device after remove() has closed its last owner.
            self.leap_camera.start()
        metadata, pixels = self.leap_camera.read_image(after, start=False)
        # A stop may race with a long-poll. Do not return images to a cancelled session.
        if session.closed or self.sessions.get(data['session']) is not session or session.camera_source != 'leapmotion_camera':
            raise ValueError('相机会话已停止')
        return encode_frame(metadata, pixels)

    def leapmotion_create(self, data):
        with self.lock:
            token = data['session']
            session = self.sessions.get(token)
            if session is None or session.closed or session.input_source != 'leapmotion':
                raise ValueError('请先选择 Leap Motion 2 并重新开始会话')
            session.last_seen = time.monotonic()
            session._leapmotion_processed_revision = -1
            session._leapmotion_loss_at = -1.
            previous = self.metric_workers.pop(token, None)
            if previous:
                previous.cancel()
            result = self.leapmotion.create(token, session.input_side)
            background = self._replace_metric_worker(token, session, self.leapmotion)
            return {**result, 'combined_frame': True, 'background_processing': background}

    def realsense_frame(self, data):
        token = data['session']
        with self.lock:
            session = self.sessions.get(token)
            if session is None or session.closed or session.input_source != 'realsense_rgbd':
                raise ValueError('请先创建 D455f 深度融合会话')
            session.last_seen = time.monotonic()
            self.realsense.create(token)
        payload = self.realsense.frame(token, data.get('after_revision', -1))
        if session.closed or self.sessions.get(token) is not session or session.input_source != 'realsense_rgbd':
            raise ValueError('深度相机会话已停止')
        return payload

    def _process_realsense(self, data, session):
        from online_control.online_motion_track.rgbd_input import prepare_packet
        token = data['session']
        epoch = getattr(session, 'input_epoch', None)
        data, snapshot, fused, diagnostics = prepare_packet(self.realsense, data, session.input_side)

        def commit(action):
            # No Application lock here: controller processing already owns its
            # session lock. remove/calibration acquire locks in the other order.
            def current():
                return (self.sessions.get(token) is session and not session.closed
                        and session.input_source == 'realsense_rgbd'
                        and getattr(session, 'input_epoch', None) == epoch)
            accepted = False
            def apply_current():
                nonlocal accepted
                if current():
                    action()
                    accepted = True
            if not current():
                return False
            if snapshot is None:
                # Only a no-hand/failed-depth packet can reach this branch.
                apply_current()
            else:
                try:
                    self.realsense.commit_if_current(token, snapshot, apply_current)
                except ValueError:
                    return False
            return accepted

        if hasattr(session, 'process_metric'):
            result = session.process_metric(data, commit)
        else:
            result = session.process(data)
            if not commit(lambda: None):
                result.update(tracking=False, display_discarded=True,
                              discard_reason='source_cancelled_or_expired')
        if result.get('display_discarded') or result.get('background_discarded'):
            diagnostics = dict(diagnostics, status='unavailable', reason='source_cancelled_or_expired')
            data['source_metadata']['depth_fusion'] = diagnostics
        result.update(source_metadata=data['source_metadata'], depth_fusion=diagnostics,
                      fused_world=fused['world'] if fused and result.get('tracking') else None,
                      depth_source='realsense_rgbd_fused' if diagnostics['status'] == 'fused' else 'none')
        if not result.get('tracking') and diagnostics['status'] == 'unavailable':
            result['message'] = '深度约束不可用，保持上一姿态：' + diagnostics.get('reason', '')
        return result

    def leapmotion_latest(self, data):
        session = self.get(data['session'])
        if session.input_source != 'leapmotion':
            raise ValueError('当前会话不是 Leap Motion 2 输入')
        background = self._metric_result(data, session)
        if background is not None:
            return background
        snapshot = self.leapmotion.latest(data['session'])
        session.last_seen = time.monotonic()
        return snapshot

    def leapmotion_frame(self, data):
        session = self.get(data['session'])
        if session.input_source != 'leapmotion':
            raise ValueError('当前会话不是 Leap Motion 2 输入')
        background = self._metric_result(data, session)
        if background is not None:
            return background
        # Serialize consumers separately: preview WebSession uses a plain Lock
        # internally, so holding session.lock across process() would deadlock.
        with self.leapmotion_frame_lock:
            snapshot = self.leapmotion_latest(data)
            snapshot['update'] = None
            now = time.perf_counter()
            if snapshot['hand'] is not None:
                if snapshot['revision'] <= session._leapmotion_processed_revision:
                    return snapshot
            elif now - session._leapmotion_loss_at < .1:
                return snapshot
            sequence = getattr(session, '_metric_sequence', -1) + 1
            session._metric_sequence = sequence
            packet = dict(session=data['session'], source='leapmotion', sequence=sequence,
                timestamp_ms=now * 1000, capture_unix_ms=time.time() * 1000,
                leapmotion_revision=snapshot['revision'])
            if 'input_epoch' in data:
                packet['input_epoch'] = data['input_epoch']
            snapshot['update'] = self.frame(packet)
            if snapshot['hand'] is not None:
                session._leapmotion_processed_revision = snapshot['revision']
            else:
                session._leapmotion_loss_at = now
            snapshot['update']['input_path'] = 'server_leapmotion_frame'
            snapshot['update']['source_age_at_solve_ms'] = snapshot['age_ms']
            if snapshot['update'].get('display_discarded') or snapshot['update'].get('background_discarded'):
                snapshot.update(hand=None, tracking=False, connected=False)
            return snapshot

    def frame(self, data):
        session = self.get(data["session"])
        source = data.get("source", "mediapipe")
        if source != session.input_source:
            raise ValueError("帧输入源与会话不一致，请重新开始会话")
        if source == 'realsense_rgbd':
            return self._process_realsense(data, session)
        if source in ('wuji_glove', 'webxr', 'leapmotion'):
            background = self._metric_result(data, session)
            if background is not None:
                # Older clients can read the completed result but cannot cause
                # another solve or re-submit a target already owned by the worker.
                return background.get('update') or dict(tracking=False, source=source,
                    message='等待后台最新骨架求解', background_processing=True)
        if source == "wuji_glove":
            snapshot = self.wuji.consume(data["session"], data.get("wuji_revision"))
            data = {**data, "hand": snapshot["hand"], "source_metadata": snapshot["source_metadata"],
                    "raw_wuji": snapshot["raw_wuji"], "raw_webxr": None, "camera_size": [0, 0]}
        if source == "webxr":
            snapshot = self.webxr.consume(data["session"], data.get("webxr_revision"))
            # Never trust a replayed/modified hand or age supplied by the PC.
            data = {**data, "hand": snapshot["hand"], "source": "webxr",
                    "source_metadata": snapshot["source_metadata"], "raw_webxr": snapshot["raw_webxr"],
                    "camera_size": [0, 0]}
        if source == 'leapmotion':
            snapshot = self.leapmotion.consume(data['session'], data.get('leapmotion_revision'))
            observed_at = time.perf_counter()
            data = {**data, 'hand': snapshot['hand'], 'source_metadata': snapshot['source_metadata'],
                    'raw_leapmotion': snapshot.get('raw_leapmotion'), 'raw_webxr': None,
                    'raw_wuji': None, 'camera_size': [0, 0]}
            def commit(action):
                accepted = False
                def apply_fresh():
                    nonlocal accepted
                    age = snapshot.get('age_ms')
                    if snapshot['hand'] is not None and (age is None or
                            age + (time.perf_counter() - observed_at) * 1000 >= self.leapmotion.stale_after_ms):
                        return
                    action()
                    accepted = True
                self.leapmotion.commit_if_current(data['session'], snapshot, apply_fresh)
                return accepted
            if hasattr(session, 'process_metric'):
                return session.process_metric(data, commit)
            # Calibration owns no hardware target, but a delayed preview must
            # still be marked obsolete before it reaches the browser.
            result = session.process(data)
            if not commit(lambda: None):
                result.update(tracking=False, display_discarded=True,
                              discard_reason='source_cancelled_or_expired')
            return result
        return session.process(data)

    def a30_command(self, data):
        session = self.get(data["session"])
        if session.model_id not in A30_MODEL_IDS or not hasattr(session, "control"):
            raise ValueError("A3.0 控制会话不可用")
        return session.command(data.get("action", "status"), data)

    def a32_command(self, data):
        token, action = data["session"], data.get("action", "status")
        with self.hardware_lock:
            if self.public:
                raise ValueError("A3.2 控制需在连接设备的本地服务运行")
            session = self.get(token)
            if session.model_id not in A32_MODEL_IDS or not hasattr(session, "control"):
                raise ValueError("A3.2 SDK 控制会话不可用")
            worker = self.metric_workers.get(token)
            if worker:
                worker.touch_client()
            if action in ("status", "tracking_start"):
                return session.command(action, data)
            if action == "connect":
                if self.hardware_owner not in (None, token):
                    raise ValueError("下位机已由另一个页面控制，请先断开")
                response = session.command(action, data)
                self.hardware_owner = token
                return response
            if self.hardware_owner != token:
                raise ValueError("请先在当前页面连接 A3.2")
            if action == "disconnect":
                try:
                    return session.command(action, data)
                finally:
                    self.hardware_owner = None
            return session.command(action, data)

    def v2_command(self, data):
        token = data["session"]
        action = data.get("action", "status")
        with self.hardware_lock:
            session = self.get(token)
            if session.model_id != "TsenseI_v2":
                raise ValueError("下位机控制仅支持 V2")
            if action == "status":
                return session.status()
            if action in ("tracking_start", "browser_rebase", "set_direct_mode"):
                return session.command(action, data)
            if self.public:
                raise ValueError("公共服务禁用串口与下位机操作，请在连接设备的电脑运行本地服务")
            if action == "ports":
                return {"ports": [{"port": d.port, "label": d.display_name}
                                  for d in session.control.scan_devices()]}
            if action == "connect":
                if self.hardware_owner not in (None, token):
                    raise ValueError("下位机已由另一个页面控制，请先在原页面断开")
                devices = {d.port: d for d in session.control.scan_devices()}
                port = data.get("port")
                if port not in devices:
                    raise ValueError("串口已不可用，请刷新串口列表")
                self.hardware_owner = token
                if not session.control.connect(devices[port]):
                    self.hardware_owner = None
                    raise ValueError(session.control.snapshot().get("last_error") or "串口连接失败")
                return session.status()
            if action in ("arm", "home", "cancel_home", "emergency_stop", "disconnect"):
                if self.hardware_owner != token:
                    raise ValueError("请先在当前页面连接串口")
                result = session.command(action, data)
                if action == "disconnect":
                    self.hardware_owner = None
                return result
            raise ValueError("未知控制操作")

    def expire(self):
        with self.lock:
            now = time.monotonic()
            expired = [k for k, s in self.sessions.items() if now - s.last_seen > (120 if isinstance(s, CalibrationSession) else 3 if s.model_id in ("TsenseI_v2", *A30_MODEL_IDS) or (s.model_id in A32_MODEL_IDS and hasattr(s, "control")) else 120)
                       or (self.public and now - s.created_at > 900)]
        for token in expired:
            self.remove(token)

    def close(self):
        for token in list(self.sessions):
            self.remove(token)
        self.leap_camera.close()
        self.realsense.close_all()


def client_disconnected(error):
    """Recognize peer shutdowns without hiding unrelated SSL or server failures."""
    return isinstance(error, (BrokenPipeError, ConnectionResetError, ConnectionAbortedError,
                              ssl.SSLEOFError, ssl.SSLZeroReturnError)) or (
        isinstance(error, ssl.SSLError)
        and getattr(error, "reason", None) == "UNEXPECTED_EOF_WHILE_READING")


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    disable_nagle_algorithm = True
    timeout = 15
    publish_paths = ("/api/webxr/publish", "/api/quest-adb/publish")

    def valid_host(self):
        try:
            host = urlsplit("http://" + self.headers.get("Host", "")).hostname
        except ValueError:
            host = None
        if host not in self.application.allowed_hosts:
            self.close_connection = True
            self.send_json({"error": "访问域名未配置"}, 403)
            return False
        return True

    def end_headers(self):
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "same-origin")
        self.send_header("Permissions-Policy", "camera=(self), serial=(self), xr-spatial-tracking=(self), microphone=()")
        super().end_headers()

    def __init__(self, *args, application, **kwargs):
        self.application = application
        super().__init__(*args, **kwargs)

    def handle_one_request(self):
        # A persistent connection can carry multiple responses. Track each one
        # separately so a failed write can never append a second HTTP response.
        self._response_started = False
        try:
            super().handle_one_request()
        except OSError as exc:
            self.close_connection = True
            if not client_disconnected(exc):
                raise

    def send_response(self, *args, **kwargs):
        self._response_started = True
        super().send_response(*args, **kwargs)

    def log_message(self, format, *args):
        if (len(args) >= 2 and str(args[1]) == "400"
                and getattr(self, "path", None) in self.publish_paths):
            # These high-rate input routes log the actual validation reason
            # below, with a per-peer limit, instead of a bare line per frame.
            return
        if len(args) < 2 or str(args[1]) not in ("200", "204"):
            super().log_message(format, *args)

    def send_json(self, value, status=200):
        payload = json.dumps(json_safe(value), ensure_ascii=False, allow_nan=False).encode("utf-8")
        try:
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)
        except OSError as exc:
            # Closing the preview can cancel a response already being written.
            # Do not attempt a second error response on the same closed socket.
            self.close_connection = True
            if not client_disconnected(exc):
                raise

    def do_GET(self):
        if not self.valid_host():
            return
        url = urlsplit(self.path)
        if url.path in ("/api/viewer/a30-usage", "/api/viewer/a30-file", "/api/viewer/logs", "/api/viewer/log-file"):
            if self.application.public or not ipaddress.ip_address(self.client_address[0]).is_loopback:
                self.send_json({"error": "本机日志仅允许本机查看"}, 403)
                return
            try:
                if url.path == "/api/viewer/a30-usage":
                    self.send_json(a30_catalog())
                    return
                query = parse_qs(url.query)
                if url.path == "/api/viewer/logs":
                    self.send_json(control_log_catalog(query.get("model", [""])[0]))
                    return
                name = query.get("name", [""])[0]
                path = a30_file(name) if url.path == "/api/viewer/a30-file" else control_log_file(query.get("model", [""])[0], name)
                if path is None:
                    self.send_json({"error": "日志文件不存在"}, 404)
                    return
            except (OSError, ValueError) as exc:
                self.send_json({"error": str(exc)}, 400)
                return
            self.send_response(200)
            self.send_header("Content-Type", "application/x-ndjson; charset=utf-8")
            self.send_header("Content-Length", str(path.stat().st_size))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            with path.open("rb") as stream:
                shutil.copyfileobj(stream, self.wfile)
            return
        if url.path == "/api/model":
            try:
                query = parse_qs(url.query)
                self.send_json(model_description(query.get("side", ["left"])[0],
                                                 query.get("model", ["TsenseI_v3_2"])[0]))
            except ValueError as exc:
                self.send_json({"error": str(exc)}, 400)
            return
        if url.path == "/api/health":
            self.send_json({"ok": True, "model": "TsenseI_v3_2",
                            "server_pid": os.getpid(),
                            "recording_allowed": not self.application.public,
                            "browser_serial": True, "browser_recording": True,
                            "input_sources": ["mediapipe", "webxr", "wuji_glove", "leapmotion", "realsense_rgbd"], "webxr_stale_after_ms": self.application.webxr.stale_after_ms,
                            "camera_sources": ["browser", "leapmotion_camera", "realsense_rgbd"], "leap_camera_target_fps": 120,
                            "wuji_transport": self.application.wuji.source,
                            "control_chain_revision": "a30-stream-20260924-v2",
                            "downlink_sha256": downlink_version(), "a30_binary_stream": True,
                            "a30_control_available": a30_control_available(),
                            "a32_control_available": a32_control_available() and not self.application.public,
                            "a32_collision_modes": {m: a32_collision_modes(m) for m in A32_MODEL_IDS},
                            "mink_available": self.application.mink_available,
                            "a30_collision_modes": a30_collision_modes(),
                            "collision_mode": self.application.collision_mode,
                            "models": [*A32_MODEL_IDS, *A30_MODEL_IDS] + (["TsenseI_v2"] if self.application.v2_available else []),
                            "hardware_allowed": not self.application.public and self.application.v2_available,
                            "public": self.application.public})
            return
        mappings = {
            "/shared/": ROOT / "online_control/shared",
            "/assets-v2/": ROOT / "assets/hands/TsenseI_hand",
            "/assets-v3-0-withid/": ROOT / "assets/hands/TsenseI_hand_v3_0_withid",
            "/assets-v3-0/": ROOT / "assets/hands/TsenseI_hand_v3_0",
            "/assets-v3-2-withid/": ROOT / "assets/hands/TsenseI_hand_v3_2_withid",
            "/assets/": ROOT / "assets/hands/TsenseI_hand_v3_2",
            "/vendor/three/": HERE / "node_modules/three",
            "/vendor/mediapipe/": HERE / "node_modules/@mediapipe/tasks-vision",
            "/": HERE / "static",
        }
        if url.path.startswith("/shared/") and not url.path.endswith(".js"):
            return self.send_json({"error":"文件不存在"},404)
        path = None
        if url.path in ('/calibration', '/calibration/'):
            path = HERE / 'static/calibration.html'
        elif url.path == "/data-viewer":
            path = ROOT / "tools/hand_control_log_viewer.html"
        elif url.path == "/api/recording":
            if self.application.public:
                self.send_json({"error": "公共服务不提供记录文件"}, 403)
                return
            record_id = parse_qs(url.query).get("id", [""])[0]
            path = self.application.recordings.get(record_id)
            if path and record_id not in self.application.recordings_ready:
                self.send_json({"error": "请结束会话并保存后再下载记录"}, 409)
                return
        elif url.path in ("/q", "/q/", "/q/adb", "/q/adb/"):
            path = HERE / "static/quest.html"
        elif url.path == "/hand_landmarker.task":
            path = ROOT / "hand_landmarker.task"
        else:
            for prefix, root in mappings.items():
                if url.path.startswith(prefix):
                    relative = unquote(url.path[len(prefix):]) or "index.html"
                    candidate = (root / relative).resolve()
                    if candidate.is_relative_to(root.resolve()):
                        path = candidate
                    break
        if path is None or not path.is_file():
            self.send_json({"error": "文件不存在"}, 404)
            return
        stat = path.stat()
        etag = f'"{stat.st_size:x}-{stat.st_mtime_ns:x}"'
        if self.headers.get("If-None-Match") == etag:
            self.send_response(304)
            self.send_header("ETag", etag)
            self.end_headers()
            return
        self.send_response(200)
        mime = {".js": "text/javascript", ".mjs": "text/javascript", ".wasm": "application/wasm"}.get(path.suffix)
        self.send_header("Content-Type", mime or mimetypes.guess_type(str(path))[0] or "application/octet-stream")
        self.send_header("Content-Length", str(stat.st_size))
        if url.path == "/api/recording":
            self.send_header("Content-Disposition", f'attachment; filename="{path.name}"')
        self.send_header("ETag", etag)
        self.send_header("Cache-Control", "no-cache")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        with path.open("rb") as stream:
            shutil.copyfileobj(stream, self.wfile)

    def do_POST(self):
        if not self.valid_host():
            return
        data = None
        try:
            origin_header = self.headers.get("Origin")
            if ((self.application.public and not origin_header)
                    or (origin_header and urlsplit(origin_header).netloc != self.headers.get("Host"))):
                self.close_connection = True
                self.send_json({"error": "只接受同源网页请求"}, 403)
                return
            length = int(self.headers.get("Content-Length", 0))
            if not 0 < length <= 128_000:
                self.close_connection = True
                self.send_json({"error": "请求大小无效"}, 413)
                return
            try:
                body = self.rfile.read(length)
            except OSError as exc:
                if not client_disconnected(exc):
                    raise
                self.close_connection = True
                return
            if len(body) != length:
                # An incomplete upload has no live peer to receive an error.
                self.close_connection = True
                return
            data = json.loads(body)
            if not isinstance(data, dict):
                raise ValueError("需要 JSON 对象")
            if self.path == "/api/viewer/log-parent":
                if (self.application.public or not ipaddress.ip_address(self.client_address[0]).is_loopback
                        or not origin_header or urlsplit(origin_header).netloc != self.headers.get("Host")):
                    self.send_json({"error": "日志目录仅允许本机同源页面修改"}, 403)
                    return
                parent = set_control_log_parent(data["parent"])
                response = {"parent": str(parent), "folders": [str(parent / model) for model in ("A30", "A32", "V2")]}
            elif self.path == "/api/session":
                owner = self.client_address[0]
                # Backend is loopback-only; Caddy replaces this header itself.
                if self.application.public and owner == "127.0.0.1":
                    owner = str(ipaddress.ip_address(self.headers.get("X-Real-IP", owner)))
                response = self.application.create(data, owner)
            elif self.path == "/api/frame":
                response = self.application.frame(data)
            elif self.path in ("/api/leap-camera/frame", "/api/realsense/frame"):
                payload = (self.application.realsense_frame(data) if self.path == "/api/realsense/frame"
                           else self.application.leap_camera_frame(data))
                self.send_response(200)
                self.send_header('Content-Type', 'application/octet-stream')
                self.send_header('Content-Length', str(len(payload)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.end_headers()
                self.wfile.write(payload)
                return
            elif self.path == "/api/feedback-pose":
                from online_control.online_motion_track.feedback import feedback_pose
                response = feedback_pose(data)
            elif self.path in tuple('/api/calibration/' + name for name in ('input', 'update', 'save', 'export', 'reload', 'touch')):
                response = self.application.calibration_action(data, self.path.rsplit('/', 1)[-1])
            elif self.path == "/api/webxr/create":
                response = self.application.webxr_create(data)
            elif self.path == "/api/wuji/create":
                response = self.application.wuji_create(data)
            elif self.path == "/api/wuji/latest":
                response = self.application.wuji_latest(data)
            elif self.path == "/api/wuji/frame":
                response = self.application.wuji_frame(data)
            elif self.path == "/api/leapmotion/create":
                response = self.application.leapmotion_create(data)
            elif self.path == "/api/leapmotion/latest":
                response = self.application.leapmotion_latest(data)
            elif self.path == "/api/leapmotion/frame":
                response = self.application.leapmotion_frame(data)
            elif self.path in ("/api/webxr/pair", "/api/quest-adb/pair"):
                response = self.application.webxr.pair(data.get("code"), self.client_address[0])
            elif self.path == "/api/webxr/code":
                self.application.get(data["session"])
                response = self.application.webxr.renew_code(data["session"])
            elif self.path == "/api/webxr/publish":
                response = self.application.webxr.publish(data["token"], data["message"])
            elif self.path == "/api/quest-adb/publish":
                response = self.application.webxr.publish_quest_adb(data["token"], data["message"])
            elif self.path == "/api/webxr/latest":
                response = self.application.webxr_latest(data)
            elif self.path == "/api/webxr/close":
                self.application.get(data["session"])
                self.application.webxr.close(data["session"])
                response = {"ok": True}
            elif self.path == "/api/a30/stream":
                session = self.application.get(data["session"])
                if session.model_id not in A30_MODEL_IDS:
                    raise ValueError("此会话不支持 A3 二进制下发")
                records = session.control.open_stream(data.get("epoch"), preview=data.get("preview") is True)
                return serve_records(self, records)
            elif self.path == "/api/a30/control":
                response = self.application.a30_command(data)
            elif self.path == "/api/a32/control":
                if not origin_header or not ipaddress.ip_address(self.client_address[0]).is_loopback:
                    self.send_json({"error": "SDK 控制接口仅接受本机同源页面"}, 403)
                    return
                response = self.application.a32_command(data)
            elif self.path == "/api/v2/control":
                if not origin_header or not ipaddress.ip_address(self.client_address[0]).is_loopback:
                    self.send_json({"error": "控制接口仅接受本机同源页面"}, 403)
                    return
                response = self.application.v2_command(data)
            elif self.path == "/api/stop":
                self.application.remove(data["session"])
                response = {"ok": True}
            else:
                self.send_json({"error": "接口不存在"}, 404)
                return
            self.send_json(response)
        except (ValueError, KeyError, TypeError, OverflowError) as exc:
            if getattr(self, "_response_started", False):
                self.close_connection = True
                raise
            if self.path in self.publish_paths:
                reason = str(exc)
                token = data.get("token") if isinstance(data, dict) else None
                if isinstance(token, str) and token:
                    reason = reason.replace(token, "[redacted]")
                self.server.log_publish_rejection(self.client_address[0], self.path, reason)
            self.send_json({"error": str(exc)}, 400)
        except Exception as exc:
            if getattr(self, "_response_started", False):
                self.close_connection = True
                raise
            # A failed logger/solver must be visible to the client, never keep
            # animating stale successful status after a broken request.
            self.send_json({"error": f"服务错误：{exc}"}, 500)


class LimitedHTTPServer(ThreadingHTTPServer):
    """Bound connection threads, including slow/static asset clients."""
    allow_reuse_address = os.name != "nt"
    warning_interval = 30.0

    def server_bind(self):
        if os.name == "nt":
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()

    def __init__(self, *args, **kwargs):
        self.slots = threading.BoundedSemaphore(64)
        self._warnings = {}
        self._warning_lock = threading.Lock()
        super().__init__(*args, **kwargs)

    def process_request(self, request, client_address):
        if not self.slots.acquire(blocking=False):
            if isinstance(request, ssl.SSLSocket):
                # No worker is available to negotiate TLS. Do not start a
                # blocking handshake in the listener just to write HTTP 503.
                self.shutdown_request(request)
                return
            try:
                request.sendall(b"HTTP/1.1 503 Service Unavailable\r\nContent-Length: 0\r\nConnection: close\r\n\r\n")
            finally:
                self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except BaseException:
            self.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.slots.release()

    def _warning_suffix(self, key):
        now = time.monotonic()
        with self._warning_lock:
            previous = self._warnings.get(key)
            if previous and now - previous[0] < self.warning_interval:
                self._warnings[key] = (previous[0], previous[1] + 1)
                return None
            suppressed = previous[1] if previous else 0
            # Bound bookkeeping for both certificate and input warnings.
            if previous is None and len(self._warnings) >= 256:
                oldest = min(self._warnings, key=lambda item: self._warnings[item][0])
                del self._warnings[oldest]
            self._warnings[key] = (now, 0)
        return f" ({suppressed} repeated warnings suppressed.)" if suppressed else ""

    def log_publish_rejection(self, peer, path, reason):
        repeated = self._warning_suffix(("publish", peer, path))
        if repeated is None:
            return
        # Do not log payloads or capabilities, or allow malformed input to
        # introduce terminal controls/newlines into the diagnostic.
        reason = "".join(char if char.isprintable() else " " for char in reason)[:240]
        print(f"[HTTP] {peer} POST {path} rejected (400): {reason}{repeated}",
              file=sys.stderr, flush=True)

    def handle_error(self, request, client_address):
        error = sys.exception()
        if client_disconnected(error):
            return
        if isinstance(error, ssl.SSLError) and getattr(error, "reason", None) in {
            "SSLV3_ALERT_CERTIFICATE_UNKNOWN", "TLSV1_ALERT_UNKNOWN_CA",
            "SSLV3_ALERT_BAD_CERTIFICATE", "SSLV3_ALERT_CERTIFICATE_EXPIRED",
        }:
            repeated = self._warning_suffix(("certificate", client_address[0]))
            if repeated is None:
                return
            print(f"[HTTPS] {client_address[0]} rejected the server certificate "
                  f"({error.reason}). Check the URL/IP and certificate validity, "
                  "then trust quest_webxr_ca.crt on the connecting computer or Quest "
                  "and restart its browser. USB alternative: "
                  f"bash migration/ubuntu/start-quest-adb.sh.{repeated}", file=sys.stderr, flush=True)
            return
        super().handle_error(request, client_address)


def make_server(port=8766, log_dir=None, *, host="127.0.0.1", tls_cert=None, tls_key=None, **options):
    if bool(tls_cert) != bool(tls_key):
        raise ValueError("--tls-cert 与 --tls-key 必须同时配置")
    context = None
    if tls_cert:
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.minimum_version = ssl.TLSVersion.TLSv1_2
        context.load_cert_chain(str(tls_cert), str(tls_key))
    app = Application(log_dir, **options)
    server = LimitedHTTPServer((host, port), partial(Handler, application=app))
    if context:
        # Negotiate in the bounded request worker after Handler sets its timeout;
        # an idle TCP client must not block the listener during TLS handshake.
        server.socket = context.wrap_socket(server.socket, server_side=True, do_handshake_on_connect=False)
    server.daemon_threads = True
    server.application = app
    app.quest_adb_connection = {
        'entry_url': '', 'adb_entry_url': '', 'adb_entry_path': '/q/adb',
        'connection_error': 'USB ADB 输入需要回环 HTTP 服务。请运行 bash migration/ubuntu/start-quest-adb.sh，'
                            '再打开 http://localhost:8766/?source=quest_adb。',
    }
    if not app.public and not context and host in ('127.0.0.1', 'localhost', '::1'):
        adb_url = f'http://localhost:{server.server_port}/q/adb'
        app.quest_adb_connection = {
            'entry_url': adb_url, 'adb_entry_url': adb_url, 'adb_entry_path': '/q/adb',
        }
    if not app.public:
        candidates = options.get('allowed_hosts', ())
        if host != '0.0.0.0':
            candidates = (host,) if host in app.allowed_hosts else ()
        quest_host = None
        for candidate in candidates:
            if candidate.lower() == 'localhost':
                continue
            try:
                address = ipaddress.ip_address(candidate)
                if address.is_loopback or address.is_unspecified:
                    continue
            except ValueError:
                pass  # Explicitly allowed DNS name.
            quest_host = candidate
            break
        if context and quest_host:
            authority = f'[{quest_host}]' if ':' in quest_host else quest_host
            app.quest_connection = {'entry_url': f'https://{authority}:{server.server_port}/q'}
        else:
            app.quest_connection = {
                'entry_url': '',
                'connection_error': '当前服务未配置 Quest 局域网 HTTPS 访问。请让电脑与头显连接同一局域网，'
                                    '再用普通 start-track.sh 启动工作台。127.0.0.1 是设备自身，不能用于头显连接电脑。',
            }
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--host", default="127.0.0.1", help="Bind address; use 0.0.0.0 for Quest LAN access")
    parser.add_argument("--tls-cert", type=Path, help="HTTPS certificate trusted by the Quest Browser")
    parser.add_argument("--tls-key", type=Path, help="Private key matching --tls-cert")
    parser.add_argument("--no-open", action="store_true")
    parser.add_argument("--wuji-source", choices=("sdk", "bridge"),
                        help="Wuji input transport (default: local sdk; bridge requires an explicit URL)")
    parser.add_argument("--wuji-address", help="Direct glove IPv4:port; otherwise discover locally")
    parser.add_argument("--wuji-serial", help="Glove serial number; also verifies identity when address is supplied")
    parser.add_argument("--wuji-url", help="Explicit legacy HTTP bridge URL; implies bridge unless source is specified")
    parser.add_argument("--log-dir", type=Path, default=HERE / "logs")
    parser.add_argument("--public", action="store_true", help="HTTPS proxy mode; no server recording or server USB")
    parser.add_argument("--allowed-host", action="append", default=[])
    parser.add_argument("--max-sessions", type=int, default=4)
    parser.add_argument("--collision-mode", choices=("off", "mink"), default="off")
    parser.add_argument("--check", action="store_true", help="Validate A3.0/A3.2 assets and left/right analytic solvers; exit")
    args = parser.parse_args()
    try:
        wuji_config = resolve_wuji_config(args.wuji_url, source=args.wuji_source,
                                         address=args.wuji_address, serial=args.wuji_serial)
    except ValueError as exc:
        parser.error(str(exc))
    if bool(args.tls_cert) != bool(args.tls_key):
        parser.error("--tls-cert 与 --tls-key 必须同时配置")
    if args.collision_mode == "mink" and not mink_available():
        parser.error("请先安装：python -m pip install -r online_control/online_motion_track/requirements-mink.txt")
    if not (HERE / "node_modules/@mediapipe/tasks-vision/vision_bundle.mjs").is_file():
        parser.error("请先执行 npm ci --prefix online_control/online_motion_track")
    if not (ROOT / "hand_landmarker.task").is_file():
        parser.error("项目根目录缺少 hand_landmarker.task")
    if not 1 <= args.max_sessions <= 32:
        parser.error("--max-sessions 必须在 1..32")
    if args.public and not args.allowed_host:
        parser.error("公网模式必须配置 --allowed-host，例如 hand.example.com")
    if args.check:
        for model_id, side in ((m, s) for m in (*A32_MODEL_IDS, *A30_MODEL_IDS) for s in ("left", "right")):
            model_description(side, model_id)
            checked = WebSession(side, recording=False, model_id=model_id,
                                 collision_mode=args.collision_mode)
            solver = checked.pipeline.session.retargeter
            if solver._analytic_kinematics is None:
                raise RuntimeError("解析雅可比初始化失败：" + str(solver._analytic_kinematics_error))
            result = solver.retarget(solver.fk.compute(), include_artifacts=False)
            if not result.success or len(result.joint_values) != 20:
                raise RuntimeError("Solver validation failed")
            if checked.pipeline.mink_guard:
                checked.pipeline.mink_guard.project(result.joint_values)
                print(side, checked.pipeline.mink_guard.diagnostics)
            checked.close()
        print("CHECK OK: A3.0/A3.2 assets, left/right analytic retarget; no camera/GPU server dependency", flush=True)
        return
    try:
        server = make_server(args.port, args.log_dir, public=args.public,
                             allowed_hosts=args.allowed_host, max_sessions=args.max_sessions,
                             collision_mode=args.collision_mode, host=args.host,
                             tls_cert=args.tls_cert, tls_key=args.tls_key,
                             **{'wuji_' + key: value for key, value in wuji_config.items()})
    except OSError as exc:
        parser.error(f"端口 {args.port} 无法使用，请通过 --port 指定其他端口：{exc}")
    scheme = "https" if args.tls_cert else "http"
    display_host = args.allowed_host[0] if args.tls_cert and args.allowed_host else (
        "127.0.0.1" if args.host == "0.0.0.0" else args.host)
    if not args.public and args.host in ("0.0.0.0", "127.0.0.1", "localhost"):
        # SDK control requires a loopback peer, even when LAN capture is enabled.
        display_host = "127.0.0.1"
    url = f"{scheme}://{display_host}:{server.server_port}"
    print(f"电脑手势映射工作台已就绪：{url}", flush=True)
    if not args.public and args.host == "0.0.0.0" and args.tls_cert and args.allowed_host:
        print(f"Quest 采集页：https://{args.allowed_host[0]}:{server.server_port}/q\n"
              "电脑 SDK 控制请使用上面的 127.0.0.1 地址。", flush=True)
    print("Ctrl+C 停止服务。", flush=True)
    stopped = threading.Event()

    def sweep():
        while not stopped.wait(.25):
            server.application.expire()
    threading.Thread(target=sweep, daemon=True).start()
    def shutdown(_signum, _frame):
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, shutdown)
    if not args.no_open:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stopped.set()
        server.server_close()
        server.application.close()


if __name__ == "__main__":
    main()
