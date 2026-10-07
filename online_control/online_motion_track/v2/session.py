"""Per-browser V2 solver, desktop collision/Ruckig/H7 pipeline and native logs."""
from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import math
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
for directory in (ROOT, ROOT / "src", ROOT / "src/tracking", ROOT / "online_control/online_motion_track/pipeline_v2"):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

from control_pipeline import LightControlPipeline
from collision_guard import SelfCollisionGuard
from pipeline_geometry import prepare_topo_points, landmarks_ok_for_retarget, thumb_diagnostics
from session_logger import SessionJsonlLogger
from hand_profiles import load_hand_profile
from TsenseI_retargeting import TopoRetargeter
# Import the lightweight module directly: tracking.__init__ also imports
# desktop-only OpenCV/HandOccNet/WiLoR backends, unused by browser detection.
from wuji_mediapipe import _OneEuroLandmarkFilter, preprocess_wuji_world_points
from utils.retarget_runtime import (_build_topo_config, _prepare_keypoints_for_solver,
                                    _to_right_joint_names, _topo_solver_failed)
from utils.control_session_recorder import ControlSessionRecorder
from usb.offline import UsbController, DEFAULT_HOMING_IQ_SEEK


class V2WebSession:
    model_id = "TsenseI_v2"

    def __init__(self, side="left", recording=True, log_dir=None, collision_mode="avoid",
                 *, usb=None, control_factory=LightControlPipeline, input_source="mediapipe"):
        if side not in ("left", "right"):
            raise ValueError("手别必须为 left 或 right")
        if collision_mode not in ("avoid", "stop", "off"):
            raise ValueError("V2 避碰模式必须为 avoid、stop 或 off")
        if input_source not in ("mediapipe", "webxr", "wuji_glove", "leapmotion"):
            raise ValueError("关键点来源必须为 mediapipe、webxr、wuji_glove 或 leapmotion")
        self.input_source = input_source
        self.lock = threading.RLock()
        self.closed = False
        self.direct_mode = False
        self.side = side
        self.collision_mode = collision_mode
        self.last_seen = self.created_at = time.monotonic()
        self.last_sequence, self.last_timestamp = -1, -1.0
        self.profile = load_hand_profile(self.model_id)
        # An independent solver per browser. Reuse desktop config and exact
        # right-hand X reflection, without its process-global warm-start state.
        self.solver = TopoRetargeter(replace(_build_topo_config(), kinematics_backend="numpy"))
        self.filter = _OneEuroLandmarkFilter(min_cutoff=1.5, beta=.25, derivative_cutoff=1.)
        self.normalized_filter = _OneEuroLandmarkFilter(min_cutoff=1.5, beta=.25, derivative_cutoff=1.)
        self.guard = SelfCollisionGuard(strategy=collision_mode) if collision_mode != "off" else None
        sample_guard = (SelfCollisionGuard(strategy="stop", path_step_rad=.04, bisection_iterations=8)
                        if self.guard else None)
        self.recorder = ControlSessionRecorder()
        self.control = control_factory(hand_side=side, ruckig_enabled=True, allow_direct_downlink=True,
            usb=usb if usb is not None else UsbController(recorder=self.recorder),
            recorder=self.recorder, source=f"web_v2/{input_source}")
        self.control.set_sample_collision_guard(sample_guard, mode=collision_mode)
        self.frame_logger = None
        self.previous_safe = None
        self.browser_epoch = 0
        self.log_dir = Path(log_dir or ROOT / "logs/control_sessions")
        try:
            if recording:
                camera_source = {"webxr": "quest3-webxr", "wuji_glove": "wuji-glove", "leapmotion": "leap-motion-2"}.get(input_source, "browser-webcam")
                self.recorder.start(output_dir=self.log_dir, metadata={
                    "hand_side": side, "model_id": self.model_id, "source": f"web/{input_source}",
                    "input_source": input_source, "camera_source": camera_source, "self_collision_mode": collision_mode,
                    "control_order": f"{input_source + ' metric' if input_source in ('webxr', 'wuji_glove', 'leapmotion') else 'MediaPipe/Wuji'} -> Topo -> collision -> Ruckig -> sample collision -> UI/HW -> DownlinkGuard -> linkage -> H7",
                    "hardware_location": "Python service host", "kinematics_backend": "numpy",
                    "control_timing": self.control.snapshot()["control_timing"]})
                self.frame_logger = SessionJsonlLogger(self.log_dir / "light_frames",
                    session_metadata={"hand_side": side, "model_id": self.model_id,
                        "input_source": input_source, "camera_source": camera_source, "self_collision_mode": collision_mode,
                        "coordinate_path": f"{input_source + ' metric 21 points' if input_source in ('webxr', 'wuji_glove', 'leapmotion') else 'Wuji'} -> wrist -> one right X reflection -> shared left solver"})
            self.control.start()
        except Exception:
            self.control.close()
            self.recorder.stop(reason="initialization_failed")
            if self.frame_logger:
                self.frame_logger.close(reason="initialization_failed")
            raise

    def status(self):
        # Do not wait for camera-rate optimization: heartbeat and stop must be
        # available while retarget runs. The control object protects its state.
        if self.closed:
            raise ValueError("会话已停止，请重新开始")
        self.last_seen = time.monotonic()
        snapshot = self.control.snapshot()
        snapshot.update(recording=self.recorder.active, recording_error=self.recorder.error,
                        model_id=self.model_id, collision_mode=self.collision_mode,
                        browser_epoch=self.browser_epoch)
        snapshot.update(direct_mode=self.direct_mode, ruckig_enabled=not self.direct_mode)
        return snapshot

    def pose(self):
        return self.control.current_ui_pose() or self.profile.init_pose(self.side)

    def description(self):
        return {"pose": self.pose(), "model_id": self.model_id,
                "input_source": self.input_source,
                "init_pose": self.profile.init_pose(self.side),
                "joint_limits": self.profile.joint_limits(self.side),
                "collision_mode": self.collision_mode, "recording": self.recorder.active,
                "log_file": str(self.recorder.path) if self.recorder.path else None,
                "frame_log_file": str(self.frame_logger.path) if self.frame_logger else None,
                "hardware": self.status()}

    def command(self, action, data):
        # Serialize the preview interlock with rebase/arm, including callers
        # that bypass the browser's disabled button.
        with self.lock:
            return self._command(action, data)

    def _command(self, action, data):
        if self.closed:
            raise ValueError("会话已停止")
        self.last_seen = time.monotonic()
        if action == "set_direct_mode":
            enabled = data.get("enabled")
            if not isinstance(enabled, bool):
                raise ValueError("enabled 必须为布尔值")
            with self.lock:
                self.control.set_armed(False)
                self.direct_mode = enabled
                self.browser_epoch += 1
                self.control.set_ruckig_enabled(not enabled)
                self.control.set_tracking_active(True)
            return self.status()
        if action == "browser_rebase":
            values = data.get("pose")
            limits = self.profile.joint_limits(self.side)
            feedback = data.get("telemetry")
            if feedback is not None:
                import struct
                from usb.telemetry import parse_hand_telem_v2, parse_hand_telem_legacy
                from usb.offline import joint_cfg_from_telemetry_frame
                if not isinstance(feedback, str) or len(feedback) not in (1560, 1864):
                    raise ValueError("遥测 payload 长度无效")
                payload = bytes.fromhex(feedback)
                if len(payload) not in (780, 932):
                    raise ValueError("遥测 payload 长度无效")
                stride, offset = (32, 4) if len(payload) == 780 else (44, 24)
                offsets = [8 + i * stride + offset for i in range(14)]
                offsets += [8 + 14 * stride + i * 44 + 24 for i in range(7)]
                if any(not math.isfinite(struct.unpack_from("<f", payload, offset)[0]) for offset in offsets):
                    raise ValueError("遥测位置包含无效数值")
                parsed = (parse_hand_telem_v2(payload) if len(payload) == 780 else parse_hand_telem_legacy(payload))
                if parsed is None or parsed.get("hand_type", int(self.side == "right")) != int(self.side == "right"):
                    raise ValueError("遥测手型不匹配")
                measured = joint_cfg_from_telemetry_frame(parsed, hand_side=self.side, strict=True)
                if self.side == "right":
                    measured = _to_right_joint_names(measured)
                if any(not math.isfinite(q) for q in measured.values()):
                    raise ValueError("遥测关节角无效")
                # The existing inverse mapping yields UI coordinates. Keep the
                # fixed DOF and model limits, using feedback rather than zero.
                values = {name: min(hi, max(lo, measured[name])) for name, (lo, hi) in limits.items()}
                values[f"{self.side}_hand_pinky_rota_joint"] = 0.0
            if not isinstance(values, dict) or set(values) != set(limits):
                raise ValueError("必须提供当前模型完整的关节姿态")
            pose = {name: float(value) for name, value in values.items()}
            if any(not math.isfinite(q) or q < limits[name][0] - 1e-6 or q > limits[name][1] + 1e-6
                   for name, q in pose.items()):
                raise ValueError("浏览器基准姿态超出关节限位")
            if any(abs(q) > 1e-8 for name, q in pose.items() if name.endswith("_pinky_rota_joint")):
                raise ValueError("小指旋转自由度固定为零")
            with self.lock:
                self.control.rebase_browser_preview(pose)
                self.browser_epoch += 1
                self.previous_safe = pose
                if self.guard:
                    self.guard.rebase(pose)
        elif action == "tracking_start":
            if self.control.snapshot()["homing_active"]:
                raise ValueError("请先完成或停止找零")
            with self.lock:
                self.solver.reset()
                self.filter.reset()
                self.normalized_filter.reset()
                self.last_sequence, self.last_timestamp = -1, -1.0
                self.control.set_tracking_active(True)
        elif action == "arm":
            enabled = data.get("enabled")
            if not isinstance(enabled, bool):
                raise ValueError("enabled 必须为布尔值")
            if enabled and self.collision_mode == "off":
                raise ValueError("真机下发必须开启 V2 碰撞保护")
            if enabled and not self.control.set_armed(True):
                raise ValueError("请连接串口、结束找零、开启识别并等待有效目标后再开启下发")
            if not enabled:
                self.control.set_armed(False)
        elif action == "home":
            iq = float(data.get("iq_seek", DEFAULT_HOMING_IQ_SEEK))
            if not math.isfinite(iq) or not 0 <= iq <= 5000:
                raise ValueError("找零 Iq 必须为 0..5000 的有限数值")
            if not self.control.start_one_key_zero(iq):
                raise ValueError(self.control.snapshot().get("last_error") or "找零已在进行")
        elif action == "cancel_home":
            self.control.cancel_one_key_zero("网页停止找零")
        elif action == "emergency_stop":
            ok = self.control.emergency_stop()
            if self.control.is_connected and not ok:
                raise RuntimeError("急停指令发送失败：" + str(self.control.snapshot().get("last_error")))
        elif action == "disconnect":
            self.control.disconnect()
        else:
            raise ValueError("未知控制操作")
        return self.status()

    def process(self, packet):
        from online_control.online_motion_track.main import StaleFrameError
        try:
            return self._process(packet)
        except StaleFrameError:
            raise  # A late network reply must not invalidate a newer target.
        except Exception as exc:
            self.control.invalidate_target(f"网页输入或记录失败：{type(exc).__name__}: {exc}")
            raise

    def process_metric(self, packet, commit):
        from online_control.online_motion_track.main import StaleFrameError
        if not callable(commit):
            raise TypeError("metric processing requires a commit callback")
        try:
            return self._process(packet, commit=commit)
        except StaleFrameError:
            raise
        except Exception as exc:
            commit(lambda: self.control.invalidate_target(f"网页输入或记录失败：{type(exc).__name__}: {exc}"))
            raise

    def _process(self, packet, *, commit=None):
        from online_control.online_motion_track.main import points, StaleFrameError
        from online_control.online_motion_track.pipeline_v3_2.recording import json_safe
        with self.lock:
            if self.closed:
                raise ValueError("会话已停止")
            metric_discarded = False
            def apply_control(action):
                nonlocal metric_discarded
                if commit is None:
                    action()
                    return True
                accepted = commit(action)
                metric_discarded = metric_discarded or not accepted
                return accepted

            sequence, timestamp = int(packet["sequence"]), float(packet["timestamp_ms"])
            if not math.isfinite(timestamp):
                raise ValueError("帧时间戳必须为有限值")
            if timestamp <= self.last_timestamp or sequence <= self.last_sequence:
                raise StaleFrameError("丢弃过期或重复帧")
            input_source = packet.get("source", self.input_source)
            if input_source not in ("mediapipe", "webxr", "wuji_glove", "leapmotion"):
                raise ValueError("关键点来源必须为 mediapipe、webxr、wuji_glove 或 leapmotion")
            if input_source != self.input_source:
                self.filter.reset(); self.normalized_filter.reset(); self.solver.reset()
                self.input_source = input_source
                self.control.source = f"web_v2/{input_source}"
            depth_source = f"{input_source}_metric" if input_source != "mediapipe" else "mediapipe_world"
            hand = packet.get("hand")
            raw = normalized = world = filtered_normalized = source = topo = None
            label = None
            match = False
            try:
                if hand is not None:
                    raw = points(hand["world"])
                    normalized = points(hand["normalized"]) if input_source == "mediapipe" else None
                    label = hand.get("label")
                    if label not in ("Left", "Right"):
                        raise ValueError("检测手别无效")
                    match = label == self.side.title()
                    if match:
                        if timestamp - self.last_timestamp > 300:
                            self.filter.reset(); self.normalized_filter.reset()
                        world = (self.filter.filter(raw, timestamp / 1000)
                                 if input_source == "mediapipe" else raw.copy())
                        filtered_normalized = (self.normalized_filter.filter(normalized, timestamp / 1000)
                                               if normalized is not None else None)
                        # Match Qt's remote-input path: preserve measured WebXR
                        # bone lengths. Wuji scaling/correction is camera-only.
                        source = (world - world[0] if input_source in ("webxr", "wuji_glove", "leapmotion")
                                  else preprocess_wuji_world_points(world))
                if not match:
                    self.filter.reset(); self.normalized_filter.reset()
            except (ValueError, TypeError, KeyError):
                apply_control(lambda: self.control.invalidate_target("网页关键点数据无效"))
                raise
            if self.recorder.active and input_source in ("webxr", "wuji_glove", "leapmotion"):
                self.recorder.record_stage(f"{input_source}_raw", {}, frame_id=sequence, hand_side=self.side,
                    input_source=input_source, timestamp_ms=timestamp, capture_unix_ms=packet.get("capture_unix_ms"),
                    raw_webxr=packet.get("raw_webxr"), raw_wuji=packet.get("raw_wuji"), raw_leapmotion=packet.get("raw_leapmotion"), source_metadata=packet.get("source_metadata"),
                    world_landmarks=[raw.tolist()] if raw is not None else [], handedness=label)
            elif self.recorder.active:
                # Reuse the desktop raw-event API; the browser already detected
                # the hand, so no server-side MediaPipe detector is instantiated.
                p = lambda values: [SimpleNamespace(x=x, y=y, z=z) for x, y, z in values]
                detection = packet.get("raw_detection")
                if detection is not None:
                    try:
                        worlds, norms, labels = detection["world"], detection["normalized"], detection["labels"]
                        if not len(worlds) == len(norms) == len(labels) or len(worlds) > 2:
                            raise ValueError("原始检测最多包含两只手，且字段数量须一致")
                        result = SimpleNamespace(hand_landmarks=[p(points(v)) for v in norms],
                            hand_world_landmarks=[p(points(v)) for v in worlds],
                            handedness=[[SimpleNamespace(category_name=v.get("label"), score=v.get("score"))] for v in labels])
                    except (TypeError, KeyError, ValueError):
                        apply_control(lambda: self.control.invalidate_target("原始检测记录无效"))
                        raise
                else:
                    result = SimpleNamespace(hand_landmarks=[p(normalized)] if hand else [],
                        hand_world_landmarks=[p(raw)] if hand else [],
                        handedness=[[SimpleNamespace(category_name=label, score=hand.get("score"))]] if hand else [])
                self.recorder.record_mediapipe_raw(result, frame_id=sequence,
                    timestamp_ms=int(timestamp), infer_ms=float(packet.get("inference_ms", 0)))
            quality = bool(match and landmarks_ok_for_retarget(source))
            raw_pose = None
            solve_ms = 0.0
            error = None
            collision = {"enabled": self.guard is not None, "strategy": self.collision_mode}
            decision = {"allow_retarget": False}
            if hand is not None and not match:
                apply_control(lambda: self.control.invalidate_target(f"手型不匹配：等待 {self.side.title()}，当前 {label}"))
            else:
                def observe():
                    nonlocal decision
                    decision = self.control.observe_tracking_frame(quality,
                        reason="hand_missing" if hand is None else "invalid_landmarks")
                apply_control(observe)
            started = time.perf_counter()
            if decision["allow_retarget"]:
                try:
                    topo = prepare_topo_points(source, self.side)
                    if input_source in ("webxr", "wuji_glove", "leapmotion"):
                        self.recorder.record_stage(input_source, {}, frame_id=sequence, hand_side=self.side,
                            landmarks=topo.tolist(), depth_source=depth_source, timestamp_ms=int(timestamp))
                    else:
                        self.recorder.record_mediapipe(topo, frame_id=sequence,
                            hand_side=self.side, depth_source=depth_source, timestamp_ms=int(timestamp))
                    solved = self.solver.retarget(_prepare_keypoints_for_solver(topo, self.side), include_artifacts=False)
                    solve_ms = (time.perf_counter() - started) * 1000
                    if _topo_solver_failed(solved.diagnostics):
                        self.solver.reset()
                        raise RuntimeError("TopoRetarget 求解失败")
                    raw_pose = {**solved.joint_values, "left_hand_pinky_rota_joint": 0.0}
                    if self.side == "right":
                        raw_pose = _to_right_joint_names(raw_pose)
                    self.recorder.record_retarget(raw_pose, frame_id=sequence, hand_side=self.side, solve_ms=solve_ms)
                    safe = dict(raw_pose)
                    if self.guard:
                        current = self.control.current_ui_pose()
                        if decision.get("reset_planner"):
                            self.guard.rebase(current)
                        safe, collision = self.guard.project(raw_pose, current_values=current)
                    # Keep IK's raw history, as in the current Qt V2 code. Ruckig
                    # and collision states must not accumulate into IK latency.
                    def push():
                        if self.control.push_target(safe, tracking_generation=decision["generation"], frame_id=sequence):
                            self.previous_safe = safe
                            self.recorder.record_stage("control_target", safe, frame_id=sequence,
                                hand_side=self.side, self_collision=collision)
                    apply_control(push)
                except Exception as exc:
                    error = f"{type(exc).__name__}: {exc}"
                    apply_control(lambda: self.control.observe_tracking_frame(False, reason="retarget_or_planning_error"))
            if raw_pose is None:
                self.recorder.record_stage("retarget_skipped", {}, frame_id=sequence,
                    hand_side=self.side, reason=error or "tracking_gate")
            snapshot = self.status()
            if self.recorder.error:
                apply_control(lambda: self.control.invalidate_target("记录失败：" + self.recorder.error))
                raise RuntimeError(self.recorder.error)
            pose = (self.previous_safe or self.profile.init_pose(self.side)) if self.direct_mode else self.pose()
            if self.frame_logger:
                self.frame_logger.record_frame({
                    "frame_id": sequence, "hand_side": self.side,
                    "input_source": input_source, "source_metadata": packet.get("source_metadata"),
                    "raw_webxr": packet.get("raw_webxr"),
                    "raw_leapmotion": packet.get("raw_leapmotion"),
                    "camera_size": packet.get("camera_size", [640,480]) if input_source == "mediapipe" else None,
                    "detected": hand is not None, "held_last_valid": False, "used_fallback_hand": False,
                    "detected_label": label, "expected_label": self.side.title(), "hand_selection_match": match,
                    "downstream_blocked_reason": snapshot.get("target_block_reason"),
                    "raw_normalized_points_21": normalized, "smoothed_normalized_points_21": filtered_normalized,
                    "raw_world_points_21_m": raw, "smoothed_world_points_21_m": world,
                    "depth_source": depth_source if hand else None,
                    "source_points_21_m": source,
                    "wuji_points_21_m": source if input_source == "mediapipe" else None,
                    "topo_input_points_21_m": topo,
                    "retarget_ran": raw_pose is not None, "retarget_quality_ok": quality,
                    "tracking_transition": snapshot["tracking_transition"], "retarget_joint_angles_rad": raw_pose,
                    "collision_safe_target_joint_angles_rad": self.previous_safe,
                    "ruckig_joint_angles_rad": snapshot["ruckig_joint_angles_rad"],
                    "hardware_joint_angles_rad": snapshot["hardware_joint_angles_rad"],
                    "linear_strokes_mm": snapshot["linear_strokes_mm"], "rotary_positions_deg": snapshot["rotary_positions_deg"],
                    "joint_angles_rad": pose, "retarget_ms": solve_ms, "retarget_error": error,
                    "thumb": thumb_diagnostics(source, topo, raw_pose), "self_collision": collision,
                    "downlink": {k: snapshot[k] for k in ("connected", "armed", "ruckig_enabled", "last_send_ok",
                        "sample_ticks", "send_ticks", "send_drops", "last_error", "homing_active", "homing_status")}})
            self.last_sequence, self.last_timestamp = sequence, timestamp
            self.last_seen = time.monotonic()
            tracking = not metric_discarded and raw_pose is not None and error is None and snapshot["target_ready"]
            return json_safe({"sequence": sequence, "tracking": tracking, "pose": pose, "points": topo,
                **({"display_discarded": True, "discard_reason": "source_cancelled_or_expired"} if metric_discarded else {}),
                "source": input_source, "depth_source": depth_source if hand else "none",
                "source_metadata": packet.get("source_metadata"),
                "message": ("输入已取消或过期，等待新数据" if metric_discarded else None) or error or snapshot.get("target_block_reason") or (
                    ("正在跟随 · 直接映射" if self.direct_mode else "正在跟随 · V2 碰撞与 Ruckig") if tracking else "等待有效手势 / 找零完成"),
                "solve_ms": (time.perf_counter() - started) * 1000,
                "retarget_joint_angles_rad": raw_pose,
                "collision_safe_target_joint_angles_rad": self.previous_safe if raw_pose is not None else None,
                "collision": {"mode": self.collision_mode, **collision}, "hardware": snapshot,
                "direct_mode": self.direct_mode})

    def close(self):
        # Interlock first, without waiting for a camera optimization to finish.
        if self.closed:
            return
        self.closed = True
        self.control.set_tracking_active(False)
        if self.control.snapshot()["homing_active"]:
            self.control.emergency_stop()
        self.control.close()
        with self.lock:
            self.recorder.stop(reason="web_session_stopped")
            if self.frame_logger:
                self.frame_logger.close(reason="web_session_stopped")
            if self.recorder.error:
                raise RuntimeError("记录未完整保存：" + self.recorder.error)
