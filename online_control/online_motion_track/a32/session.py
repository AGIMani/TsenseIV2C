"""A3.2 camera session with separately clocked native SDK downlink."""
import threading
import time

from online_control.online_motion_track.main import StaleFrameError, WebSession
from utils.control_session_recorder import control_log_directory
from .control import A32Control


class A32WebSession(WebSession):
    def __init__(self, side="left", recording=False, log_dir=None, collision_mode="off", *, model_id="TsenseI_v3_2",
                 input_source="mediapipe"):
        angle_log_dir = log_dir if log_dir is not None else control_log_directory({"model_id": model_id})
        super().__init__(side, recording, angle_log_dir, collision_mode, model_id=model_id, input_source=input_source)
        self.lock = threading.RLock()
        self.solver = None
        try:
            self.control = A32Control(self.pipeline.session.profile, side, collision_mode, recording_dir=log_dir)
            # Inline execution remains available for deterministic, non-running
            # controls and legacy server-side angle recording. The browser's
            # normal control recorder remains active in the parent process.
            # Isolate every live source from the 500 Hz control interpreter.
            if not recording and self.control.thread.is_alive():
                from .solver_process import SolverProcess
                self.solver = SolverProcess(side=side, collision_mode=collision_mode,
                    model_id=model_id, input_source=input_source)
        except Exception:
            if hasattr(self, 'control'):
                self.control.close()
            super().close()
            raise

    def description(self):
        return {"model_id": self.model_id, "pose": self.pipeline.session.pose,
                "input_source": self.input_source,
                "init_pose": self.pipeline.session.profile.init_pose(self.pipeline.session.hand_side),
                "joint_limits": self.control.device.joint_limits(),
                "collision_mode": "off" if self.control.direct_mode else self.pipeline.collision_mode,
                "direct_mode": self.control.direct_mode,
                "actuation": self.pipeline.stroke_mapper.description(), "hardware": self.status(),
                "recording": self.recorder is not None,
                "log_file": str(self.recorder.path) if self.recorder else None}

    def status(self, after_tick=None, *, touch_client=True):
        if self.closed:
            raise ValueError("会话已停止")
        if touch_client:
            self.last_seen = time.monotonic()
            self.control.touch()
        return self.control.snapshot(after_tick=after_tick, refresh_device=touch_client)

    def command(self, action, data):
        # Keep browser liveness independent of an expensive retarget solve.
        if action == "status":
            return self.status(data.get("after_tick"))
        with self.lock:
            if self.closed:
                raise ValueError("会话已停止")
            device = self.control.device
            self.control.touch()
            if action == "connect" and not device.is_connected:
                self.control.start_recording(transport=data.get("transport", "usb"),
                    input_source=self.input_source,
                    camera_source={"webxr": "quest3-webxr", "wuji_glove": "wuji-glove", "leapmotion": "leap-motion-2"}.get(self.input_source, "browser-webcam"))
            self.control.record_action("requested", request_action=action,
                                       request={k: v for k, v in data.items() if k != "session"})
            try:
                self._command(action, data, device)
            except Exception as exc:
                self.control._record("record_error", exc, action=action)
                if action == "connect" and not device.is_connected:
                    self.control._record("stop", reason="connect_failed")
                raise
            self.control.record_action(action)
            if action == "disconnect":
                self.control._record("stop", reason="disconnected")
            return self.status(data.get("after_tick"))

    def _command(self, action, data, device):
        """Execute with the session lock held; command() records its outcome."""
        if action == "set_direct_mode":
            self.set_direct_mode(data.get("enabled"))
        elif action == "connect":
            if not device.is_connected:
                self.control.reset_connection()
            transport = data.get("transport", "usb")
            if transport == "usb":
                device.connect_usb(data.get("port", "auto"))
            elif transport == "eth":
                device.connect_eth(data.get("host", ""), data.get("port", 5001))
            else:
                raise ValueError("A3.2 transport 必须为 usb 或 eth")
            self.control.last_error = ""
        elif action == "disconnect":
            try:
                self.control.stop()
            finally:
                device.disconnect()
                self.control.reset_connection()
        elif action == "arm":
            self.control.arm(data.get("enabled"), prepare_seed=self._prepare_seed)
        elif action in ("emergency_stop", "cancel_home", "disable"):
            self.control.stop()
        elif action == "home":
            self.control.stop()
            device.home(slot=data.get("slot"), iq_counts=data.get("iq_counts", 80))
        elif action == "reset_home":
            self.control.stop()
            device.reset_home(slot=data.get("slot"))
        elif action == "tracking_start":
            if self.solver:
                self.solver.call('reset')
            self.control.start_tracking()
            self.last_sequence, self.last_timestamp = -1, -1.
            self.filter.reset()
            self.normalized_filter.reset()
        else:
            raise ValueError("未知 A3.2 操作")

    def set_direct_mode(self, enabled):
        if not isinstance(enabled, bool):
            raise ValueError("enabled 必须为布尔值")
        with self.lock, self.control.lock:
            if self.closed:
                raise ValueError("会话已停止")
            if self.control.armed or self.control.stop_pending:
                raise ValueError("请先关闭下发，再切换直接控制模式")
            if self.control.direct_mode == enabled:
                return
            if self.solver:
                self.solver.call('set_direct_mode', enabled)
            self.pipeline.collision_bypass = enabled
            self.control.set_direct_mode(enabled)
            self.direct_mode = enabled
            self.pipeline.session.retargeter.reset()
            self.filter.reset()
            self.normalized_filter.reset()

    def _prepare_seed(self, pose):
        """Rebase the estimator and target guard before enabling, as on A3.0."""
        guard = None if self.control.direct_mode else self.pipeline.mink_guard
        if guard and hasattr(guard, "q"):
            q = guard._vector(pose)
            if not guard._feasible(q):
                raise ValueError("反馈姿态不满足 Mink 最小间距，请调整实机或选择碰撞停止模式")
            guard.q = q.copy()
        elif guard and hasattr(guard, "current"):
            guard.current = dict(pose)
            guard.guard.rebase(pose)
        self.pipeline.session.set_pose(pose)
        self.pipeline.session.retargeter.reset()
        self.filter.reset()
        self.normalized_filter.reset()
        if self.solver:
            self.solver.call('seed', pose)

    def _record_frame(self, packet, result):
        if not self.control.recorder.writer.active:
            return
        hand = packet.get("hand") or {}
        raw = packet.get("raw_detection")
        raw = raw if isinstance(raw, dict) else {}
        labels = raw.get("labels") or ([hand] if hand else [])
        details = {"frame_id": result["sequence"], "timestamp_ms": packet["timestamp_ms"],
                   "capture_unix_ms": packet["capture_unix_ms"],
                   "tracking_session_id": packet.get("session"), "input_source": self.input_source,
                   "direct_mode": self.control.direct_mode}
        if self.input_source in ("webxr", "wuji_glove", "leapmotion"):
            self.control.record_event(f"{self.input_source}_raw", **details, raw_webxr=packet.get("raw_webxr"), raw_wuji=packet.get("raw_wuji"), raw_leapmotion=packet.get("raw_leapmotion"),
                source_metadata=packet.get("source_metadata"),
                world_landmarks=[hand["world"]] if hand else [],
                handedness=[hand.get("label")] if hand else [])
        else:
            self.control.record_event("mediapipe_raw", **details,
                infer_ms=packet.get("inference_ms"),
                world_landmarks=raw.get("world", [hand["world"]] if hand else []),
                normalized_landmarks=raw.get("normalized", [hand["normalized"]] if hand else []),
                handedness=[[{"label": item.get("label"), "score": item.get("score")}] for item in labels],
                source_scope="all_detected_hands" if raw else "selected_hand_only")
        if result.get("points") is not None:
            self.control.record_event(self.input_source, **details, landmarks=result["points"],
                                      depth_source=result.get("depth_source", "mediapipe_world"))
        retarget = result.get("retarget_joint_angles_rad")
        if retarget:
            self.control.record_event("retarget", **details, joints=retarget,
                                      solve_ms=result.get("solve_ms"), tracking=bool(result["tracking"]),
                                      control_accepted=bool(result["tracking"]), message=result.get("message", ""))
        else:
            self.control.record_event("retarget_skipped", **details,
                reason="no_hand" if not hand else "solve_rejected", message=result.get("message", ""))
        if result["tracking"]:
            self.control.record_event("control_target", **details, joints=dict(self.control.target),
                                      self_collision=result.get("collision"))
        elif retarget:
            self.control.record_event("control_target_skipped", **details, reason="control_rejected",
                                      message=result.get("message", ""), self_collision=result.get("collision"))

    def process(self, packet):
        return self._process(packet, touch_client=True)

    def process_metric(self, packet, commit):
        if not callable(commit):
            raise TypeError("metric processing requires a commit callback")
        return self._process(packet, touch_client=True, commit=commit)

    def reset_metric_source(self):
        """Reset estimation after a trusted relay stream change, without motion."""
        with self.lock:
            if self.closed:
                raise ValueError("会话已停止，请重新开始")
            if self.solver:
                self.solver.call('reset')
            self.filter.reset()
            self.normalized_filter.reset()
            self.pipeline.session.retargeter.reset()
            self.last_sequence, self.last_timestamp = -1, -1.

    def process_background(self, packet, commit):
        """Solve without renewing the PC lease; atomically guard target changes.

        commit(action) must revalidate the caller's epoch, lease and original
        observation age under its short commit lock, then execute action or
        return False. No source lock is held during the expensive solve.
        """
        if not callable(commit):
            raise TypeError("background processing requires a commit callback")
        return self._process(packet, touch_client=False, commit=commit)

    def _process(self, packet, *, touch_client, commit=None):
        with self.lock:
            if self.closed:
                raise ValueError("会话已停止，请重新开始")
            began = time.perf_counter()
            stage = "骨架求解"
            try:
                if self.solver:
                    result = self.solver.process(packet)
                elif touch_client:
                    result = super().process(packet)
                else:
                    result = super().process(packet, touch_client=False)
                result["direct_mode"] = self.control.direct_mode
                solved = time.perf_counter()
                stage = "更新控制目标"
                def apply_result():
                    if result["tracking"]:
                        self.control.push(result["pose"], result["sequence"])
                    elif packet.get("hand") is None or packet["hand"].get("label") != self.pipeline.expected_label:
                        self.control.hold()
                    else:
                        self.control.reject_input(result["message"])
                if commit is not None:
                    if not commit(apply_result):
                        return {**result, "tracking": False, "background_discarded": True,
                                "message": "输入已取消或过期，等待新数据",
                                "discard_reason": "source_cancelled_or_expired"}
                else:
                    apply_result()
                committed = time.perf_counter()
                stage = "记录骨架"
                self._record_frame(packet, result)
                recorded = time.perf_counter()
                stage = "读取 SDK 状态"
                state = self.status(touch_client=touch_client)
                result["collision_safe_target_joint_angles_rad"] = state["target_joint_angles_rad"]
                result["command_joint_angles_rad"] = state["command_joint_angles_rad"]
                if state["armed"]:
                    result["pose"] = state["command_joint_angles_rad"]
                result["hardware"] = state
                ended = time.perf_counter()
                result["server_session_timing"] = {
                    "solve_dispatch_ms": (solved-began)*1000,
                    "control_commit_ms": (committed-solved)*1000,
                    "frame_record_ms": (recorded-committed)*1000,
                    "snapshot_ms": (ended-recorded)*1000,
                    "total_ms": (ended-began)*1000,
                }
                return result
            except StaleFrameError as exc:
                self.control.record_event("retarget_skipped", frame_id=packet.get("sequence"),
                                          reason="stale_frame", message=str(exc))
                raise
            except Exception as exc:
                self.control._record("record_error", exc, action="tracking_input",
                                     frame_id=packet.get("sequence"), stage=stage,
                                     exception_type=type(exc).__name__)
                reason = f"{stage}失败：{exc}；已停止下发"
                try:
                    if commit is None:
                        self.control.reject_input(reason)
                    else:
                        # A retired solve must not stop a newer source epoch.
                        commit(lambda: self.control.reject_input(reason))
                except Exception as stop_error:
                    self.control.last_error = f"{reason}；停止命令失败：{stop_error}"
                raise

    def close(self):
        with self.lock:
            try:
                self.control.close()
            finally:
                try:
                    if self.solver:
                        self.solver.close()
                finally:
                    super().close()
