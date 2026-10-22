"""Browser owns USB; this session owns retarget and planned/direct joint targets."""
import time
import threading
from online_control.online_motion_track.main import StaleFrameError, WebSession
from .control import A30Control


class A30WebSession(WebSession):
    def __init__(self, side="left", recording=False, log_dir=None, collision_mode="stop", *, model_id="TsenseI_v3_0",
                 input_source="mediapipe"):
        super().__init__(side, recording, log_dir, collision_mode, model_id=model_id, input_source=input_source)
        # Extend the base estimator's transaction through controller commit.
        # Rebase/mode/close must not slip between the two stages.
        self.lock = threading.RLock()
        try:
            self.control = A30Control(self.pipeline.session.profile, side, collision_mode, preview_only=True)
        except Exception:
            super().close(); raise

    def status(self):
        if self.closed: raise ValueError("会话已停止")
        self.last_seen = time.monotonic()
        return self.control.snapshot()

    def description(self):
        return {"model_id": self.model_id, "pose": self.control.snapshot()["ruckig_joint_angles_rad"],
                "input_source": self.input_source,
                "init_pose": self.pipeline.session.profile.init_pose(self.pipeline.session.hand_side),
                "joint_limits": self.control.limits,
                "direct_mode": self.control.direct_mode,
                "collision_mode": "off" if self.control.direct_mode else self.pipeline.collision_mode,
                "actuation": self.pipeline.stroke_mapper.description(), "hardware": self.status(),
                "recording": self.recorder is not None,
                "log_file": str(self.recorder.path) if self.recorder else None}

    def command(self, action, data):
        # Status only reads the independently locked controller. Waiting for
        # retarget/Mink here delays the stream clock and the first recovery
        # packet, despite the control thread still producing safe samples.
        if action == "status":
            return self.status()
        if action == "browser_rebase" and data.get("resume_from_commanded_pose") is True:
            # Automatic recovery touches only the controller. Its own lock
            # orders rebase against push; the latest target survives either
            # ordering. A slow estimator must not consume the recovery timeout.
            if self.closed: raise ValueError("会话已停止")
            if not isinstance(data.get("telemetry"), str):
                raise ValueError("A3.0 恢复轨迹需要一包 444 或 484 字节遥测")
            self.last_seen = time.monotonic()
            return self.control.rebase(data["telemetry"], data.get("pose"),
                                       resume_from_commanded_pose=True)
        with self.lock:
            return self._command_locked(action, data)

    def _command_locked(self, action, data):
        if self.closed: raise ValueError("会话已停止")
        if action == "status": return self.status()
        if action == "set_direct_mode":
            enabled = data.get("enabled")
            if not isinstance(enabled, bool):
                raise ValueError("enabled 必须为布尔值")
            with self.lock:
                self.control.set_direct_mode(enabled)
                self.direct_mode = enabled
                self.pipeline.collision_bypass = enabled
                self.pipeline.session.set_pose(self.control.pose)
                self.pipeline.session.retargeter.reset()
            return self.status()
        if action == "browser_rebase":
            if not isinstance(data.get("telemetry"), str):
                raise ValueError("A3.0 首次开启下发需接收一包 444 或 484 字节遥测")
            with self.lock:
                resume = data.get("resume_from_commanded_pose") is True
                state = self.control.rebase(data["telemetry"], data.get("pose"),
                                            resume_from_commanded_pose=resume)
                # Transport recovery changes the commanded trajectory origin,
                # not the observation or the gesture being estimated. Keep the
                # filters, solver warm start, interaction graph and target-side
                # collision state together. The control's separate sample guard
                # was rebased above and still checks the path from the last write.
                if not resume:
                    pose = state["ruckig_joint_angles_rad"]
                    self.pipeline.session.set_pose(pose)
                    self.pipeline.session.retargeter.reset()
                    guard = None if self.control.direct_mode else self.pipeline.mink_guard
                    if guard and hasattr(guard, "current"):
                        guard.current = dict(pose); guard.guard.rebase(pose)
                    elif guard and hasattr(guard, "q"):
                        q = guard._vector(pose)
                        if guard._feasible(q):
                            guard.q = q.copy()
                        else:
                            self.control.invalidate("反馈姿态不满足 Mink 最小间距")
                            raise ValueError("反馈姿态不满足 Mink 最小间距，请调整实机或选择碰撞停止模式")
                    self.filter.reset(); self.normalized_filter.reset()
            return self.status()
        if action == "tracking_start":
            with self.lock:
                self.last_sequence, self.last_timestamp = -1, -1.
                self.filter.reset(); self.normalized_filter.reset()
                self.control.hold_target()
            return self.status()
        raise ValueError("A3.0 串口/找零仅在访问网页的本机执行")

    def process(self, packet):
        with self.lock:
            return self._process_locked(packet)

    def process_metric(self, packet, commit):
        if not callable(commit):
            raise TypeError("metric processing requires a commit callback")
        with self.lock:
            return self._process_locked(packet, commit=commit)

    def _process_locked(self, packet, *, commit=None):
        try:
            result = super().process(packet)
            def apply_result():
                if result["tracking"]:
                    self.control.push(result["pose"], result["sequence"])
                elif packet.get("hand") is None or packet["hand"].get("label") != self.pipeline.expected_label:
                    self.control.hold_target()
                else:
                    self.control.invalidate(result["message"])
            if commit is not None:
                if not commit(apply_result):
                    return {**result, "tracking": False, "display_discarded": True,
                            "message": "输入已取消或过期，等待新数据",
                            "discard_reason": "source_cancelled_or_expired"}
            else:
                apply_result()
            state = self.status()
            result["collision_safe_target_joint_angles_rad"] = state["target_joint_angles_rad"]
            if not state["direct_mode"]:
                result.update(pose=state["ruckig_joint_angles_rad"], actuation=state["actuation"])
            result.update(hardware=state, direct_mode=state["direct_mode"])
            return result
        except StaleFrameError:
            raise  # Ignore obsolete input without erasing a newer safe target.
        except Exception:
            if commit is None:
                self.control.invalidate("输入或求解异常")
            else:
                commit(lambda: self.control.invalidate("输入或求解异常"))
            raise

    def close(self):
        with self.lock:
            self.control.close(); super().close()
