"""A3.2 motion editor using the native SDK in the server process.

This session owns its model, motion file and trajectory.  It never uses the
A3.0 browser serial encoder or launches an SDK executable.
"""
from __future__ import annotations

import json
import copy
import io
import math
from functools import wraps
from pathlib import Path
import threading
import time
import uuid
import zipfile

from ruckig import InputParameter, OutputParameter, Result, Ruckig

from a32_driver import A32Device
from a32_kinematics.joint_coordinates import (
    COORDINATE_VERSION, SWING_AXIS_ALIGNMENT, POSITIVE_COORDINATE_VERSION,
    POSITIVE_SWING_AXIS_ALIGNMENT, requested_limits, migrate_legacy_pose, migrate_positive_pose,
)
from a32_recording import A32SessionRecorder
from hand_profiles import load_hand_profile
from utils.control_clock import precise_control_timer
from utils.motion_limits import configure_ruckig


MODEL_IDS = ("TsenseI_v3_2", "TsenseI_v3_2_withid")


def synchronized(method):
    @wraps(method)
    def locked(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)
    return locked


class A32Session:
    def __init__(self, data_dir, side="left", *, model_id="TsenseI_v3_2", device=None, clock=None, start=True, recording_dir=None):
        if model_id not in MODEL_IDS:
            raise ValueError("不支持的 A3.2 型号")
        self.model_id, self.side = model_id, side
        self._lock = threading.RLock()
        self._io_stop = threading.Event()
        self._io_thread = None
        self._closed = False
        self.stop_reason = ""
        self.sample_ticks = 0
        self.last_control_dt_s = 0.
        self.max_control_dt_s = 0.
        self._clock = clock or time.perf_counter
        self.profile = load_hand_profile(model_id)
        self.device = device if device is not None else A32Device(side, model_id=model_id)
        self.limits = {name: tuple(0.0 if abs(value) < 1e-12 else value for value in bounds)
                       for name, bounds in self.device.joint_limits().items()}
        # Expose the authored flexion and swing ranges in the editor. Hardware
        # conversion clips to the mechanism intersection (including index/ring
        # MCP) and the narrower swing command range.
        self.limits.update({name: bounds for name, bounds in self.profile.joint_limits(side).items()
                            if requested_limits(name) is not None})
        self.pose = self.validate_pose(self.profile.init_pose(side))
        self.names = tuple(self.limits)
        self.target_pose = dict(self.pose)
        self.downlink_ranges = self._downlink_ranges()
        self.data_dir = Path(data_dir)
        self.recorder = A32SessionRecorder(recording_dir,
                                          model_id=model_id, hand_side=side,
                                          channels=getattr(self.device, "channels", None))
        self.generation = uuid.uuid4().hex
        self.motions, self.selected = {}, 0
        self.transition, self.interval = 1.0, .2
        self.loop = self.sync = self.armed = False
        self.dialog = self.playback = None
        self.home_slot, self.home_iq = 0, 0.0
        self.name = ""
        self.message = "A3.2 · 本机原生 SDK · 连接后可勾选实时控制；左右手均支持硬件控制"
        self._moving = False
        self._reset_planner()
        if self.path.exists():
            self.motions = self.validate_file(json.loads(self.path.read_text(encoding="utf-8")))
        if start:
            self._io_thread = threading.Thread(target=self._io_loop, name="A32-motion-play", daemon=True)
            self._io_thread.start()

    def _io_loop(self):
        # The service loop also handles HTTP jobs and dialogs. Keep the SDK clock
        # independent of its event queue, as A3.0 does with its I/O worker.
        with precise_control_timer():
            deadline = time.perf_counter()
            while not self._io_stop.is_set():
                self.tick()
                deadline += .01
                now = time.perf_counter()
                if deadline <= now:
                    deadline = now + .01
                self._io_stop.wait(max(0., deadline - now))

    @property
    def path(self):
        suffix = "_withid" if self.model_id.endswith("_withid") else ""
        return self.data_dir / f"motions_a32{suffix}_{self.side}.json"

    @property
    def connected(self):
        return bool(self.device.is_connected)

    def _downlink_ranges(self):
        if not hasattr(self.device, "prepare_targets"):
            return {}  # Lightweight test transports may omit pure preview data.
        endpoints = [self.device.prepare_targets({name: limits[i] for name, limits in self.limits.items()}, clamp=True)
                     for i in (0, 1)]
        result = {}
        for channel in self.device.channels:
            key = "self" if channel["channel"] == "self" else "mit_pos"
            values = [endpoint[key][channel["index"]] for endpoint in endpoints]
            joint = channel["joint"]
            lo, hi = self.limits[joint]
            curve = []
            # Keep the exact non-linear SDK conversion available to the web
            # editor so the stroke display can be used as a real slider scale.
            # The pose is complete for every sample; only this joint changes.
            for sample in range(33):
                fraction = sample / 32
                pose = dict(self.pose)
                pose[joint] = lo + fraction * (hi - lo)
                target = self.device.prepare_targets(pose, clamp=True)
                curve.append([pose[joint], target[key][channel["index"]]])
            result[joint] = {"min": min(values), "max": max(values), "unit": channel["unit"],
                             "slot": f"{channel['channel']} {channel['index']}",
                             "at_ui_min": values[0], "at_ui_max": values[1], "curve": curve}
        return result

    def _reset_planner(self):
        count = len(self.names)
        self.planner = Ruckig(count, .01)
        self.input, self.output = InputParameter(count), OutputParameter(count)
        self.input.current_position = [self.pose[n] for n in self.names]
        self.input.current_velocity = self.input.current_acceleration = [0.] * count
        self.input.target_position = list(self.input.current_position)
        self.input.target_velocity = self.input.target_acceleration = [0.] * count
        configure_ruckig(self.input, self.names)
        self._moving = False
        self._command_requested = False
        self.target_pose = dict(self.pose)
        self._last_tick = self._clock()

    def close(self):
        self._io_stop.set()
        if self._io_thread is not None and self._io_thread is not threading.current_thread():
            self._io_thread.join(timeout=2)
        with self._lock:
            if self._closed:
                return
            self._closed = True
            try:
                self.stop(reason="session_closed")
            finally:
                try:
                    self.device.close()
                finally:
                    try:
                        self.recorder.stop(reason="session_closed")
                    finally:
                        # Signal handlers/HTTP requests can retain this session
                        # until interpreter teardown. Release nanobind objects
                        # now, including the fresh planner created by stop().
                        self.planner = self.input = self.output = None

    @synchronized
    def stop(self, reason="user_stop"):
        self.playback = self.dialog = None
        self.sync = self.armed = False
        self.stop_reason = reason
        self._reset_planner()
        if self.connected:
            self.device.stop()
        self.recorder.record_action("stop", self.device.get_state(), reason=reason,
                                    pose=dict(self.pose), sample_ticks=self.sample_ticks)

    def spec(self, key, kind, **values):
        return {"id": f"{self.generation}:{key}", "kind": kind, "enabled": True, **values}

    @synchronized
    def snapshot(self):
        device = self.device.get_state()
        busy = bool(device.get("homing"))
        button = lambda key, text, enabled=True: self.spec(key, "button", text=text, enabled=enabled)
        check = lambda key, text, value, enabled=True: self.spec(key, "check", text=text, value=value, enabled=enabled and not busy)
        number = lambda key, value, maximum=600: self.spec(key, "number", value=value, min=0., max=maximum, step=.01, enabled=not busy)
        names = list(self.motions)
        controls = {
            "handSide": self.spec("handSide", "select", value=int(self.side == "right"), options=["左手", "右手"], enabled=not self.connected),
            "choosemotion": self.spec("choosemotion", "select", value=self.selected, options=names or ["无动作"], enabled=bool(names) and not busy),
            "savemotion": button("savemotion", "保存动作", not busy),
            "loadmotion": button("loadmotion", "加载选定动作", bool(names) and not busy),
            "deletemotion": button("deletemotion", "删除选定动作", bool(names) and not busy),
            "motionstreamplay": button("motionstreamplay", "停止播放" if self.playback else "动作序列播放", bool(names) and not busy),
            "transition": number("transition", self.transition), "interval": number("interval", self.interval),
            "motionStreamLoop": check("motionStreamLoop", "循环播放", self.loop),
            "isConnectOffline": check("isConnectOffline", "动作同步下位机", self.sync, self.connected),
            "realtimeJointControl": check("realtimeJointControl", "实时关节控制", self.armed and not self.sync, self.connected),
            "a32Enable": button("a32Enable", "使能 18 self + 2 MIT", self.connected and not busy and not device.get("enabled")),
            "a32Disable": button("a32Disable", "停止下发", self.connected),
            "a32Reset": button("a32Reset", "复位找零状态", self.connected),
            "a32Init": button("a32Init", "回到初始姿态", not busy),
            "a32Home": button("a32Home", "自研电机找零", self.connected and not busy),
            "a32HomeSlot": self.spec("a32HomeSlot", "select", value=self.home_slot, options=["全部 18 self"] + [f"self {i}" for i in range(18)], enabled=not busy),
            "a32HomeIq": self.spec("a32HomeIq", "number", value=self.home_iq, min=0., max=1000., step=1., enabled=not busy),
        }
        joints = []
        for name, (lo, hi) in self.limits.items():
            short = name.replace(self.side + "_hand_", "").replace("_joint2", "").replace("_joint", "").lower()
            joints.append(self.spec(name, "number", name=name, short=short, value=self.pose[name], min=lo, max=hi, step=.001,
                                    enabled=not busy and not self.playback, downlink=self.downlink_ranges.get(name)))
        dialogs = []
        if self.dialog:
            items = [self.spec("dialogInfo", "label", text=self.dialog["text"])]
            if self.dialog["action"] == "save":
                items.append(self.spec("motionName", "text", value=self.name))
            items += [button("confirm", "确认"), button("cancel", "取消")]
            dialogs = [{"id": self.dialog["id"], "title": self.dialog["title"], "modal": True, "items": items}]
        assets_root = self.profile.root.parents[2]
        urdf = "/" + self.profile.path("urdf", self.side).relative_to(assets_root).as_posix()
        return {"model_id": self.model_id, "side": self.side, "urdf": urdf,
                "controls": controls, "joints": joints, "dialogs": dialogs, "pose": dict(self.pose),
                "connected": self.connected, "playing": bool(self.playback), "homing": busy,
                "serial_mode": "native-sdk", "count": len(names), "file": self.path.name,
                "recording": self.recorder.snapshot(),
                "device": {**device, "armed": self.armed, "command_started": self.armed and self._command_requested,
                           "stop_reason": self.stop_reason,
                           "sample_ticks": self.sample_ticks, "control_dt_s": self.last_control_dt_s,
                           "max_control_dt_s": self.max_control_dt_s}, "message": self.message}

    def ask(self, action, title, text):
        self.dialog = {"id": uuid.uuid4().hex, "action": action, "title": title, "text": text}

    def payload(self, motions=None):
        return {"_model_id": self.model_id, "_hand_side": self.side,
                "_joint_coordinate_version": COORDINATE_VERSION,
                "_swing_axis_alignment": SWING_AXIS_ALIGNMENT,
                "motions": self.motions if motions is None else motions}

    def validate_pose(self, pose):
        if not isinstance(pose, dict) or pose.keys() != self.limits.keys():
            raise ValueError("A3.2 动作必须包含本型号和手别的完整 20 个关节")
        for name, value in pose.items():
            lo, hi = self.limits[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lo - 1e-8 <= value <= hi + 1e-8:
                raise ValueError(f"A3.2 关节数值超出模型与实机可用范围：{name}")
        return {name: min(self.limits[name][1], max(self.limits[name][0], value)) for name, value in pose.items()}

    def validate_file(self, data):
        if not isinstance(data, dict) or data.get("_model_id") != self.model_id or data.get("_hand_side") != self.side:
            raise ValueError("请选择相同型号和手别的动作文件；A3.2 两款模型与 A3.0 / V2 分开保存")
        motions = data.get("motions")
        if not isinstance(motions, dict) or len(motions) > 2000:
            raise ValueError("motions 必须为不超过 2000 个动作的对象")
        version = data.get("_joint_coordinate_version")
        if version not in (None, POSITIVE_COORDINATE_VERSION, COORDINATE_VERSION):
            raise ValueError("A3.2 动作坐标版本不受支持")
        alignment = POSITIVE_SWING_AXIS_ALIGNMENT if version == POSITIVE_COORDINATE_VERSION else SWING_AXIS_ALIGNMENT
        if data.get("_swing_axis_alignment") not in (None, alignment):
            raise ValueError("A3.2 动作 swing 轴向版本不受支持")
        motions = copy.deepcopy(motions)
        for name, entry in motions.items():
            if not isinstance(name, str) or not name.strip() or len(name) > 200 or not isinstance(entry, dict):
                raise ValueError("动作名称或内容无效")
            if version is None:
                entry["pose"] = migrate_legacy_pose(entry.get("pose"), self.limits, model_id=self.model_id)
            elif version == POSITIVE_COORDINATE_VERSION:
                entry["pose"] = migrate_positive_pose(entry.get("pose"), self.limits)
            self.validate_pose(entry.get("pose"))
            for field in ("transition", "interval"):
                value = entry.get(field)
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 600:
                    raise ValueError("动作时间必须为 0..600 秒")
        return motions

    def persist(self, motions):
        self.validate_file(self.payload(motions))
        self.data_dir.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.payload(motions), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        tmp.replace(self.path)
        self.motions = motions
        self.selected = max(0, min(self.selected, len(motions) - 1))

    @synchronized
    def move(self, pose, duration=.12):
        pose = self.validate_pose(pose)
        if self.device.get_state().get("homing"):
            raise ValueError("找零期间不能改变姿态")
        if not self.armed and not self.playback:
            self.pose = pose
            self._reset_planner()
            return
        self.target_pose = dict(pose)
        self.input.target_position = [pose[n] for n in self.names]
        self.input.minimum_duration = max(.01, duration)
        self._moving = True
        self._command_requested = True

    def start_segment(self):
        p = self.playback
        p["entry"] = p["entries"][p["index"]]
        p.pop("arrived", None)
        self.move(p["entry"]["pose"], p["entry"]["transition"])

    @synchronized
    def tick(self):
        try:
            now = self._clock()
            elapsed = max(1e-6, now - self._last_tick)
            self._last_tick = now
            state = self.device.get_state()
            self.recorder.record_state({**state, "armed": self.armed, "stop_reason": self.stop_reason,
                                        "sample_ticks": self.sample_ticks})
            if self.armed:
                self.last_control_dt_s = elapsed
                self.max_control_dt_s = max(self.max_control_dt_s, elapsed)
                if self._command_requested and elapsed > .1:
                    raise ValueError("SDK 轨迹采样间隔超过 100 ms，已停止下发")
                if not state.get("connected") or not state.get("enabled") or state.get("homing"):
                    raise ValueError("SDK 已断开、软件使能已关闭或正在找零，已停止下发")
            candidate = dict(self.pose)
            result = None
            if self._moving:
                self.planner.delta_time = elapsed
                result = self.planner.update(self.input, self.output)
                if result not in (Result.Working, Result.Finished):
                    raise ValueError(f"A3.2 轨迹计算失败：{result}")
                planned_pose = dict(zip(self.names, self.output.new_position))
                self.recorder.record_event("ruckig", joints=planned_pose, frame_id=self.sample_ticks)
                candidate = self.validate_pose(planned_pose)
                for name, bound in zip(self.names, self.output.trajectory.position_extrema):
                    lo, hi = self.limits[name]
                    if bound.min < lo - 1e-8 or bound.max > hi + 1e-8:
                        raise ValueError(f"A3.2 轨迹将超出关节可用范围：{name}")
            if self.armed and self._command_requested:
                try:
                    targets = self.device.send_joint_targets(candidate, clamp=True)
                except Exception as exc:
                    self.recorder.record_command(self._preview_targets(candidate), pose=candidate,
                                                 sent=False, error=str(exc), control_dt_s=elapsed)
                    raise
                self.recorder.record_command(targets, pose=candidate, control_dt_s=elapsed)
                self.sample_ticks += 1
            # Commit the displayed/next planner state only after a successful
            # device write.  A failed SDK send retains the last applied pose.
            self.pose = candidate
            if result is not None:
                self.output.pass_to_input(self.input)
                self.input.current_position = [candidate[n] for n in self.names]
                if result == Result.Finished:
                    self._moving = False
            if self.playback and not self._moving:
                p = self.playback
                p.setdefault("arrived", now)
                if now - p["arrived"] >= p["entry"]["interval"]:
                    p["index"] += 1
                    if p["index"] == len(p["entries"]):
                        if not self.loop:
                            self.playback = None
                            return
                        p["index"] = 0
                    self.start_segment()
        except Exception as exc:
            self.message = str(exc)
            self.recorder.record_error(exc, control_dt_s=self.last_control_dt_s, sample_ticks=self.sample_ticks)
            try:
                self.stop(reason=str(exc))
            except Exception as stop_error:
                self.message += f"；停止下发失败：{stop_error}"

    def _preview_targets(self, pose):
        try:
            return self.device.prepare_targets(pose, clamp=True)
        except Exception:
            return {"joint_angles_rad": dict(pose)}

    def _enable(self, seed):
        try:
            self.device.enable_all(start_pose=seed, send_initial_pose=False)
        except Exception as exc:
            self.recorder.record_error(exc, action="enable", start_pose=seed)
            raise
        self.recorder.record_action("enable", self.device.get_state(), start_pose=seed, initial_pose_sent=False)

    def recording_download(self):
        # Only this recorder's existing files can be downloaded. Active logs
        # are a flushed prefix; trim any trailing partial record during a write.
        files = [Path(name) for name in self.recorder.snapshot()["files"]]
        if not files:
            raise ValueError("尚无控制记录；连接 SDK 后会自动记录")
        def complete_bytes(path):
            data = path.read_bytes()
            end = data.rfind(b"\n")
            return data[:end + 1] if end >= 0 else b""
        if len(files) == 1:
            return complete_bytes(files[0]), "application/x-ndjson", files[0].name
        output = io.BytesIO()
        with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED) as bundle:
            for path in files:
                bundle.writestr(path.name, complete_bytes(path))
        return output.getvalue(), "application/zip", files[0].stem + ".zip"

    @synchronized
    def dispatch(self, action, body):
        if action == "connect":
            if self.connected:
                raise ValueError("请先断开当前 SDK 设备")
            self.stop()
            self.recorder.stop(reason="reconnect")
            self.recorder.start(metadata={"platform": "motion_play", "transport": body.get("transport", "usb")})
            self.recorder.record_action("connect_requested", request=dict(body))
            try:
                if body.get("transport", "usb") == "usb":
                    self.device.connect_usb(str(body.get("port", "auto")))
                elif body.get("transport") == "eth":
                    self.device.connect_eth(str(body.get("host", "")), body.get("tcp_port", 5001))
                else:
                    raise ValueError("SDK 连接类型必须为 usb 或 eth")
            except Exception as exc:
                self.recorder.record_error(exc, action="connect")
                self.recorder.stop(reason="connect_failed")
                raise
            self.recorder.record_action("connected", self.device.get_state())
            self.stop_reason = ""
            self.message = "SDK 已连接；收到完整有效反馈后可开启实时控制或动作同步"
        elif action == "disconnect":
            try:
                self.stop()
            finally:
                try:
                    self.device.disconnect()
                finally:
                    self.recorder.record_action("disconnected", self.device.get_state())
                    self.recorder.stop(reason="disconnected")
        elif action == "stop":
            reason = "page_hidden" if body.get("reason") == "page_hidden" else "user_stop"
            self.stop(reason=reason)
            self.message = "页面已切到后台，已暂停下发并保留 SDK 连接；返回后勾选实时控制或动作同步恢复" if reason == "page_hidden" else "已停止下发；可重新勾选实时控制或动作同步"
        elif action == "export":
            return self.payload()
        elif action == "recording_download":
            return self.recording_download()
        elif action == "import":
            motions = self.validate_file(json.loads(body["text"].lstrip("\ufeff")))
            self.stop()
            self.persist(motions)
        elif action == "widget":
            self.widget(body)
        else:
            raise ValueError("未知 A3.2 操作")
        return True

    @synchronized
    def widget(self, body):
        ident = str(body.get("id", ""))
        if not ident.startswith(self.generation + ":"):
            raise ValueError("模型或手别已变化，请重试")
        key = ident.split(":", 1)[1]
        state = self.snapshot()
        specs = list(state["controls"].values()) + state["joints"] + [s for d in state["dialogs"] for s in d["items"]]
        spec = next((s for s in specs if s["id"] == ident), None)
        if not spec or not spec["enabled"]:
            raise ValueError("此控件当前不可用")
        if self.dialog and key not in ("confirm", "cancel", "motionName", "a32Disable", "a32Reset"):
            raise ValueError("请先处理确认对话框")
        value = body.get("value")
        if spec["kind"] == "number" and (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not spec["min"] <= value <= spec["max"]):
            raise ValueError("数值超出允许范围")
        if spec["kind"] == "check" and not isinstance(value, bool):
            raise ValueError("复选框值无效")
        if spec["kind"] == "select" and (type(value) is not int or not 0 <= value < len(spec["options"])):
            raise ValueError("选项不存在")
        self.recorder.record_action("widget", widget=key, value=value)
        if key in self.pose:
            self.move({**self.target_pose, key: value})
        elif key in ("transition", "interval"):
            setattr(self, key, value)
        elif key == "a32HomeSlot":
            self.home_slot = value
        elif key == "a32HomeIq":
            self.home_iq = float(value)
        elif key == "choosemotion":
            self.selected = value
            entry = list(self.motions.values())[value]
            self.transition, self.interval = entry["transition"], entry["interval"]
        elif key == "motionStreamLoop":
            self.loop = value
        elif key == "a32Enable":
            self._enable(self.validate_pose(self.device.starting_pose(self.pose)))
            self.message = "SDK 已使能，尚未下发轨迹；可开启实时关节控制或动作同步"
        elif key == "a32Disable":
            self.stop()
            self.message = "已停止下发，未发送失能；设备可能继续执行已接收的命令或找零"
        elif key == "a32Reset":
            self.stop()
            self.device.reset_home(slot=None if self.home_slot == 0 else self.home_slot - 1)
            self.message = "已停止下发并请求复位所选 self 找零状态，未发送失能；请检查反馈后重新找零/使能"
        elif key in ("isConnectOffline", "realtimeJointControl"):
            self.playback = None
            if value:
                seed = self.validate_pose(self.device.starting_pose(self.pose))
                if not self.device.get_state().get("enabled"):
                    self._enable(seed)
                self.pose = seed
                self._reset_planner()
                self.stop_reason = ""
                self.max_control_dt_s = 0.
                self.armed = True
                self.sync = key == "isConnectOffline"
                self.message = "SDK 已使能，等待调整关节或启动动作；勾选本身不下发姿态"
            else:
                self.stop()
        elif key == "a32Init":
            self.playback = None
            self.move(self.profile.init_pose(self.side), self.transition)
        elif key == "a32Home":
            if self.home_iq <= 0 or not self.home_iq.is_integer():
                raise ValueError("请先填写已标定的正整数找零 Iq 码值")
            self.ask("home", "确认 A3.2 自研电机找零", f"通过 SDK 对 {state['controls']['a32HomeSlot']['options'][self.home_slot]} 找零，Iq={self.home_iq:g}（码值）。MIT 两路不参与。确认机械空间与电流参数后继续；完成后需重新使能。")
        elif key == "savemotion":
            self.name = list(self.motions)[self.selected] if self.motions else "动作1"
            self.ask("save", "保存 A3.2 动作", "输入动作名称；同名动作将覆盖。")
        elif key == "motionName":
            if not isinstance(value, str) or len(value) > 200:
                raise ValueError("动作名称过长")
            self.name = value
        elif key == "deletemotion":
            self.ask("delete", "删除 A3.2 动作", "确认删除：" + list(self.motions)[self.selected])
        elif key == "loadmotion":
            self.playback = None
            self.move(list(self.motions.values())[self.selected]["pose"], self.transition)
        elif key == "motionstreamplay":
            if self.playback:
                self.stop()
            else:
                if self.armed and not self.sync:
                    raise ValueError("请使用动作同步下位机播放，或先关闭实时关节控制")
                self.playback = {"entries": list(self.motions.values())[self.selected:], "index": 0}
                self.start_segment()
        elif key == "cancel":
            self.dialog = None
        elif key == "confirm":
            action = self.dialog["action"]
            if action == "save":
                if not self.name.strip():
                    raise ValueError("动作名称不能为空")
                self.persist({**self.motions, self.name.strip(): {"pose": dict(self.pose), "transition": self.transition, "interval": self.interval}})
                self.selected = list(self.motions).index(self.name.strip())
            elif action == "delete":
                motions = dict(self.motions)
                motions.pop(list(motions)[self.selected])
                self.persist(motions)
            elif action == "home":
                self.stop()
                self.device.home(slot=None if self.home_slot == 0 else self.home_slot - 1, iq_counts=int(self.home_iq))
            self.dialog = None
        else:
            raise ValueError("未知 A3.2 控件")
