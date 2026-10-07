"""A3.0 manual motion editor; isolated model, files and H7 transport."""
import json
import math
from pathlib import Path
import time
import uuid

from a30_kinematics import A30StrokeMapper
from a30_device import A30Device
from downlink_policy import a30_ranges


class A30Session:
    def __init__(self, data_dir, side="left", *, model_id="TsenseI_v3_0"):
        self.model_id = model_id
        self.data_dir = Path(data_dir)
        self.side = side
        self.mapper = A30StrokeMapper(side, model_id=model_id)
        # Prepared conversion buffers are isolated from the I/O worker.
        self.device = A30Device(A30StrokeMapper(side, model_id=model_id))
        self.downlink_ranges = a30_ranges(self.mapper)
        self.pose = dict(self.device.pose)
        self.limits = {n: (0. if abs(lo) < 1e-12 else lo, hi)
                       for n, (lo, hi) in zip(self.mapper.joint_names, self.mapper._limits)}
        self.generation = uuid.uuid4().hex
        self.motions = {}
        self.selected = 0
        self.transition, self.interval = 1., .2
        self.loop, self.sync = False, False
        self.linear_iq = self.rotary_iq = 0.
        self.dialog = None
        self.name = ""
        self.playback = None
        self.message = "A3.0 · 20关节 · 浏览器本机串口 · 等待操作"
        if self.path.exists():
            self.motions = self.validate_file(json.loads(self.path.read_text(encoding="utf-8")))
        # The service loop calls tick(); device I/O retains its 2 ms worker.

    @property
    def path(self):
        return self.data_dir / f"motions_a30{'_withid' if self.model_id.endswith('_withid') else ''}_{self.side}.json"

    def close(self):
        self.stop()
        self.device.close_io()

    def stop(self):
        self.playback = self.dialog = None
        self.sync = False
        self.device.stop()

    def spec(self, key, kind, **values):
        return {"id": f"{self.generation}:{key}", "kind": kind, "enabled": True, **values}

    def snapshot(self):
        busy = bool(self.device.homing)
        button = lambda key, text, enabled=True: self.spec(key, "button", text=text, enabled=enabled)
        check = lambda key, text, value: self.spec(key, "check", text=text, value=value, enabled=not busy)
        number = lambda key, value, maximum=600: self.spec(key, "number", value=value, min=0., max=maximum, step=.01, enabled=not busy)
        names = list(self.motions)
        controls = {
            "handSide": self.spec("handSide", "select", value=int(self.side == "right"), options=["左手", "右手"], enabled=not self.device.connected),
            "choosemotion": self.spec("choosemotion", "select", value=self.selected, options=names or ["无动作"], enabled=bool(names) and not busy),
            "savemotion": button("savemotion", "保存动作", not busy),
            "loadmotion": button("loadmotion", "加载选定动作", bool(names) and not busy),
            "deletemotion": button("deletemotion", "删除选定动作", bool(names) and not busy),
            "motionstreamplay": button("motionstreamplay", "停止播放" if self.playback else "动作序列播放", bool(names) and not busy),
            "transition": number("transition", self.transition), "interval": number("interval", self.interval),
            "motionStreamLoop": check("motionStreamLoop", "循环播放", self.loop),
            "isConnectOffline": check("isConnectOffline", "动作同步下位机", self.sync),
            "realtimeJointControl": check("realtimeJointControl", "实时关节控制", self.device.armed and not self.sync),
            "openSendLog": check("openSendLog", "显示下发HEX", self.device.log_hex),
            "a30Home": button("a30Home", "18路一键找零", self.device.connected and not busy),
            "a30Reset": button("a30Reset", "复位回零态", self.device.connected),
            "a30Init": button("a30Init", "回到初始姿态", not busy),
            "a30PowerOff": button("a30PowerOff", "停止 / 自研断电", self.device.connected),
            "a30LinearIq": number("a30LinearIq", self.linear_iq, 5000),
            "a30RotaryIq": number("a30RotaryIq", self.rotary_iq, 5000),
        }
        joints = []
        for name, (lo, hi) in self.limits.items():
            short = name.replace(self.side+"_hand_", "").replace("_joint2", "").replace("_joint", "").lower()
            joints.append(self.spec(name, "number", name=name, short=short, value=self.pose[name], min=lo, max=hi, step=.001,
                                    enabled=not busy and not self.playback, downlink=self.downlink_ranges[name]))
        dialogs = []
        if self.dialog:
            items = [self.spec("dialogInfo", "label", text=self.dialog["text"])]
            if self.dialog["action"] == "save":
                items.append(self.spec("motionName", "text", value=self.name))
            items += [button("confirm", "确认"), button("cancel", "取消")]
            dialogs = [{"id": self.dialog["id"], "title": self.dialog["title"], "modal": True, "items": items}]
        return {"model_id": self.model_id, "side": self.side,
                "urdf": f"/assets/hands/TsenseI_hand_{self.model_id.removeprefix('TsenseI_')}/urdf/TsenseI_hand_{self.side}_{self.model_id.removeprefix('TsenseI_')}.urdf",
                "controls": controls, "joints": joints, "dialogs": dialogs,
                "pose": dict(self.device.pose if self.device.armed else self.pose),
                "connected": self.device.connected, "playing": bool(self.playback), "homing": busy,
                "count": len(names), "file": self.path.name, "device": self.device.status(), "message": self.message}

    def ask(self, action, title, text):
        self.dialog = {"id": uuid.uuid4().hex, "action": action, "title": title, "text": text}

    def payload(self, motions=None):
        return {"_model_id": self.model_id, "_hand_side": self.side,
                "motions": self.motions if motions is None else motions}

    def persist(self, motions):
        self.data_dir.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.payload(motions), ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
        tmp.replace(self.path)
        self.motions = motions
        self.selected = max(0, min(self.selected, len(motions)-1))

    def validate_file(self, data):
        if not isinstance(data, dict) or data.get("_model_id") != self.model_id or data.get("_hand_side") != self.side:
            raise ValueError("请选择相同型号和手别的动作文件，A3.0 与 A3.0 with ID 动作独立保存")
        motions = data.get("motions")
        if not isinstance(motions, dict) or len(motions) > 2000:
            raise ValueError("A3.0 motions 必须为不超过2000个动作的对象")
        for name, entry in motions.items():
            if not isinstance(name, str) or not name.strip() or len(name) > 200 or not isinstance(entry, dict):
                raise ValueError("动作名称或内容无效")
            self.mapper.convert(entry.get("pose"))
            for field in ("transition", "interval"):
                value = entry.get(field)
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 600:
                    raise ValueError("动作时间必须为0..600秒")
        return motions

    def move(self, pose):
        self.mapper.convert(pose)
        if self.device.homing:
            raise ValueError("找零期间不能改变姿态")
        self.pose = dict(pose)
        if self.device.armed:
            self.device.set_target(pose)

    def start_segment(self):
        p = self.playback
        entry = p["entries"][p["index"]]
        p.update(start=time.perf_counter(), origin=dict(self.device.pose if self.device.armed else self.pose), entry=entry)
        if self.device.armed:
            self.device.set_target(entry["pose"], minimum_duration=max(.01, entry["transition"]))

    def tick(self):
        if not self.device.io_running:
            self.device.tick()
        if self.sync and not self.device.armed:
            self.playback, self.sync = None, False
        if not self.playback:
            return
        p = self.playback
        entry = p["entry"]
        elapsed = time.perf_counter()-p["start"]
        fraction = min(1., elapsed/max(.01, entry["transition"]))
        if self.device.armed:
            self.pose = dict(self.device.pose)
            reached = max(abs(self.pose[n]-entry["pose"][n]) for n in self.pose) < 1e-5
        else:
            self.pose = {n: p["origin"][n]+fraction*(entry["pose"][n]-p["origin"][n]) for n in self.pose}
            reached = fraction == 1.
        if reached and fraction == 1.:
            p.setdefault("arrived", time.perf_counter())
            if time.perf_counter()-p["arrived"] >= entry["interval"]:
                p.pop("arrived")
                p["index"] += 1
                if p["index"] == len(p["entries"]):
                    if not self.loop:
                        self.playback = None
                        return
                    p["index"] = 0
                self.start_segment()

    def dispatch(self, action, body):
        if action == "connect":
            self.stop()
            self.device.connect(body["port"])
        elif action == "disconnect":
            self.stop()
            self.device.disconnect()
        elif action == "stop":
            self.stop()
        elif action == "export":
            return self.payload()
        elif action == "import":
            motions = self.validate_file(json.loads(body["text"].lstrip("\ufeff")))
            self.stop()
            self.persist(motions)
        elif action == "widget":
            self.widget(body)
        else:
            raise ValueError("未知 A3.0 操作")
        return True

    def widget(self, body):
        ident = str(body.get("id", ""))
        if not ident.startswith(self.generation+":"):
            raise ValueError("模型或手别已变化，请重试")
        key = ident.split(":", 1)[1]
        state = self.snapshot()
        specs = list(state["controls"].values()) + state["joints"] + [s for d in state["dialogs"] for s in d["items"]]
        spec = next((s for s in specs if s["id"] == ident), None)
        if not spec or not spec["enabled"]:
            raise ValueError("此控件当前不可用")
        if self.dialog and key not in ("confirm", "cancel", "motionName", "a30PowerOff", "a30Reset"):
            raise ValueError("请先处理确认对话框")
        value = body.get("value")
        if spec["kind"] == "number":
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not spec["min"] <= value <= spec["max"]:
                raise ValueError("数值超出允许范围")
        if spec["kind"] == "check" and not isinstance(value, bool):
            raise ValueError("复选框值无效")
        if spec["kind"] == "select" and (type(value) is not int or not 0 <= value < len(spec["options"])):
            raise ValueError("选项不存在")
        if key in self.pose:
            self.move({**self.pose, key: value})
        elif key in ("transition", "interval"):
            setattr(self, key, value)
        elif key in ("a30LinearIq", "a30RotaryIq"):
            setattr(self, "linear_iq" if key == "a30LinearIq" else "rotary_iq", value)
        elif key == "choosemotion":
            self.selected = value
            entry = list(self.motions.values())[value]
            self.transition, self.interval = entry["transition"], entry["interval"]
        elif key == "motionStreamLoop":
            self.loop = value
        elif key == "openSendLog":
            self.device.log_hex = value
        elif key in ("isConnectOffline", "realtimeJointControl"):
            self.playback = None
            if value:
                self.pose = self.device.arm(fallback_pose=self.pose)
                self.sync = key == "isConnectOffline"
            else:
                self.device.stop()
                self.sync = False
        elif key == "a30PowerOff":
            self.stop()
            self.device.stop(power_off=True)
            self.message = "已请求自研18路断电；外接2路仅停发（协议未提供独立断电）"
        elif key == "a30Reset":
            self.stop()
            self.device.reset_home()
        elif key == "a30Init":
            self.playback = None
            self.move({n: min(hi, max(lo, 0.)) for n, (lo, hi) in self.limits.items()})
        elif key == "a30Home":
            if self.linear_iq <= 0 or self.rotary_iq <= 0:
                raise ValueError("请先分别填写直线和旋转电机已标定的找零 Iq 码值")
            self.ask("home", "确认 A3.0 找零", f"将逐轴复位、使能并找零18路自研电机。直线 Iq={self.linear_iq:g}，旋转 Iq={self.rotary_iq:g}（码值，不是A）。外接2路无独立找零指令。确认机械活动空间与电流参数后继续。")
        elif key == "savemotion":
            self.name = list(self.motions)[self.selected] if self.motions else "动作1"
            self.ask("save", "保存 A3.0 动作", "输入动作名称；同名动作将覆盖。")
        elif key == "motionName":
            if not isinstance(value, str) or len(value) > 200:
                raise ValueError("动作名称过长")
            self.name = value
        elif key == "deletemotion":
            self.ask("delete", "删除 A3.0 动作", "确认删除：" + list(self.motions)[self.selected])
        elif key == "loadmotion":
            self.playback = None
            self.move(list(self.motions.values())[self.selected]["pose"])
        elif key == "motionstreamplay":
            if self.playback:
                self.stop()
            else:
                if self.device.armed and not self.sync:
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
                entry = {"pose": dict(self.pose), "transition": self.transition, "interval": self.interval}
                self.persist({**self.motions, self.name.strip(): entry})
                self.selected = list(self.motions).index(self.name.strip())
            elif action == "delete":
                motions = dict(self.motions)
                motions.pop(list(motions)[self.selected])
                self.persist(motions)
            elif action == "home":
                self.stop()
                self.device.start_home(self.linear_iq, self.rotary_iq)
            self.dialog = None
        else:
            raise ValueError("未知 A3.0 控件")
