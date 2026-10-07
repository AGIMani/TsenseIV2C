"""Server-side A3.0 serial transport. No V2 commands, no automatic enable.

The live browser session uses a dedicated timed I/O worker. Public device
operations share a reentrant lock; UI callbacks never own serial scheduling.
"""
import math
from functools import wraps
import struct
import threading
import time

import serial
from ruckig import Ruckig, InputParameter, OutputParameter, Result
from a30_kinematics.downlink import (
    crc16_modbus, pack_hand_batch, pack_self_homing,
    pack_self_pwr, slot_to_bus_node, SELF_JOINTS,
)
from a30_kinematics.feedback import measured_pose
from downlink_policy import encode_online_targets as encode_targets
from utils.control_clock import precise_control_timer
from online_control.shared.downlink import configure_ruckig


def synchronized(method):
    @wraps(method)
    def call(self, *args, **kwargs):
        with self.lock:
            return method(self, *args, **kwargs)
    return call


class A30Device:
    # Playback owns its rate; gesture control keeps the shared 500 Hz clock.
    CONTROL_DT = .001

    def __init__(self, mapper, serial_factory=serial.Serial, clock=time.perf_counter):
        self.lock = threading.RLock()
        self.io_stop = threading.Event()
        self.io_thread = None
        self.mapper, self.serial_factory, self.clock = mapper, serial_factory, clock
        self.port = None
        self.buffer = bytearray()
        self.payload = None
        self.feedback_at = -math.inf
        self.feedback_key = None
        self.seed_missing = []
        self.seed_clipped = []
        self.seq = 0
        self.armed = False
        self.homing = None
        self.report = []
        self.error = ""
        self.rx_count = self.tx_count = 0
        self.pose = {n: 0. if abs(lo) < 1e-12 else min(hi, max(lo, 0.)) for n, (lo, hi) in zip(mapper.joint_names, mapper._limits)}
        self.target = dict(self.pose)
        self.ruckig = Ruckig(20, self.CONTROL_DT)
        self.input, self.output = InputParameter(20), OutputParameter(20)
        configure_ruckig(self.input, mapper.joint_names)
        self.last_tick = self.clock()
        self.accumulator = 0.
        self.last_send = -math.inf
        self.next_send = self.clock()
        self.log_hex = False

    @property
    @synchronized
    def connected(self):
        return self.port is not None and self.port.is_open

    def start_io(self):
        if self.io_thread is not None and self.io_thread.is_alive():
            return
        self.io_stop.clear()
        self.io_thread = threading.Thread(target=self._io_loop, name="A30-serial", daemon=True)
        self.io_thread.start()

    def close_io(self):
        self.io_stop.set()
        if self.io_thread is not None:
            self.io_thread.join(timeout=2)
        self.disconnect()

    @property
    def io_running(self):
        return self.io_thread is not None and self.io_thread.is_alive()

    def _io_loop(self):
        with precise_control_timer():
            deadline = time.perf_counter()
            while not self.io_stop.is_set():
                if not self.connected:
                    self.io_stop.wait(.02)
                    deadline = time.perf_counter()
                    continue
                self.tick()
                deadline += self.CONTROL_DT
                now = time.perf_counter()
                if deadline <= now:
                    deadline += (math.floor((now-deadline)/self.CONTROL_DT)+1)*self.CONTROL_DT
                # Python's high-resolution sleep avoids the offscreen Qt timer
                # granularity on Windows. Sleep to the absolute 1 ms schedule;
                # OS scheduling can still delay wakeup.
                time.sleep(max(0., deadline-time.perf_counter()))

    @synchronized
    def connect(self, port):
        self.disconnect()
        self.port = self.serial_factory(port, baudrate=115200, timeout=0, write_timeout=.05)
        self.buffer.clear()
        self.payload = self.feedback_key = None
        self.feedback_at = -math.inf
        self.seed_missing = []
        self.seed_clipped = []
        self.error = "等待 A3.0 H7 遥测；实时控制不要求找零或全部电机在线"
        # Opening the port never writes hand-type, positions, enable or homing.

    @synchronized
    def write(self, frame, *, q=None):
        if not self.connected:
            raise ValueError("请先连接 A3.0 H7 串口")
        # Live positions and arm's baseline carry the ordered URDF pose for a
        # capable transport's validation and last-write state. Other commands
        # stay ordinary; transport sampling gaps do not disconnect the port.
        trajectory_writer = getattr(self.port, "write_trajectory", None) if q is not None else None
        written = trajectory_writer(frame, q) if callable(trajectory_writer) else self.port.write(frame)
        if written != len(frame):
            raise OSError("A3.0 串口写入不完整")
        self.tx_count += 1
        if self.log_hex:
            print("[A3.0 TX] " + frame.hex(" "), flush=True)

    @synchronized
    def read(self):
        if not self.connected:
            return
        # Bounded work per tick, retaining partial frames and resynchronizing CRC.
        self.buffer.extend(self.port.read(min(self.port.in_waiting, 16384)))
        while len(self.buffer) >= 8:
            if self.buffer[:2] != b"\xaa\x55":
                del self.buffer[0]
                continue
            length = struct.unpack_from("<H", self.buffer, 2)[0]
            # Same frame-size bound as A3_USB_Downlink.pop_aa55_frames.
            # Validate the whole wire frame before trimming firmware extensions.
            if length == 0 or length + 8 > 2048:
                del self.buffer[0]
                continue
            if len(self.buffer) < length + 8:
                break
            frame = bytes(self.buffer[:length+8])
            if crc16_modbus(frame[:-2]) != struct.unpack_from("<H", frame, len(frame)-2)[0]:
                del self.buffer[0]
                continue
            del self.buffer[:length+8]
            if length >= 444 and frame[4:6] == b"\x03\x02":
                take = 484 if length >= 484 else 444
                payload = frame[6:6+take]
                key = (struct.unpack_from("<H", payload)[0], struct.unpack_from("<I", payload, 4)[0])
                if key != self.feedback_key:
                    self.payload, self.feedback_key = payload, key
                    self.feedback_at = self.clock()
                    self.rx_count += 1
                    if self.error.startswith("等待 A3.0 H7 遥测"):
                        self.error = ""

    @synchronized
    def require_feedback(self, *, all_online=False):
        p = self.payload
        if p is None or self.clock() - self.feedback_at > .5:
            raise ValueError("A3.0 遥测缺失或超过 500 ms，请等待设备反馈后重新开启")
        mask = struct.unpack_from("<I", p, 8)[0]
        mit_mask = struct.unpack_from("<H", p, 2)[0] if len(p) == 484 else 0
        if all_online and (mask & 0x3ffff != 0x3ffff or mit_mask & 3 != 3):
            raise ValueError("A3.0 需要全部 18+2 路电机在线")
        for i in range(18):
            if mask & (1 << i) and not math.isfinite(struct.unpack_from("<f", p, 16+24*i)[0]):
                raise ValueError("A3.0 位置反馈无效")
        for i in range(2):
            if not mit_mask & (1 << i):
                continue
            o = 444 + 20*i
            # As in the shared A3 feedback parser, the online mask is authoritative;
            # firmware variants cache different CAN identifiers in these bytes.
            if struct.unpack_from("<I", p, o+12)[0] or p[o+16] == 5:
                raise ValueError("A3.0 外接电机反馈无效或报告故障")
        return p

    @synchronized
    def hold_feedback(self):
        p = self.require_feedback(all_online=True)
        own = [struct.unpack_from("<f", p, 16+24*i)[0] for i in range(18)]
        # Raw external motor-side values: do NOT multiply the reduction twice.
        ext = [(struct.unpack_from("<I", p, 446+20*i)[0]-0x80000000)/2097152*360 for i in range(2)]
        self.write(pack_hand_batch(self.seq, own, ext, [0., 0.], [0., 0.]))
        self.seq = (self.seq + 1) & 255

    @synchronized
    def arm(self, fallback_pose=None):
        if self.homing:
            raise ValueError("请等待找零结束")
        if not self.connected:
            raise ValueError("请先连接 A3.0 H7 串口")
        self.read()
        self.seed_missing = []
        self.seed_clipped = []
        clipped = []
        try:
            pose = measured_pose(self.require_feedback(), self.mapper,
                                 fallback_pose=dict(self.pose if fallback_pose is None else fallback_pose),
                                 missing_joints=self.seed_missing, clipped_joints=clipped,
                                 clamp=True)
        except ValueError as exc:
            self.error = str(exc)
            raise
        self.seed_clipped = clipped
        self.mapper.convert(pose)
        self.pose = self.target = dict(pose)
        self.input.current_position = [pose[n] for n in self.mapper.joint_names]
        self.input.current_velocity = [0.] * 20
        self.input.current_acceleration = [0.] * 20
        self.input.target_position = list(self.input.current_position)
        self.input.target_velocity = [0.] * 20
        self.input.target_acceleration = [0.] * 20
        self.input.minimum_duration = None
        try:
            # In-range axes hold their position; out-of-range feedback uses the
            # nearest model boundary, and absent axes use the editor pose.
            # Never send uninitialized offline telemetry as a position target.
            self.write(encode_targets(self.mapper.convert(pose), hand_side=self.mapper.hand_side, seq=self.seq),
                       q=[float(pose[n]) for n in self.mapper.joint_names])
            self.seq = (self.seq+1) & 255
            for slot in range(18):
                self.write(pack_self_pwr(*slot_to_bus_node(slot), 1))
        except Exception:
            self.stop(power_off=True)
            raise
        self.accumulator = 0.
        self.last_tick = self.clock()
        self.next_send = self.last_tick + self.CONTROL_DT
        self.error, self.armed = "", True
        return dict(pose)

    @synchronized
    def set_target(self, pose, minimum_duration=None):
        self.mapper.convert(pose)  # Complete names, finite values, mechanism limits.
        self.target = dict(pose)
        self.input.minimum_duration = minimum_duration

    @synchronized
    def stop(self, power_off=False):
        was_homing = self.homing is not None
        self.armed, self.homing = False, None
        if self.connected and (was_homing or power_off):
            for slot in range(18):
                try:
                    self.write(pack_self_homing(*slot_to_bus_node(slot), 2, 0.))
                    if power_off:
                        self.write(pack_self_pwr(*slot_to_bus_node(slot), 2))
                except (OSError, ValueError):
                    self.error = "停止命令未全部写入，请检查设备"
                    break

    @synchronized
    def disconnect(self):
        self.stop()
        if self.port is not None:
            self.port.close()
        self.port = None
        self.payload = None
        self.seed_clipped = []

    @synchronized
    def reset_home(self):
        self.read()
        self.require_feedback()
        self.stop()
        for slot in range(18):
            self.write(pack_self_homing(*slot_to_bus_node(slot), 2, 0.))
        self.error = "已发送18路复位回零态；不改变关节目标，不复位外接两路"

    @synchronized
    def start_home(self, linear_iq, rotary_iq):
        if any(isinstance(iq, bool) or not isinstance(iq, (int, float)) or not math.isfinite(iq) or not 0 < iq <= 5000 for iq in (linear_iq, rotary_iq)):
            raise ValueError("直线与旋转找零 Iq 码值须分别设置为 (0,5000]")
        self.stop()
        self.read()
        self.require_feedback(all_online=True)
        self.hold_feedback()
        self.report = []
        self.homing = {"slot": 0, "phase": "next", "linear_iq": linear_iq, "rotary_iq": rotary_iq}
        self.error = ""

    @synchronized
    def home_step(self):
        from a30_kinematics.downlink import SELF_ANGLE_SLOTS
        h = self.homing
        self.require_feedback(all_online=True)
        slot = h["slot"]
        if slot == 18:
            self.report.extend({"joint": name, "success": False, "reason": "源协议未提供独立找零，实时控制从实际反馈起步"}
                               for name in ("thumb_rotaback_joint2", "thumb_rota_block_joint"))
            self.homing = None
            self.error = f"找零结束：自研 {sum(r['success'] for r in self.report)}/18 路完成；外接2路不支持独立找零"
            return
        iq = h["rotary_iq"] if slot in SELF_ANGLE_SLOTS else h["linear_iq"]
        address = slot_to_bus_node(slot)
        now = self.clock()
        if h["phase"] == "next":
            self.write(pack_self_homing(*address, 2, iq))
            h.update(phase="reset", since=now, key=self.feedback_key)
            return
        fresh = self.feedback_key != h["key"]
        o = 12 + 24*slot
        state = self.payload[o+3] if self.payload[o:o+3] == bytes(3) else None
        if h["phase"] == "reset" and fresh and state == 0:
            self.write(pack_self_pwr(*address, 1))
            self.write(pack_self_homing(*address, 1, iq))
            h.update(phase="seek", since=now, key=self.feedback_key)
            return
        done = h["phase"] == "seek" and fresh and state == 2
        failed = h["phase"] == "seek" and fresh and state == 3
        timeout = now-h["since"] > (1.5 if h["phase"] == "reset" else 15.)
        if done or failed or timeout:
            if not done:
                self.write(pack_self_homing(*address, 2, iq))
            self.report.append({"joint": SELF_JOINTS[slot], "slot": slot, "success": done,
                                "reason": "完成" if done else "下位机找零错误" if failed else "等待复位/找零确认超时"})
            h.update(slot=slot+1, phase="next")

    @synchronized
    def tick(self):
        now = self.clock()
        elapsed = now - self.last_tick
        self.last_tick = now
        try:
            self.read()
            if self.homing:
                self.home_step()
            elif self.armed:
                self.require_feedback()
                if elapsed > .1:
                    raise ValueError("控制循环超过100 ms，已停发，请重新开启实时控制")
                self.accumulator += elapsed
                self.input.target_position = [self.target[n] for n in self.mapper.joint_names]
                while self.accumulator + 1e-12 >= self.CONTROL_DT:
                    result = self.ruckig.update(self.input, self.output)
                    if result not in (Result.Working, Result.Finished):
                        raise ValueError(f"Ruckig 无法生成轨迹：{result}")
                    self.output.pass_to_input(self.input)
                    self.pose = dict(zip(self.mapper.joint_names, self.output.new_position))
                    self.accumulator = max(0., self.accumulator - self.CONTROL_DT)
                if now + 1e-12 >= self.next_send:
                    self.write(encode_targets(self.mapper.convert(self.pose), hand_side=self.mapper.hand_side, seq=self.seq),
                               q=[float(self.pose[n]) for n in self.mapper.joint_names])
                    self.seq = (self.seq+1) & 255
                    self.last_send = now
                    # Maintain the absolute schedule through timer jitter;
                    # missed positions are skipped, never sent in a burst.
                    missed = max(1, math.floor((now-self.next_send + 1e-12)/self.CONTROL_DT) + 1)
                    self.next_send += missed * self.CONTROL_DT
        except (ValueError, OSError, RuntimeError) as exc:
            self.stop()
            self.error = str(exc)
            if isinstance(exc, OSError):
                self.disconnect()

    @synchronized
    def status(self):
        try:
            self.require_feedback()
            fresh = True
        except ValueError:
            fresh = False
        mask = struct.unpack_from("<I", self.payload, 8)[0] if self.payload else 0
        mit_mask = struct.unpack_from("<H", self.payload, 2)[0] if self.payload and len(self.payload) == 484 else 0
        return {"fresh": fresh, "self_online": (mask & 0x3ffff).bit_count(),
                "external_online": (mit_mask & 3).bit_count(), "seed_missing": list(self.seed_missing),
                "seed_clipped": list(self.seed_clipped),
                "armed": self.armed, "rx": self.rx_count, "tx": self.tx_count,
                "error": self.error, "homing": dict(self.homing) if self.homing else None,
                "report": self.report, "external_homing_supported": False}
