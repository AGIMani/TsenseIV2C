"""Qt-free V2 motion editor and single-owner 500 Hz hardware control.

Control/dialog descriptions are explicit data. Trajectories reuse the shared
Ruckig, protocol conversion and DownlinkGuard, without GUI compatibility objects.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import threading
import time
import uuid

from joint_schema import JOINT_LIMITS, URDF_JOINT_MAP, default_joint_position
from protocol import (convert_hardware_to_ui, convert_ui_to_display,
                      convert_ui_to_hardware, remap_motion_cfg_file_to_ui,
                      remap_motion_cfg_ui_to_file)
from usb.offline import (DEFAULT_HOMING_IQ_SEEK, DOWNLINK_HZ_DEFAULT,
                         FingerSeqSweepWorker, FingerSweepParams,
                         LM_HOMING_RESET, LM_HOMING_START)
from utils.comm_diag import record_form2_stream_tick
from utils.downlink_guard import DownlinkGuard
from utils.motion_file import (build_motion_file_payload, normalize_hand_side,
                               parse_motion_file_data, save_motion_json)
from utils.ruckig_interpolator import RuckigJointInterpolator
from downlink_policy import OnlineUsbController, v2_ranges
from v2_homing import V2Calibration, V2Homing

JOINTS = tuple(URDF_JOINT_MAP.values())
LIMITS = {name: JOINT_LIMITS[short] for short, name in URDF_JOINT_MAP.items()}
PERIOD = 1.0 / max(1.0, float(DOWNLINK_HZ_DEFAULT))
POSITION_TOLERANCE, SETTLE_TICKS, FLUSH_TIMEOUT = 1e-5, 3, 5.0
RESERVED_NAMES = {'_hand_side', 'hand_side', '_meta', '无动作'}


def validate_motion_data(data, *, allow_empty=False):
    """Validate old lists/current files before replacing the working sequence."""
    if isinstance(data, dict) and data.get('_model_id') in (
            'TsenseI_v3_0', 'TsenseI_v3_0_withid', 'TsenseI_v3_2', 'TsenseI_v3_2_withid'):
        raise ValueError('A3.0 / A3.2 动作不能用于 V2，请先切换模型')
    parsed = parse_motion_file_data(data)
    if parsed is None:
        raise ValueError('动作文件必须是 JSON 对象或旧版动作数组')
    motions, side = parsed
    if not motions and not allow_empty:
        raise ValueError('动作文件中没有动作')
    cleaned = {}
    for name, entry in motions.items():
        if not isinstance(name, str) or not name.strip() or name in RESERVED_NAMES or len(name) > 200:
            raise ValueError('动作名称无效或过长')
        cfg = {}
        for key, value in entry.items():
            if key == 'name':
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{name}: {key} 必须是有限数值')
            if key in ('_transition_s', '_interval_s') and value < 0:
                raise ValueError('过渡和间隔时间不能为负数')
            cfg[key] = value
        cleaned[name] = remap_motion_cfg_file_to_ui(cfg, hand_side=side)
    return cleaned, side


class _SweepTransport:
    """Fence the shared raw-frame sweep worker against stop/replacement races."""
    def __init__(self, owner, epoch):
        self.owner, self.epoch = owner, epoch

    @property
    def is_connected(self):
        with self.owner._lock:
            return self.epoch == self.owner._epoch and self.owner.connected and not self.owner._closed

    def _write_frame(self, frame, **kwargs):
        with self.owner._lock:
            if not self.is_connected:
                return False
            return self.owner.usb._write_frame(frame, **kwargs)

    def __getattr__(self, name):
        if name in ('_ser', '_rx_thread', '_rx_drain'):
            return getattr(self.owner.usb, name, None)
        raise AttributeError(name)


class V2Session:
    model_id = 'TsenseI_v2'

    def __init__(self, data_dir, side='left', *, usb=None, clock=None, start=True):
        self.data_dir = Path(data_dir)
        self.side = normalize_hand_side(side)
        self.usb = usb if usb is not None else OnlineUsbController()
        self.generation = uuid.uuid4().hex
        self._lock = threading.RLock()
        self._clock = clock if callable(clock) else (getattr(clock, 'monotonic', None) or time.perf_counter)
        self._closed, self._epoch = False, 0
        self._shutdown, self._wake = threading.Event(), threading.Event()
        self._worker, self._background = None, bool(start)
        self.pose = {name: default_joint_position(name) for name in JOINTS}
        self.live = dict(self.pose)
        self.motions, self.selected = {}, 0
        self.transition, self.interval = .5, 0.
        self.sync = self.realtime = self.loop = self.log_hex = False
        self.name, self.dialog, self.calibration = '', None, None
        self._calibration_token = None
        self.homing_iq = float(DEFAULT_HOMING_IQ_SEEK)
        self.homing = V2Homing(self.usb, clock=self._clock)
        self.message = 'V2 · 21关节 · 浏览器本机串口 · 等待操作'
        self.playback_timing = self._task = self._sweep = None
        self._read_preference, self._remember_read = None, False
        self._realtime_target, self._realtime_flush, self._realtime_settled = None, False, 0
        self._interpolator, self._guard = RuckigJointInterpolator(), DownlinkGuard()
        self._last_success_hw = self._last_success_ui = self._last_success_ts = None
        self._seeded = False
        if self.path.is_file():
            self.motions, self.side = validate_motion_data(json.loads(self.path.read_text(encoding='utf-8-sig')), allow_empty=True)
            self._select(0)
        if start:
            self._worker = threading.Thread(target=self._run, name='V2Motion500Hz', daemon=True)
            self._worker.start()

    @property
    def path(self):
        return self.data_dir / 'motions.json'

    @property
    def connected(self):
        return bool(self.usb.is_connected)

    @property
    def playing(self):
        return bool(self._task and self._task['kind'] == 'sequence')

    @property
    def _can_send(self):
        return self.connected and bool(self.usb.should_process_downlink) and not self._closed

    def spec(self, key, kind, **values):
        return {'id': f'{self.generation}:{key}', 'kind': kind, 'enabled': True, 'tip': '', **values}

    def snapshot(self):
        with self._lock:
            busy = self.homing.active or self.calibration is not None
            names = list(self.motions)
            button = lambda key, text, enabled=True: self.spec(key, 'button', text=text, enabled=enabled)
            check = lambda key, text, value: self.spec(key, 'check', text=text, value=value, enabled=not busy)
            sweep = self._sweep is not None and self._sweep.is_running
            controls = {
                'handSide': self.spec('handSide', 'select', value=int(self.side == 'right'), options=['左手', '右手']),
                'choosemotion': self.spec('choosemotion', 'select', value=self.selected, options=names or ['无动作'], enabled=bool(names) and not busy),
                'savemotion': button('savemotion', '保存动作', not busy),
                'loadmotion': button('loadmotion', '加载选定动作', bool(names) and not busy),
                'deletemotion': button('deletemotion', '删除选定动作', bool(names) and not busy),
                'motionstreamplay': button('motionstreamplay', '停止' if self.playing else '动作序列播放', bool(names) and not busy),
                'motiondirectlysend': button('motiondirectlysend', '停止往返' if sweep else '五指往返', self.connected and not busy),
                'allzero': button('allzero', '全部电机回零', self.connected and not busy),
                'allreset': button('allreset', '全部电机复位', self.connected),
                'calibrationRotateMotor': button('calibrationRotateMotor', '旋转电机校准', self.connected and not busy),
                'oneKeyZero': button('oneKeyZero', self.homing.status if self.homing.active else '一键找零', self.connected and self.calibration is None),
                'transition': self.spec('transition', 'number', value=self.transition, min=.001, max=5., step=.001, enabled=not busy),
                'interval': self.spec('interval', 'number', value=self.interval, min=0., max=30., step=.001, enabled=not busy),
                'motionStreamLoop': check('motionStreamLoop', '循环播放', self.loop),
                'isConnectOffline': check('isConnectOffline', '动作同步下位机', self.sync),
                'realtimeJointControl': check('realtimeJointControl', '实时关节控制', self.realtime),
                'openSendLog': check('openSendLog', '显示下发HEX', self.log_hex),
            }
            joints = [self.spec(name, 'number', name=name, short=short, value=self.pose[name],
                                min=LIMITS[name][0], max=LIMITS[name][1], step=.001,
                                enabled=not busy and self._task is None and not sweep)
                      for short, name in URDF_JOINT_MAP.items()]
            ranges = v2_ranges(self.side, joints)
            for joint in joints:
                joint['downlink'] = ranges[joint['name']]
            return {'model_id': self.model_id, 'side': self.side,
                    'urdf': f'/assets/hands/TsenseI_hand/urdf/TsenseI__hand_{self.side}_v1.urdf',
                    'controls': controls, 'joints': joints, 'dialogs': self._dialogs(),
                    'pose': convert_ui_to_display(self.pose, hand_side=self.side),
                    'connected': self.connected, 'playing': self.playing, 'homing': self.homing.active,
                    'count': len(names), 'file': self.path.name,
                    'playback_timing': dict(self.playback_timing) if self.playback_timing else None,
                    'message': self.message}

    def _dialogs(self):
        dialogs = []
        if self.calibration:
            c = self.calibration
            prefix = f'calibration:{self._calibration_token}:'
            spec = lambda key, kind, **kw: self.spec(prefix + key, kind, **kw)
            items = [spec(key, 'label', text=text) for key, text in c.info().items()]
            items += [spec('calibrationPosition', 'number', value=c.position, min=-3600., max=3600., step=1.),
                      spec('calibrationSend', 'button', text='发送位置'),
                      spec('calibrationEnable', 'button', text='使能'),
                      spec('calibrationDisable', 'button', text='断电'),
                      spec('calibrationPrevious', 'button', text='上一个', enabled=c.index > 0),
                      spec('calibrationNext', 'button', text='下一个', enabled=c.index < 6),
                      spec('calibrationZero', 'button', text='已完成' if c.completed else '确认回零', enabled=not c.completed),
                      spec('calibrationClose', 'button', text='关闭')]
            dialogs.append({'id': prefix, 'title': '旋转电机校准', 'modal': False, 'items': items})
        if self.dialog:
            d = self.dialog
            prefix = f'dialog:{d["id"]}:'
            spec = lambda key, kind, **kw: self.spec(prefix + key, kind, **kw)
            items = [spec('dialogInfo', 'label', text=d['text'])]
            if d['action'] == 'save':
                items.append(spec('motionName', 'text', value=self.name))
            elif d['action'] in ('allzero', 'home'):
                items += [spec('iqLabel', 'label', text='iq_seek'),
                          spec('homingIq', 'number', value=self.homing_iq, min=0., max=5000., step=50.)]
            elif d['action'] == 'realtime':
                items.append(spec('rememberRead', 'check', text='记住我的选择（本次运行）', value=self._remember_read))
            items.append(spec('confirm', 'button', text='是' if d['action'] == 'realtime' else '确认'))
            if d['action'] == 'realtime':
                items.append(spec('no', 'button', text='否'))
            if d['action'] not in ('info', 'realtime_fallback'):
                items.append(spec('cancel', 'button', text='取消'))
            dialogs.append({'id': d['id'], 'title': d['title'], 'modal': True, 'items': items})
        return dialogs

    def ask(self, action, title, text, **details):
        self.dialog = {'id': uuid.uuid4().hex, 'action': action, 'title': title, 'text': text, **details}

    def _info(self, title, text):
        self.message = text
        self.ask('info', title, text)

    def _select(self, index, entry=None):
        self.selected = max(0, min(index, len(self.motions) - 1))
        if self.motions:
            self.transition, self.interval = self._timing(entry if entry is not None else list(self.motions.values())[self.selected])
            self.transition, self.interval = min(5., self.transition), min(30., self.interval)

    @staticmethod
    def _timing(entry):
        return max(.001, float(entry.get('_transition_s', .5))), max(0., float(entry.get('_interval_s', 0.)))

    @staticmethod
    def _normalize(entry):
        return {name: float(entry.get(name, default_joint_position(name))) for name in JOINTS}

    @staticmethod
    def _clamp(entry):
        return {name: min(hi, max(lo, float(entry.get(name, default_joint_position(name)))))
                for name, (lo, hi) in LIMITS.items()}

    def payload(self, motions=None, side=None):
        side = side or self.side
        return build_motion_file_payload({name: remap_motion_cfg_ui_to_file(cfg, hand_side=side)
                                          for name, cfg in (self.motions if motions is None else motions).items()}, side)

    def _persist(self, motions, side=None):
        self.data_dir.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_name(f'.{self.path.name}.{uuid.uuid4().hex}.tmp')
        try:
            save_motion_json(str(temporary), self.payload(motions, side))
            temporary.replace(self.path)
        finally:
            temporary.unlink(missing_ok=True)
        self.motions = motions

    def dispatch(self, action, body):
        with self._lock:
            if self._closed:
                raise ValueError('会话已关闭')
            if action == 'widget':
                return self.widget(body)
            if action == 'export':
                return self.payload()
            if action == 'import':
                motions, side = validate_motion_data(json.loads(body['text'].lstrip('\ufeff')))
                self.stop(reason='import_motion_file')
                if side != self.side and self.connected and not self.usb.send_hand_type(side):
                    raise ValueError('H7 左右手配置写入失败，未替换动作文件')
                self._persist(motions, side)
                self.side = side
                self.generation = uuid.uuid4().hex
                self._select(0)
                self.message = f'已导入 {len(motions)} 个动作'
            elif action == 'stop':
                self.stop(reason=body.get('reason', 'user_stop'))
            elif action == 'disconnect':
                self.stop(reason='disconnect')
                self.usb.disconnect()
            elif action == 'connect':
                self.stop(reason='connect')
                if not self.usb.connect_port(body['port']) or not self.usb.send_hand_type(self.side):
                    self.usb.disconnect()
                    raise ValueError('浏览器串口会话建立失败')
            else:
                raise ValueError('未知 V2 操作')
            return True

    def widget(self, body):
        with self._lock:
            ident = str(body.get('id', ''))
            if not ident.startswith(self.generation + ':'):
                raise ValueError('模型或手别已变化，请重试')
            state = self.snapshot()
            specs = list(state['controls'].values()) + state['joints'] + [item for d in state['dialogs'] for item in d['items']]
            spec = next((item for item in specs if item['id'] == ident), None)
            if not spec or not spec['enabled']:
                raise ValueError('此控件当前不可用')
            full_key = ident[len(self.generation) + 1:]
            if self.dialog and not full_key.startswith(f'dialog:{self.dialog["id"]}:'):
                raise ValueError('请先处理当前确认对话框')
            key = full_key.rsplit(':', 1)[-1]
            value = body.get('value')
            if spec['kind'] == 'number':
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not spec['min'] <= value <= spec['max']:
                    raise ValueError('数值超出关节/时间允许范围')
            elif spec['kind'] == 'check' and not isinstance(value, bool):
                raise ValueError('复选框值无效')
            elif spec['kind'] == 'select' and (type(value) is not int or not 0 <= value < len(spec['options'])):
                raise ValueError('选项不存在')
            elif spec['kind'] == 'text' and (not isinstance(value, str) or len(value) > 200 or value.strip() in RESERVED_NAMES):
                raise ValueError('动作名称无效或过长')
            if key in JOINTS:
                self.pose[key] = round(float(value), 3)
                if self.realtime and self._can_send:
                    self._realtime_target = dict(self.pose)
                    self._realtime_flush = bool(body.get('commit', True))
                    self._realtime_settled = 0
                    self._wake.set()
            elif key in ('transition', 'interval'):
                setattr(self, key, float(value))
            elif key == 'choosemotion':
                self._select(value)
            elif key == 'handSide':
                side = ('left', 'right')[value]
                if side != self.side:
                    restore_realtime, restore_sync = self.realtime, self.sync
                    self.stop(reason='hand_side_changed')
                    if self.connected and not self.usb.send_hand_type(side):
                        raise ValueError('H7 左右手配置写入失败')
                    self.side = side
                    self._last_success_hw = self._last_success_ui = None
                    self._seeded = False
                    self.generation = uuid.uuid4().hex
                    # The previous job and sweep epoch stay cancelled. Restore
                    # the user's control mode only after the new hand type was
                    # accepted, with a fresh planner for the current UI pose.
                    self.sync = restore_sync
                    if restore_realtime and self.connected:
                        self._enable_realtime(False)
            elif key == 'motionStreamLoop':
                self.loop = value
            elif key == 'openSendLog':
                self.log_hex = value
                self.usb.send_hex_log_enabled = value
            elif key == 'isConnectOffline':
                self._cancel_motion()
                self.realtime = False
                self.sync = value
            elif key == 'realtimeJointControl':
                self._cancel_motion()
                self.sync = self.realtime = False
                if value:
                    self._require_connection()
                    if self._read_preference is None:
                        self._remember_read = False
                        self.ask('realtime', '实时关节控制', '是否先从下位机读取当前关节角度并同步到模型？\n选“是”：读遥测对齐滑块。\n选“否”：用最近下发/显示姿态对齐，避免单关节拖动导致整手突跳。')
                    else:
                        self._enable_realtime(self._read_preference)
            elif key == 'savemotion':
                self.name = ''
                self.ask('save', '保存动作', '请输入动作名称:')
            elif key == 'motionName':
                self.name = value
            elif key == 'homingIq':
                self.homing_iq = float(value)
            elif key == 'rememberRead':
                self._remember_read = value
            elif key == 'deletemotion':
                name = list(self.motions)[self.selected]
                self.ask('delete', '确认删除', f'确定要删除动作 "{name}" 吗？此操作不可撤销。', name=name)
            elif key == 'loadmotion':
                entry = list(self.motions.values())[self.selected]
                target = self._clamp(self._normalize(entry))
                self._cancel_motion()
                if self.sync and self._can_send:
                    name = list(self.motions)[self.selected]
                    self._start_timeline([name], single_target=target, transition=self.transition)
                else:
                    self.pose = self.live = dict(target)
                    # Loading a pose while realtime is selected is display-only.
                    self._realtime_target = None
            elif key == 'motionstreamplay':
                if self.playing:
                    self._cancel_motion()
                else:
                    self._start_timeline(list(self.motions)[self.selected:])
            elif key == 'motiondirectlysend':
                if self._sweep and self._sweep.is_running:
                    self._cancel_motion()
                else:
                    self._require_connection()
                    self.stop(reason='finger_sweep')
                    self._sweep = FingerSeqSweepWorker(_SweepTransport(self, self._epoch))
                    self._sweep.params = FingerSweepParams(pos_start=0., pos_end=-16., period_s=4., send_hz=DOWNLINK_HZ_DEFAULT, loop=False)
                    self._sweep.start()
            elif key == 'allzero':
                self.ask('allzero', '确认回零', '将向全部 14 台直线电机发送机械寻零指令，确认执行？')
            elif key == 'allreset':
                self._require_connection()
                self.stop(reason='reset_homing')
                if not self.usb.send_all_lm_homing(LM_HOMING_RESET, 0.):
                    raise ValueError('复位指令未能发送，请检查串口连接')
            elif key == 'oneKeyZero':
                if self.homing.active:
                    self.stop(reason='手动停止找零')
                elif self.playing:
                    raise ValueError('请先停止动作序列播放，再执行一键找零')
                else:
                    self.ask('home', '确认一键找零', '将全部直线电机机械找零，固定延时 1 秒，再按 SPI2 M1/M0、SPI1 M1/M0、SPI4 M0/M1 顺序逐轴找零。确认机械活动空间与 iq_seek 后继续。')
            elif key == 'calibrationRotateMotor':
                self.ask('calibration', '确认校准', '将依次校准 SPI0 M0/M1 → SPI1 M0/M1 → SPI2 M0 → SPI3 M0/M1。确认开始？')
            elif key.startswith('calibration'):
                self._calibration_action(key, value)
            elif key == 'cancel':
                self.dialog = None
            elif key in ('confirm', 'no'):
                self._answer(key == 'confirm')
            else:
                raise ValueError('未知 V2 控件')
            return True

    def _answer(self, affirmative):
        d = self.dialog
        self.dialog = None
        action = d['action']
        if action == 'save':
            name = self.name.strip()
            if not name:
                raise ValueError('动作名称不能为空')
            if name in self.motions:
                self.ask('overwrite', '动作已存在', f'动作 "{name}" 已存在，是否覆盖？', name=name)
            else:
                self._save_motion(name)
        elif action == 'overwrite':
            self._save_motion(d['name'])
        elif action == 'delete':
            self._cancel_motion()
            motions = dict(self.motions)
            motions.pop(d['name'], None)
            self._persist(motions)
            self._select(len(motions) - 1)
            if motions:
                self.pose = self.live = self._clamp(self._normalize(list(motions.values())[-1]))
        elif action == 'realtime':
            if self._remember_read:
                self._read_preference = affirmative
            self._enable_realtime(affirmative)
        elif action == 'realtime_fallback':
            self._enable_realtime(False)
        elif action == 'allzero':
            self._require_connection()
            self.stop(reason='all_linear_homing')
            if not self.usb.send_all_lm_homing(LM_HOMING_START, self.homing_iq):
                raise ValueError('回零指令未能发送，请检查串口连接')
        elif action == 'home':
            self._require_connection()
            self.stop(reason='onekey_homing')
            self.homing.usb = self.usb
            self.homing.start(side=self.side, iq_seek=self.homing_iq)
        elif action == 'calibration':
            self._require_connection()
            self.stop(reason='rotary_calibration')
            self.calibration = V2Calibration(self.usb)
            self._calibration_token = uuid.uuid4().hex
        elif action != 'info':
            raise ValueError('确认操作已变化')

    def _save_motion(self, name):
        old_order = list(self.motions)
        selected = old_order[self.selected] if old_order else None
        order = [key for key in old_order if key != name]
        index = len(order)
        if selected == name:
            index = old_order.index(name)
        elif selected in order:
            index = order.index(selected) + 1
        order.insert(index, name)
        entry = {**self.pose, '_transition_s': self.transition, '_interval_s': self.interval}
        motions = {key: entry if key == name else self.motions[key] for key in order}
        self._persist(motions)
        self._select(order.index(name))
        self._info('保存成功', f'动作 "{name}" 已保存。')

    def _calibration_action(self, key, value):
        c = self.calibration
        if key == 'calibrationClose':
            self.calibration = None
            return
        self._require_connection()
        if key == 'calibrationPosition':
            c.position = float(value)
        elif key == 'calibrationSend':
            c.send_position(c.position)
        elif key == 'calibrationEnable':
            c.enable()
        elif key == 'calibrationDisable':
            c.disable()
        elif key == 'calibrationPrevious':
            c.previous()
        elif key == 'calibrationNext':
            c.next()
        elif key == 'calibrationZero':
            c.confirm_zero()
            if c.completed:
                self._info('校准完成', '全部旋转电机回零指令已依次下发完成。可点击「关闭」退出。')

    def _require_connection(self):
        if not self.connected:
            raise ValueError('请先连接下位机串口')

    def _read_seed(self, read, *, require_telemetry=False):
        seed = None
        if read and self._can_send:
            try:
                seed = self.usb.read_latest_joint_angles(hand_side=self.side)
            except Exception:
                pass
        if require_telemetry and not seed:
            return None
        if not seed:
            seed = self._last_success_ui or self.pose
        normalized = {key.replace('right_hand_', 'left_hand_', 1): value for key, value in seed.items()}
        return self._clamp({**self.pose, **normalized})

    def _enable_realtime(self, read):
        self._require_connection()
        seed = self._read_seed(read, require_telemetry=read)
        if seed is None:
            self.ask('realtime_fallback', '读取失败', '未读取到有效遥测关节角。\n将改用最近下发/显示姿态对齐。')
            return
        self.pose = seed
        self._seed(self.pose)
        self.realtime, self.sync = True, False
        self._realtime_target = dict(self.pose)
        self._realtime_settled = 0
        self._next_realtime_tick = self._clock()
        self._wake.set()

    def _seed(self, pose):
        pose = dict(pose)
        self._interpolator.reset()
        self._interpolator.seed_ramp(pose, pose)
        self._interpolator.pop_reset_flag()
        self._guard.reset()
        hw = convert_ui_to_hardware(pose, hand_side=self.side)
        now = self._clock()
        self._guard.filter(hw, now=now)
        self._last_success_hw, self._last_success_ui, self._last_success_ts = dict(hw), dict(pose), now
        self.live = pose
        self._seeded = True

    def _send_step(self, target, *, source, stream):
        if not self._can_send:
            return False, dict(self.live)
        if not self._seeded:
            self._seed(self.live)
        self._interpolator.push(target)
        output = self._interpolator.sample(delta_time=PERIOD)
        self._interpolator.pop_reset_flag()
        if not output:
            return False, dict(self.live)
        hardware = convert_ui_to_hardware(output, hand_side=self.side)
        now = self._clock()
        safe = self._guard.filter(hardware, now=now, expected_jump=False, ruckig_faults=self._interpolator.last_faults)
        if any(abs(safe.get(name, value) - value) > 1e-12 for name, value in hardware.items()):
            output = convert_hardware_to_ui(safe, hand_side=self.side)
            self._interpolator.sync_current_pose(output)
        ok = bool(self.usb.send_joint_angles(safe, source=source, stream=stream, hand_side=self.side))
        if ok:
            self._last_success_hw, self._last_success_ui, self._last_success_ts = dict(safe), dict(output), now
        else:
            # Preserve both baselines: the H7 0.5-degree floor is not invertible.
            self._guard.reset()
            self._guard.filter(self._last_success_hw, now=self._last_success_ts)
            output = dict(self._last_success_ui)
            self._interpolator.sync_current_pose(output)
        self.live = dict(output)
        record_form2_stream_tick(sent=ok)
        return ok, output

    def _cancel_motion(self):
        self._epoch += 1
        if self._task:
            stats = self._task['stats']
            stats.update(stopped=True, total_s=self._clock() - self._task['cycle_start'])
            self.playback_timing = dict(stats)
        self._task = None
        self._realtime_target = None
        self._realtime_settled = 0
        if self._sweep:
            self._sweep.stop()
        self._wake.set()

    def stop(self, reason='user_stop'):
        with self._lock:
            self._cancel_motion()
            self.sync = self.realtime = False
            self.homing.stop(reason)
            self.homing.result = None
            self.calibration = self.dialog = None
            self.message = f'已停止：{reason}'

    def close(self):
        with self._lock:
            self.stop(reason='session_closed')
            self._closed = True
            self._shutdown.set()
            self._wake.set()
            sweep, worker = self._sweep, self._worker
        if worker and worker is not threading.current_thread():
            worker.join(timeout=2.)
        if sweep:
            sweep.join(timeout=2.)
        self.usb.disconnect()

    def _start_timeline(self, names, *, single_target=None, transition=None):
        if not names:
            raise ValueError('当前没有动作，请先加载或保存动作')
        self._cancel_motion()
        self.realtime = False
        self.pose = self._read_seed(True) if self.sync and self._can_send else dict(self.pose)
        self._seed(self.pose)
        entries = [(name, dict(self.motions[name])) for name in names]
        if single_target is not None:
            entries = [(names[0], {**single_target, '_transition_s': transition, '_interval_s': 0.})]
        self._task = {'kind': 'load' if single_target is not None else 'sequence', 'entries': entries,
                      'downlink': bool(self.sync and self._can_send),
                      'selection': {name: index for index, name in enumerate(self.motions)},
                      'entry_map': dict(entries)}
        self._prepare_cycle(self._task)
        self._wake.set()

    def _prepare_cycle(self, task):
        prepare = self._clock()
        previous = dict(self.live)
        total = 0.
        segments = []
        for name, entry in task['entries']:
            target = self._clamp(self._normalize(entry))
            transition, hold = self._timing(entry)
            begin = total
            total += transition + hold
            segments.append((begin, total, transition, previous, target, name))
            previous = target
        now = self._clock()
        task.update(segments=segments, phase='timeline', index=0, cycle_start=now,
                    next_tick=now + min(PERIOD, total), deadline=now + total,
                    stats={'nominal_s': total, 'prepare_s': now - prepare, 'timeline_s': 0.,
                           'ticks': 0, 'sent': 0, 'missed_ticks': 0, 'stopped': False})
        first_name, first_entry = task['entries'][0]
        self._select(task['selection'][first_name], first_entry)
        task['selected_name'] = first_name

    def _motion_step(self, now):
        if self._closed:
            return
        task = self._task
        if task:
            if task['downlink'] and not self._can_send:
                self.stop(reason='device_disconnected')
                return
            if now + 1e-12 < task['next_tick']:
                return
            if task['phase'] == 'settle':
                target = task['segments'][-1][4]
                if now >= task['settle_deadline']:
                    self.message = 'Ruckig 终态收敛超时；保持最后安全姿态，未直接跳到目标'
                    self._finish_task(task, now)
                    return
                ok, output = self._send_step(target, source='V2Session/动作序列(Ruckig最终态)', stream=False)
                self.pose = dict(output)
                error = max(abs(output[name] - target[name]) for name in JOINTS)
                task['settled'] = task['settled'] + 1 if ok and error <= POSITION_TOLERANCE else 0
                if task['settled'] >= SETTLE_TICKS:
                    self._finish_task(task, self._clock())
                else:
                    task['next_tick'] = self._clock() + PERIOD
                return
            elapsed = now - task['cycle_start']
            segments = task['segments']
            index = task['index']
            while index + 1 < len(segments) and elapsed >= segments[index][1]:
                index += 1
            task['index'] = index
            begin, _, duration, origin, target, name = segments[index]
            fraction = min(1., max(0., (elapsed - begin) / duration))
            blend = (1. - math.cos(math.pi * fraction)) * .5
            desired = {joint: origin[joint] + (target[joint] - origin[joint]) * blend for joint in JOINTS}
            if task['downlink']:
                ok, output = self._send_step(desired, source='V2Session/动作序列(Ruckig 500Hz)', stream=True)
            else:
                ok, output = False, desired
                self.live = dict(output)
            self.pose = dict(output)
            if name != task['selected_name']:
                self._select(task['selection'][name], task['entry_map'][name])
                task['selected_name'] = name
            stats = task['stats']
            stats['ticks'] += 1
            stats['sent'] += int(ok)
            after_send = self._clock()
            stats['timeline_s'] = after_send - task['cycle_start']
            if now >= task['deadline']:
                self.playback_timing = dict(stats)
                if self.loop and task['kind'] == 'sequence':
                    self._prepare_cycle(task)
                elif task['downlink']:
                    task.update(phase='settle', settle_start=after_send, settle_deadline=after_send + FLUSH_TIMEOUT,
                                settled=0, next_tick=after_send + PERIOD)
                else:
                    self._finish_task(task, after_send)
                return
            next_tick = task['next_tick'] + PERIOD
            if next_tick < after_send:
                missed = math.floor((after_send - next_tick) / PERIOD) + 1
                stats['missed_ticks'] += missed
                next_tick += missed * PERIOD
            task['next_tick'] = min(next_tick, task['deadline'])
        elif self.realtime and self._realtime_target is not None and self._realtime_settled < SETTLE_TICKS:
            if not self._can_send:
                self.stop(reason='device_disconnected')
                return
            if now + 1e-12 < self._next_realtime_tick:
                return
            ok, output = self._send_step(self._realtime_target, source='V2Session/实时关节控制(Ruckig终态)', stream=not self._realtime_flush)
            error = max(abs(output[name] - self._realtime_target[name]) for name in JOINTS)
            self._realtime_settled = self._realtime_settled + 1 if ok and error <= POSITION_TOLERANCE else 0
            self._next_realtime_tick = max(now + PERIOD, self._clock())

    def _finish_task(self, task, now):
        task['stats'].update(settle_s=now - task.get('settle_start', now), total_s=now - task['cycle_start'], stopped=False)
        self.playback_timing = dict(task['stats'])
        self._task = None

    def tick(self):
        with self._lock:
            if self._closed:
                return
            if not self.connected and (self.realtime or self.homing.active or self.calibration or (self._task and self._task['downlink'])):
                self.stop(reason='device_disconnected')
                return
            self.homing.tick()
            if self.homing.result:
                result, self.homing.result = self.homing.result, None
                self._info(result['title'], result['text'])
            if not self._background:
                self._motion_step(self._clock())

    def _run(self):
        while not self._shutdown.is_set():
            with self._lock:
                try:
                    self._motion_step(self._clock())
                except Exception as exc:
                    self.stop(reason='motion_error')
                    self.message = f'运动已停止：{exc}'
                task = self._task
                active = bool(task or (self.realtime and self._realtime_target is not None and self._realtime_settled < SETTLE_TICKS))
                deadline = task['next_tick'] if task else getattr(self, '_next_realtime_tick', self._clock() + PERIOD)
                delay = max(0., deadline - self._clock())
            self._wake.wait(timeout=min(.05, delay) if active else .05)
            self._wake.clear()
