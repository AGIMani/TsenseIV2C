"""One Manus SDK owner, shared by both hands and all browser sessions."""
from copy import deepcopy
import importlib
from pathlib import Path
import sys
import threading
import time
import uuid

import numpy as np

STALE_AFTER_MS = 200
SOURCE = 'manus_glove'
MODULE_DIR = Path(__file__).resolve().parents[2] / 'vendor/manussdk/build/python'
# The native binding has one process-global callback owner, even across Applications.
SDK_OWNER = threading.Lock()
FINGER_SPEC = [(5, joint) for joint in (1, 2, 4, 5)] + [
    (chain, joint) for chain in (6, 7, 8, 9) for joint in (2, 3, 4, 5)]


def _uint(value, name, bits=64):
    if type(value) is not int or not 0 <= value < 1 << bits:
        raise ValueError(f'Manus {name}必须为 uint{bits}')
    return value


def _timestamp(value):
    if isinstance(value, str):
        if (not value.isascii() or not value.isdecimal() or not 1 <= len(value) <= 20
                or str(int(value)) != value):
            raise ValueError('Manus 压缩时间戳无效')
        value = int(value)
    return str(_uint(value, '压缩时间戳'))


def mediapipe_rows(nodes):
    """Map semantic SDK nodes to wrist, thumb, index, middle, ring, pinky."""
    if not 21 <= len(nodes) <= 256:
        raise ValueError('Manus 骨架缺少完整节点元数据')
    ids = {_uint(node.node_id, '节点 ID', 32) for node in nodes}
    for node in nodes:
        _uint(node.parent_id, '父节点 ID', 32)
        _uint(node.chain_type, '节点链类型', 32)
        _uint(node.finger_joint_type, '指关节类型', 32)
    roots = [i for i, node in enumerate(nodes) if
             int(node.parent_id) == int(node.node_id) or int(node.parent_id) not in ids]
    hand_roots = [i for i in roots if int(nodes[i].chain_type) == 13]
    roots = hand_roots or roots
    if len(ids) != len(nodes) or len(roots) != 1:
        raise ValueError('Manus 骨架缺少唯一腕部根节点')
    parents = {node.node_id: node.parent_id for node in nodes}
    root_id = nodes[roots[0]].node_id
    for node_id in ids:
        visited = set()
        while node_id != root_id:
            if node_id in visited or node_id not in parents:
                raise ValueError('Manus 骨架父子关系不完整或存在循环')
            visited.add(node_id)
            node_id = parents[node_id]
    lookup = {}
    for row, node in enumerate(nodes):
        key = (int(node.chain_type), int(node.finger_joint_type))
        # SDK manuals call the thumb's third node either Intermediate or Distal.
        if key == (5, 3):
            key = (5, 4)
        if key in FINGER_SPEC:
            if key in lookup:
                raise ValueError(f'Manus 骨架关节重复：{key}')
            lookup[key] = row
    missing = [key for key in FINGER_SPEC if key not in lookup]
    if missing:
        raise ValueError(f'Manus 骨架缺少手指关节：{missing}')
    return [roots[0]] + [lookup[key] for key in FINGER_SPEC]


def hand_frame(snapshot, nodes, side):
    """Global metre positions -> wrist-relative 21 points; keep handedness."""
    if side not in ('left', 'right'):
        raise ValueError('Manus 手别无效')
    skeleton = np.asarray(snapshot['skeleton'], dtype=float)
    if skeleton.shape != (len(nodes), 10) or not np.isfinite(skeleton).all():
        raise ValueError('Manus 需要有限的 N×10 骨架与对应节点信息')
    semantic_rows = mediapipe_rows(nodes)
    # The SDK makes no promise that SkeletonNode[] and NodeInfo[] share order.
    # The binding copies IDs atomically with its positions, before releasing its
    # callback lock. Older bindings must be rebuilt, never guessed by row index.
    node_ids = snapshot.get('node_ids')
    if not isinstance(node_ids, (list, tuple)) or len(node_ids) != len(nodes):
        raise ValueError('Manus 绑定缺少原子节点 ID，请重新编译 vendor/manussdk')
    id_to_row = {_uint(node_id, '骨架节点 ID', 32): row for row, node_id in enumerate(node_ids)}
    if len(id_to_row) != len(nodes) or set(id_to_row) != {node.node_id for node in nodes}:
        raise ValueError('Manus 骨架 ID 与节点元数据不匹配或重复')
    for node in nodes:
        node_side = getattr(node, 'side', 0)
        if type(node_side) is not int or node_side not in (0, 3, 1 if side == 'left' else 2):
            raise ValueError('Manus 节点手别与所选手套不一致')
    rows = [id_to_row[nodes[row].node_id] for row in semantic_rows]
    points = skeleton[rows, :3].copy()
    points -= points[0]
    if np.max(np.abs(points)) > 10:
        raise ValueError('Manus 手部相对坐标超出 10 米范围')
    raw = deepcopy({key: value for key, value in snapshot.items() if key != 'skeleton'})
    raw['source_timestamp_raw'] = _timestamp(snapshot['source_timestamp_raw'])
    raw.update(schema='manus.raw_skeleton.v1', side=side, units='m',
        coordinate_space='manus_world', skeleton=skeleton.tolist(), joint_mapping=rows,
        node_info=[dict(**{key: int(getattr(node, key)) for key in
                    ('node_id', 'parent_id', 'chain_type', 'finger_joint_type')},
                    side=int(getattr(node, 'side', 0))) for node in nodes])
    hand = dict(world=points.tolist(), normalized=points.tolist(), label=side.title(),
        source=SOURCE, coordinate_space='manus_wrist_relative', normalized_is_image=False)
    return raw, hand


def load_sdk(module_dir=None):
    directory = str(Path(module_dir or MODULE_DIR).expanduser().resolve())
    if directory not in sys.path:
        sys.path.insert(0, directory)
    try:
        return importlib.import_module('manus_glove')
    except (ImportError, OSError) as exc:
        raise ImportError(f'请用运行服务的 Python 执行 scripts/build_manus.sh；Manus SDK 加载失败：{exc}') from exc


class ManusSource:
    def __init__(self, *, mode='integrated', module_dir=None, left_calibration=None,
                 right_calibration=None, connect_timeout=10, clock=time.monotonic,
                 sdk_loader=None, retry_delay=1.):
        if mode not in ('integrated', 'local', 'remote'):
            raise ValueError('Manus mode 必须为 integrated、local 或 remote')
        if type(connect_timeout) is not int or not 1 <= connect_timeout <= 60:
            raise ValueError('Manus connect_timeout 必须在 1..60 秒')
        self.mode, self.connect_timeout = mode, connect_timeout
        self.calibrations = {side: Path(path).expanduser().resolve() for side, path in
            (('left', left_calibration), ('right', right_calibration)) if path is not None}
        for path in self.calibrations.values():
            if not path.is_file():
                raise ValueError(f'Manus 标定文件不存在：{path}')
        self.clock, self.retry_delay = clock, retry_delay
        self.sdk_loader = sdk_loader or (lambda: load_sdk(module_dir))
        self.lock = threading.RLock()
        self.worker = self.on_change = None
        self.stream_id = None
        self.revision = 0
        self.sides = {side: self._empty() for side in ('left', 'right')}

    @staticmethod
    def _empty():
        return dict(revision=0, epoch=0, sequence=-1, glove_id=0, observed_at=None,
                    raw=None, hand=None, event='waiting', error=None, sequences={})

    def _notify(self):
        if self.on_change:
            self.on_change()

    def _current(self, worker):
        return self.worker is worker and not worker['stop'].is_set()

    def _clear(self, worker, event, error=None, side=None):
        changed = False
        with self.lock:
            if not self._current(worker):
                return
            for key in (side,) if side else ('left', 'right'):
                state = self.sides[key]
                if state['event'] == event and state['error'] == error and state['hand'] is None:
                    continue
                self.revision += 1
                state.update(hand=None, raw=None, observed_at=None, event=event, error=error,
                             revision=self.revision, epoch=state['epoch']+1)
                changed = True
        if changed:
            self._notify()

    def _publish(self, worker, side, snapshot, nodes):
        sequence = _uint(snapshot['sequence'], '回调序号')
        arrival = _uint(snapshot['host_arrival_ns'], '回调时间')
        glove_id = _uint(snapshot['glove_id'], '手套 ID', 32)
        now = self.clock()
        if snapshot['valid'] is not True or sequence <= 0 or not glove_id or arrival <= 0 or arrival > int(now * 1e9):
            raise ValueError('Manus 骨架或回调时间无效')
        with self.lock:
            state = self.sides[side]
            if not self._current(worker) or sequence <= state['sequences'].get(glove_id, -1):
                return
        raw, hand = hand_frame(snapshot, nodes, side)
        with self.lock:
            if not self._current(worker):
                return
            state = self.sides[side]
            # Each new SDK callback has its own host arrival, even when the
            # glove repeats its packed publish time.
            state['sequences'][glove_id] = sequence
            if (state['hand'] is None or state['glove_id'] != snapshot['glove_id'] or
                    (now - state['observed_at']) * 1000 >= STALE_AFTER_MS):
                state['epoch'] += 1
            self.revision += 1
            state.update(revision=self.revision, sequence=sequence, glove_id=snapshot['glove_id'],
                observed_at=arrival / 1e9, raw=raw, hand=hand, event='tracking', error=None)
        self._notify()

    def _run(self, worker):
        while not worker['stop'].is_set():
            if not SDK_OWNER.acquire(timeout=.1):
                continue
            glove = None
            try:
                if not self._current(worker):
                    return
                sdk = self.sdk_loader()
                if not self._current(worker):
                    return
                glove = sdk.ManusGlove()
                with self.lock:
                    worker['glove'] = glove
                if not hasattr(glove, 'get_raw_skeleton_snapshot'):
                    raise ImportError('Manus 绑定缺少原子帧接口，请重新编译 vendor/manussdk')
                self._clear(worker, 'connecting')
                if not self._current(worker):
                    return
                if not glove.connect(mode=self.mode, world_coordinates=True, timeout_seconds=self.connect_timeout):
                    raise RuntimeError('Manus 连接失败，请检查 USB 接收器或 Manus Core')
                if not self._current(worker):
                    return
                if hasattr(glove, 'set_recording_enabled'):
                    glove.set_recording_enabled(False)
                with self.lock:
                    self.stream_id = uuid.uuid4().hex
                    for state in self.sides.values():
                        state['sequence'] = -1
                        state['sequences'].clear()
                mappings, calibrated, calibration_errors = {}, set(), {}
                self._clear(worker, 'connected')
                while not worker['stop'].is_set():
                    if not glove.is_connected():
                        raise RuntimeError('Manus SDK 已断开，正在重连')
                    for side in ('left', 'right'):
                        try:
                            if not self._current(worker):
                                break
                            glove_id = glove.get_glove_id(side)
                            if not self._current(worker):
                                break
                            if not glove_id:
                                mappings.pop(side, None)
                                calibrated = {key for key in calibrated if key[0] != side}
                                calibration_errors = {key: value for key, value in calibration_errors.items() if key[0] != side}
                                self._clear(worker, 'glove_missing', side=side)
                                continue
                            if side in self.calibrations and (side, glove_id) not in calibrated:
                                if (side, glove_id) in calibration_errors:
                                    raise RuntimeError(calibration_errors[(side, glove_id)])
                                # Never expose a cached pre-calibration sample after changing the model.
                                calibration = self.calibrations[side].read_bytes()
                                if not self._current(worker):
                                    break
                                result = glove.set_calibration(side, calibration)
                                if not self._current(worker):
                                    break
                                if result != 1:
                                    error = f'Manus {side} 标定加载失败，SDK code={result}；检查文件后重新开始输入'
                                    calibration_errors[(side, glove_id)] = error
                                    raise RuntimeError(error)
                                calibrated.add((side, glove_id))
                                with self.lock:
                                    cached = dict(glove.get_raw_skeleton_snapshot(side))
                                    sequence = _uint(cached['sequence'], '标定前回调序号')
                                    self.sides[side]['sequence'] = sequence
                                    self.sides[side]['sequences'][glove_id] = sequence
                                    self.sides[side]['glove_id'] = glove_id
                                mappings.pop(side, None)
                                self._clear(worker, 'calibrated', side=side)
                            snapshot = dict(glove.get_raw_skeleton_snapshot(side))
                            if not self._current(worker):
                                break
                            if not snapshot['valid'] or snapshot['glove_id'] != glove_id:
                                mappings.pop(side, None)
                                self._clear(worker, 'waiting_for_skeleton', side=side)
                                continue
                            # Node IDs travel with the coordinates. Reordering rows
                            # keeps the same hierarchy; a changed set needs a new one.
                            layout = (glove_id, frozenset(snapshot.get('node_ids') or ()))
                            if side not in mappings or mappings[side][0] != layout:
                                mappings[side] = (layout, glove.get_node_info(side))
                            self._publish(worker, side, snapshot, mappings[side][1])
                        except Exception as exc:
                            mappings.pop(side, None)
                            self._clear(worker, 'invalid_frame', str(exc), side=side)
                    worker['stop'].wait(.008)
            except (ImportError, OSError) as exc:
                self._clear(worker, 'sdk_unavailable', str(exc))
            except Exception as exc:
                self._clear(worker, 'sdk_error', str(exc))
            finally:
                try:
                    if glove is not None:
                        while True:
                            try:
                                glove.disconnect()
                                break
                            except Exception as exc:
                                self._clear(worker, 'disconnect_error', str(exc))
                                # Retain the global SDK owner until cleanup
                                # succeeds, including after session cancellation.
                                time.sleep(max(.01, self.retry_delay))
                        with self.lock:
                            worker['glove'] = None
                        glove = None
                finally:
                    SDK_OWNER.release()
            worker['stop'].wait(self.retry_delay)

    def _start(self):
        if self.worker is None or not self.worker['thread'].is_alive():
            worker = dict(stop=threading.Event())
            worker['thread'] = threading.Thread(target=self._run, args=(worker,),
                                                name='manus-input', daemon=True)
            self.worker = worker
            worker['thread'].start()

    def _snapshot(self, side):
        state = self.sides[side]
        age = None if state['observed_at'] is None else max(0., (self.clock()-state['observed_at'])*1000)
        stopped = self.worker is None or self.worker['stop'].is_set()
        fresh = not stopped and state['hand'] is not None and age is not None and age < STALE_AFTER_MS
        metadata = dict(source=SOURCE, source_joint_count=len(state['raw']['skeleton']) if state['raw'] else None,
            output_joint_count=21, units='m', native_units='m',
            coordinate_space='manus_world', output_coordinate_space='manus_wrist_relative',
            normalized_is_image=False, stream_id=self.stream_id, source_epoch=state['epoch'],
            sequence=state['sequence'], age_ms=age, stale_after_ms=STALE_AFTER_MS,
            tracking=fresh, tracking_valid=None, stale=not fresh, transport='manus_sdk', mode=self.mode,
            event='stopped' if stopped else state['event'], error=state['error'],
            raw_manus=deepcopy(state['raw']))
        return dict(source=SOURCE, revision=state['revision'], age_ms=age,
            hand=deepcopy(state['hand']) if fresh else None, tracking=fresh, connected=fresh,
            source_metadata=metadata, raw_manus=deepcopy(state['raw']))

    def latest(self, side):
        if side not in self.sides:
            raise ValueError('Manus 手别无效')
        with self.lock:
            self._start()
            return self._snapshot(side)

    def commit_if_current(self, side, snapshot, action):
        with self.lock:
            current = self._snapshot(side)
            meta = snapshot['source_metadata']
            if (self.worker is None or self.worker['stop'].is_set() or
                    meta['source_epoch'] != current['source_metadata']['source_epoch'] or
                    meta['stream_id'] != self.stream_id or
                    (snapshot['hand'] is not None) != current['tracking']):
                return False
            if snapshot['hand'] is not None:
                raw = snapshot.get('raw_manus')
                arrival = raw.get('host_arrival_ns') if isinstance(raw, dict) else None
                if (type(arrival) is not int or arrival <= 0 or
                        not 0 <= self.clock()-arrival/1e9 < STALE_AFTER_MS/1000):
                    return False
            action()
            return True

    def close(self, *, wait=True):
        with self.lock:
            worker = self.worker
            if worker is None:
                return
            worker['stop'].set()
            glove = worker.get('glove')
            cancel = getattr(glove, 'request_disconnect', None)
            if cancel is not None:
                cancel()
            for state in self.sides.values():
                self.revision += 1
                state.update(hand=None, raw=None, observed_at=None, event='stopped',
                             revision=self.revision, epoch=state['epoch']+1)
        self._notify()
        if wait and worker['thread'] is not threading.current_thread():
            worker['thread'].join(timeout=self.connect_timeout+5)
