"""Historical A3 protocol reference extracted from A3_USB_Downlink.pyw.
Only wire-format constants and pure functions are retained; no GUI or device I/O.
"""
import struct
from typing import Optional

SELF_MOTOR_NUM = 18
MIT_MOTOR_NUM = 2
DEV_TELEM = 3
FUN_HAND_POS_BATCH = 32
FUN_SELF_PWR = 33
FUN_SELF_HOMING = 34
FUN_SELF_POS_PID = 35
FUN_SELF_CUR_PID = 36
FUN_SELF_OPEN_SPIN = 37
HAND_BATCH_PAYLOAD_LEN = 100
HAND_BATCH_WIRE_LEN = 108
HAND_TELEM_HDR = struct.Struct('<HHII')
SELF_TELEM_SLOT = struct.Struct('<5f2h')
MIT_STATUS_BYTES = 20
MIT_POS_COUNTS = 2097152.0
MIT_GEAR_RATIO = 50.0
MIT_FAULT_BITS = ((0, '驱动'), (1, '过压'), (2, '欠压'), (3, '过流'), (4, '编码器'))

def pack_aa55(fun: int, payload: bytes=b'') -> bytes:
    plen = len(payload)
    hdr = bytes([170, 85]) + struct.pack('<HBB', plen, DEV_TELEM, fun)
    body = hdr + payload
    return body + struct.pack('<H', crc16_modbus(body))

def pack_self_pwr(bus: int, node: int, cmd: int, drv: int=3) -> bytes:
    """0x21 使能/断电。drv: 1=DRV1 2=DRV2 3=全部。"""
    return pack_aa55(FUN_SELF_PWR, bytes((bus & 255, node & 255, cmd & 255, drv & 255)))

def pack_self_homing(bus: int, node: int, cmd: int, iq_seek: float) -> bytes:
    """0x22 回零。iq_seek 为 Iq 码值（float32），不是安培。"""
    return pack_aa55(FUN_SELF_HOMING, struct.pack('<BBBB f 4x', bus, node, cmd, 0, iq_seek))

def pack_self_pos_pid(bus: int, node: int, kp: float, ki: float, kd: float) -> bytes:
    return pack_aa55(FUN_SELF_POS_PID, struct.pack('<BBBB 3f', bus, node, 0, 0, kp, ki, kd))

def pack_self_cur_pid(bus: int, node: int, id_kp: float, id_ki: float, id_kd: float, iq_kp: float, iq_ki: float, iq_kd: float) -> bytes:
    return pack_aa55(FUN_SELF_CUR_PID, struct.pack('<BBBB 6f', bus, node, 0, 0, id_kp, id_ki, id_kd, iq_kp, iq_ki, iq_kd))

def pack_self_open_spin(bus: int, node: int, cmd: int, freq_hz: float, half_s: float) -> bytes:
    """0x25 开环往复。cmd: 1=开始 2=停止。"""
    return pack_aa55(FUN_SELF_OPEN_SPIN, struct.pack('<BBBB 2f', bus, node, cmd, 0, freq_hz, half_s))

def _crc16_modbus_table() -> tuple[int, ...]:
    tab = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = crc >> 1 ^ 40961 if crc & 1 else crc >> 1
        tab.append(crc)
    return tuple(tab)
CRC16_MODBUS_TAB = _crc16_modbus_table()
HAND_BATCH_HDR = bytes([170, 85]) + struct.pack('<HBB', HAND_BATCH_PAYLOAD_LEN, DEV_TELEM, FUN_HAND_POS_BATCH)

def crc16_modbus(data: bytes | bytearray | memoryview) -> int:
    """Modbus CRC16，初值 0xFFFF，与 H7 tsensei_app_usb_crc16_modbus 一致。"""
    crc = 65535
    tab = CRC16_MODBUS_TAB
    for b in data:
        crc = crc >> 8 ^ tab[(crc ^ b) & 255]
    return crc & 65535

def pack_hand_batch_into(buf: bytearray, seq: int, self_pos: tuple[float, ...] | list[float], mit_pos_deg: tuple[float, ...] | list[float], mit_vel_rpm: tuple[float, ...] | list[float], mit_iq_ff_a: tuple[float, ...] | list[float]) -> None:
    """组 USB 0x20 整包到 108 字节缓冲，避免每帧新分配。"""
    if len(buf) < HAND_BATCH_WIRE_LEN:
        raise ValueError('buf 需要 108 字节')
    if len(self_pos) != SELF_MOTOR_NUM:
        raise ValueError(f'self_pos 需要 {SELF_MOTOR_NUM} 个')
    if len(mit_pos_deg) != MIT_MOTOR_NUM:
        raise ValueError('mit_pos_deg 需要 2 个')
    buf[0:6] = HAND_BATCH_HDR
    struct.pack_into('<B18f2f2f2f3x', buf, 6, seq & 255, *self_pos, *mit_pos_deg, *mit_vel_rpm, *mit_iq_ff_a)
    struct.pack_into('<H', buf, 106, crc16_modbus(memoryview(buf)[:106]))

def pack_hand_batch(seq: int, self_pos: list[float], mit_pos_deg: list[float], mit_vel_rpm: list[float], mit_iq_ff_a: list[float]) -> bytes:
    """组 USB 0x20 整包，返回 108 字节。"""
    buf = bytearray(HAND_BATCH_WIRE_LEN)
    pack_hand_batch_into(buf, seq, self_pos, mit_pos_deg, mit_vel_rpm, mit_iq_ff_a)
    return bytes(buf)

def decode_homing_or_theta(value: float) -> str:
    raw = struct.pack('<f', value)
    if 1 <= raw[0] <= 4:
        remain = int.from_bytes(raw[1:3], 'little')
        return f'{OPEN_SPIN_ST[raw[0]]} {remain} ms'
    if raw[0] == 0 and raw[1] == 0 and (raw[2] == 0):
        st = raw[3]
        names = ('WAIT', 'SEEK', 'DONE', 'ERR')
        return names[st] if st < 4 else f'home={st}'
    return f'θ={value:.3f} rad'

def mit_ident_ok(raw: bytes, idx: int) -> bool:
    """Data0=0x01+4N，Data1=0x05。N=1→05 05，N=2→09 05。"""
    if len(raw) < 2 or idx < 0:
        return False
    n = idx + 1
    return raw[0] == 1 + 4 * n and raw[1] == 5

def mit_motor_to_out(motor_val: float) -> float:
    """电机端 → 界面输出轴（deg 或 rpm）。"""
    return motor_val / MIT_GEAR_RATIO

def parse_mit_status20(raw: bytes) -> Optional[dict]:
    """解析 16/20mm 状态 20B（CAN 0x702+4N，小端偏置）。"""
    if len(raw) < MIT_STATUS_BYTES:
        return None
    pos_code = int.from_bytes(raw[2:6], 'little') - 2147483648
    vel_code = int.from_bytes(raw[6:8], 'little') - 32768
    iq_code = int.from_bytes(raw[8:10], 'little') - 32768
    volt_u16 = int.from_bytes(raw[10:12], 'little')
    fault = int.from_bytes(raw[12:16], 'little')
    state = raw[16]
    calib = raw[17]
    mode = raw[18]
    mtype = raw[19]
    faults = [name for bit, name in MIT_FAULT_BITS if fault >> bit & 1]
    return {'ident0': raw[0], 'ident1': raw[1], 'pos_code': pos_code, 'pos_deg': pos_code / MIT_POS_COUNTS * 360.0, 'vel_rpm': vel_code / 10.0, 'iq_a': iq_code * 11.0 / 4096.0, 'vbus_v': volt_u16 / 10.0, 'fault': fault, 'faults': faults, 'state': state, 'calib': calib, 'mode': mode, 'mtype': mtype}
