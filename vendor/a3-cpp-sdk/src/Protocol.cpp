#include "a3/detail/Protocol.h"
#include "a3/Version.h"

#include <cmath>
#include <cstddef>
#include <cstring>
#include <sstream>

namespace a3 {
namespace {

constexpr uint16_t crc_tab[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780,
    0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440, 0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1,
    0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841, 0xD801,
    0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81, 0x1F40,
    0xDD01, 0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680,
    0xD641, 0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0,
    0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501,
    0x35C0, 0x3480, 0xF441, 0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840, 0x2800, 0xE8C1, 0xE981,
    0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1,
    0xEC81, 0x2C40, 0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640, 0x2200,
    0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041, 0xA001, 0x60C0, 0x6180, 0xA141,
    0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480,
    0xA441, 0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 0xAA01, 0x6AC0,
    0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01,
    0x7BC0, 0x7A80, 0xBA41, 0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381,
    0x7340, 0xB101, 0x71C0, 0x7080, 0xB041, 0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0,
    0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01,
    0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40,
    0x9901, 0x59C0, 0x5880, 0x9841, 0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81,
    0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41, 0x4400, 0x84C1,
    0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341, 0x4100,
    0x81C1, 0x8081, 0x4040};

void wr_u16(uint8_t *p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

void wr_f32(uint8_t *p, float v) { std::memcpy(p, &v, 4); }

uint16_t rd_u16(const uint8_t *p) {
    return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}

uint32_t rd_u32(const uint8_t *p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

float rd_f32(const uint8_t *p) {
    float v;
    std::memcpy(&v, p, 4);
    return v;
}

int16_t rd_i16(const uint8_t *p) { return static_cast<int16_t>(rd_u16(p)); }

int samples_that_fit(size_t avail, int stride) {
    if (stride <= 0 || avail < 2) return 0;
    return static_cast<int>((avail - 2) / static_cast<size_t>(stride)) + 1;
}

void matrix_fn_stats(const uint8_t *raw, int n, int stride, size_t avail, TouchPart *part) {
    if (n < 0) n = 0;
    part->vals.assign(static_cast<size_t>(n), NAN);
    part->ok = 0;
    part->mean = 0.f;
    part->mx = 0.f;
    int use = samples_that_fit(avail, stride);
    if (raw == nullptr || use > n) use = raw == nullptr ? 0 : n;
    if (use <= 0) return;
    double s = 0.0;
    for (int i = 0; i < use; ++i) {
        uint16_t v = rd_u16(raw + static_cast<size_t>(i) * static_cast<size_t>(stride));
        if (v == kInvalidU16) continue;
        float fv = static_cast<float>(v);
        part->vals[static_cast<size_t>(i)] = fv;
        s += fv;
        if (fv > part->mx) part->mx = fv;
        part->ok++;
    }
    if (part->ok) part->mean = static_cast<float>(s / part->ok);
}

} // namespace

uint16_t crc16Modbus(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc = static_cast<uint16_t>((crc >> 8) ^ crc_tab[(crc ^ data[i]) & 0xFF]);
    }
    return crc;
}

std::vector<uint8_t> packAa55(uint8_t fun, const uint8_t *payload, size_t plen) {
    std::vector<uint8_t> out(8 + plen);
    out[0] = 0xAA;
    out[1] = 0x55;
    wr_u16(&out[2], static_cast<uint16_t>(plen));
    out[4] = kDevTelem;
    out[5] = fun;
    if (plen && payload) std::memcpy(&out[6], payload, plen);
    uint16_t crc = crc16Modbus(out.data(), 6 + plen);
    wr_u16(&out[6 + plen], crc);
    return out;
}

void packHandBatch(uint8_t *wire108,
                   uint8_t seq,
                   const float self_pos[kSelfMotorNum],
                   const float mit_pos_motor_deg[kMitMotorNum],
                   const float mit_vel_motor_rpm[kMitMotorNum],
                   const float mit_iq_a[kMitMotorNum]) {
    std::memset(wire108, 0, kHandBatchWireLen);
    wire108[0] = 0xAA;
    wire108[1] = 0x55;
    wr_u16(&wire108[2], kHandBatchPayloadLen);
    wire108[4] = kDevTelem;
    wire108[5] = kFunHandPosBatch;
    wire108[6] = seq;
    uint8_t *p = &wire108[7];
    for (int i = 0; i < kSelfMotorNum; ++i) {
        wr_f32(p, self_pos[i]);
        p += 4;
    }
    for (int i = 0; i < kMitMotorNum; ++i) {
        wr_f32(p, mit_pos_motor_deg[i]);
        p += 4;
    }
    for (int i = 0; i < kMitMotorNum; ++i) {
        wr_f32(p, mit_vel_motor_rpm[i]);
        p += 4;
    }
    for (int i = 0; i < kMitMotorNum; ++i) {
        wr_f32(p, mit_iq_a[i]);
        p += 4;
    }
    uint16_t crc = crc16Modbus(wire108, 106);
    wr_u16(&wire108[106], crc);
}

std::vector<uint8_t> packSelfPwr(uint8_t bus, uint8_t node, uint8_t cmd, uint8_t drv) {
    uint8_t pl[4] = {bus, node, cmd, drv};
    return packAa55(kFunSelfPwr, pl, 4);
}

std::vector<uint8_t> packSelfHoming(uint8_t bus, uint8_t node, uint8_t cmd, float iq_seek) {
    uint8_t pl[12]{};
    pl[0] = bus;
    pl[1] = node;
    pl[2] = cmd;
    wr_f32(&pl[4], iq_seek);
    return packAa55(kFunSelfHoming, pl, 12);
}

std::vector<uint8_t> packSelfPosPid(uint8_t bus, uint8_t node, float kp, float ki, float kd) {
    uint8_t pl[16]{};
    pl[0] = bus;
    pl[1] = node;
    wr_f32(&pl[4], kp);
    wr_f32(&pl[8], ki);
    wr_f32(&pl[12], kd);
    return packAa55(kFunSelfPosPid, pl, 16);
}

std::vector<uint8_t> packSelfCurPid(uint8_t bus, uint8_t node,
                                    float id_kp, float id_ki, float id_kd,
                                    float iq_kp, float iq_ki, float iq_kd) {
    uint8_t pl[28]{};
    pl[0] = bus;
    pl[1] = node;
    wr_f32(&pl[4], id_kp);
    wr_f32(&pl[8], id_ki);
    wr_f32(&pl[12], id_kd);
    wr_f32(&pl[16], iq_kp);
    wr_f32(&pl[20], iq_ki);
    wr_f32(&pl[24], iq_kd);
    return packAa55(kFunSelfCurPid, pl, 28);
}

std::vector<uint8_t> packSelfOpenSpin(uint8_t bus, uint8_t node, uint8_t cmd,
                                      float freq_hz, float half_s) {
    uint8_t pl[12]{};
    pl[0] = bus;
    pl[1] = node;
    pl[2] = cmd;
    wr_f32(&pl[4], freq_hz);
    wr_f32(&pl[8], half_s);
    return packAa55(kFunSelfOpenSpin, pl, 12);
}

std::vector<uint8_t> packTouchCmd(uint8_t src, uint8_t cmd, const uint8_t *data, size_t n) {
    std::vector<uint8_t> pl(4 + n);
    pl[0] = src;
    pl[1] = cmd;
    wr_u16(&pl[2], static_cast<uint16_t>(n));
    if (n && data) std::memcpy(&pl[4], data, n);
    return packAa55(kFunTouchCmd, pl.data(), pl.size());
}

std::vector<uint8_t> packHostInfo() {
    uint8_t pl[kHostInfoPayloadLen] = {
        static_cast<uint8_t>(A3_SDK_VERSION_MAJOR),
        static_cast<uint8_t>(A3_SDK_VERSION_MINOR),
        static_cast<uint8_t>(A3_SDK_VERSION_PATCH),
        kHostProtoMajor,
        kHostProtoMinor,
        0,
        0,
        0};
    return packAa55(kFunHostInfo, pl, kHostInfoPayloadLen);
}

std::vector<uint8_t> packHandSideQuery() {
    const uint8_t pl[2] = {0, 0};
    return packAa55(kFunHandSide, pl, 2);
}

bool parseHandSide(const uint8_t *payload, size_t plen, HandSide *side, uint8_t *status) {
    if (!payload || !side || plen < 2) return false;
    if (payload[1] > static_cast<uint8_t>(HandSide::Left)) return false;
    if (status) *status = payload[0];
    *side = static_cast<HandSide>(payload[1]);
    return true;
}

bool parseHostInfo(const uint8_t *payload, size_t plen, FwInfo *out) {
    if (!payload || !out || plen < kHostInfoPayloadLen) return false;
    out->valid = true;
    out->proto_major = payload[0];
    out->proto_minor = payload[1];
    out->app_major = payload[2];
    out->app_minor = payload[3];
    out->app_patch = payload[4];
    out->app_build = payload[5];
    out->hand_side = (payload[6] <= static_cast<uint8_t>(HandSide::Left))
                         ? static_cast<HandSide>(payload[6])
                         : HandSide::Unset;
    return true;
}

std::vector<std::vector<uint8_t>> popAa55Frames(std::vector<uint8_t> &buf, size_t max_total) {
    std::vector<std::vector<uint8_t>> frames;
    while (true) {
        size_t idx = 0;
        bool found = false;
        for (; idx + 1 < buf.size(); ++idx) {
            if (buf[idx] == 0xAA && buf[idx + 1] == 0x55) {
                found = true;
                break;
            }
        }
        if (!found) {
            if (!buf.empty() && buf.back() == 0xAA) {
                uint8_t last = buf.back();
                buf.clear();
                buf.push_back(last);
            } else {
                buf.clear();
            }
            break;
        }
        if (idx) buf.erase(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(idx));
        if (buf.size() < 8) break;
        uint16_t plen = rd_u16(&buf[2]);
        size_t total = static_cast<size_t>(plen) + 8;
        if (plen == 0 || total > max_total) {
            buf.erase(buf.begin());
            continue;
        }
        if (buf.size() < total) break;
        std::vector<uint8_t> raw(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(total));
        uint16_t crc_rx = rd_u16(&raw[total - 2]);
        if (crc16Modbus(raw.data(), total - 2) == crc_rx) {
            frames.push_back(std::move(raw));
            buf.erase(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(total));
        } else {
            buf.erase(buf.begin());
        }
    }
    return frames;
}

bool parseFrame(const std::vector<uint8_t> &raw, FrameView *out) {
    if (!out || raw.size() < 8) return false;
    if (raw[0] != 0xAA || raw[1] != 0x55 ||
        crc16Modbus(raw.data(), raw.size() - 2) != rd_u16(raw.data() + raw.size() - 2)) return false;
    out->plen = rd_u16(&raw[2]);
    if (raw.size() != static_cast<size_t>(out->plen) + 8) return false;
    out->dev = raw[4];
    out->fun = raw[5];
    out->payload = raw.data() + 6;
    return true;
}

bool parseHandTelem(const uint8_t *payload, size_t plen,
                    std::array<SelfTelem, kSelfMotorNum> *self,
                    std::array<MitTelem, kMitMotorNum> *mit,
                    uint16_t *seq, uint32_t *tick_ms) {
    if (!payload || !self || plen < kHandTelemSelfLen) return false;
    if (seq) *seq = rd_u16(payload);
    uint16_t mit_mask = rd_u16(payload + 2);
    if (tick_ms) *tick_ms = rd_u32(payload + 4);
    uint32_t rx_mask = rd_u32(payload + 8);
    const uint8_t *off = payload + kHandTelemHdrLen;
    for (int slot = 0; slot < kSelfMotorNum; ++slot) {
        SelfTelem &s = (*self)[static_cast<size_t>(slot)];
        s.homing_or_theta = rd_f32(off);
        s.pos_act = rd_f32(off + 4);
        s.iq_meas = rd_f32(off + 8);
        s.iq_ref = rd_f32(off + 12);
        if (slotIsRotary(slot)) {
            s.iq_meas = static_cast<float>(static_cast<double>(s.iq_meas) / kRotaryIqCountsPerAmp);
            s.iq_ref = static_cast<float>(static_cast<double>(s.iq_ref) / kRotaryIqCountsPerAmp);
        } else {
            s.iq_meas = static_cast<float>(static_cast<double>(s.iq_meas) * kLinearIqAmpPerCount);
            s.iq_ref = static_cast<float>(static_cast<double>(s.iq_ref) * kLinearIqAmpPerCount);
        }
        s.pos_ref = rd_f32(off + 16);
        s.grav_iq_mA = rd_i16(off + 20);
        s.mcu_temp_centi = rd_i16(off + 22);
        s.online = ((rx_mask >> slot) & 1u) != 0;
        s.open_spin = homingBytesAreOpenSpin(off);
        s.homing = homingState(s.homing_or_theta);
        off += kHandTelemSlotBytes;
    }
    if (mit && plen >= kHandTelemPayloadLen) {
        const uint8_t *moff = payload + kHandTelemSelfLen;
        for (int idx = 0; idx < kMitMotorNum; ++idx) {
            const uint8_t *raw = moff + idx * kMitStatusBytes;
            MitTelem &m = (*mit)[static_cast<size_t>(idx)];
            m.online = ((mit_mask >> idx) & 1u) != 0;
            uint8_t ident0 = raw[0];
            uint8_t ident1 = raw[1];
            int n = idx + 1;
            bool ident_ok = (ident0 == static_cast<uint8_t>(0x01 + 4 * n)) && ident1 == 0x05;
            m.valid = ident_ok || m.online;
            if (!m.valid) {
                m = MitTelem{};
                m.online = false;
                m.valid = false;
                continue;
            }
            int32_t pos_code = static_cast<int32_t>(rd_u32(raw + 2) - 0x80000000u);
            int16_t vel_code = static_cast<int16_t>(rd_u16(raw + 6) - 0x8000);
            int16_t iq_code = static_cast<int16_t>(rd_u16(raw + 8) - 0x8000);
            uint16_t volt = rd_u16(raw + 10);
            m.fault = rd_u32(raw + 12);
            m.state = raw[16];
            m.calib = raw[17];
            m.mode = raw[18];
            m.mtype = raw[19];
            m.pos_out_deg = mitMotorToOut(static_cast<float>(pos_code) / kMitPosCounts * 360.f);
            m.vel_out_rpm = mitMotorToOut(static_cast<float>(vel_code) / 10.f);
            m.iq_a = static_cast<float>(iq_code) * 11.f / 4096.f;
            m.vbus_v = static_cast<float>(volt) / 10.f;
        }
    }
    return true;
}

bool parseBoardTelem(const uint8_t *payload, size_t plen, BoardTelem *out) {
    if (!payload || !out || plen < kBoardTelemPayloadLen) return false;
    out->valid = true;
    out->ntc_c_centi = rd_i16(payload + 2);
    out->ntc_b_centi = rd_i16(payload + 4);
    out->vin24_mV = rd_u16(payload + 6);
    return true;
}

bool parseProtect(const uint8_t *payload, size_t plen, ProtectEvent *out) {
    if (!payload || !out || plen < kProtectPayloadLen) return false;
    uint8_t ot = payload[2];
    uint8_t oc = payload[3];
    if (ot > 1 || oc > 1 || (ot | oc) == 0) return false;
    int slot = (payload[4] <= 17) ? int(payload[4]) : busNodeToSlot(payload[0], payload[1]);
    if (slot < 0 || slot >= kSelfMotorNum) return false;
    out->active = true;
    out->over_temp = ot == 1;
    out->over_current = oc == 1;
    out->bus = payload[0];
    out->motor_id = payload[1];
    out->slot = slot;
    return true;
}

bool parseTouchData(uint8_t cmd, const uint8_t *data, size_t n, TouchSlot *out) {
    if (!out || !data) return false;
    out->cmd = cmd;
    if (cmd == kTouchCmdMulti) {
        if (n < 8) return false;
        out->kind = TouchKind::Multi;
        out->seq = rd_u16(data + 5);
        uint8_t mode = data[4];
        uint8_t flags = data[7];
        int stride = (mode == 2) ? 6 : 2;
        size_t off = 8;
        for (int i = 0; i < 3; ++i) {
            TouchPart &pt = out->parts[static_cast<size_t>(i)];
            if (off + 2 > n) {
                pt = TouchPart{};
                continue;
            }
            pt.rows = data[off];
            pt.cols = data[off + 1];
            off += 2;
            int nn = pt.rows * pt.cols;
            size_t nbytes = static_cast<size_t>(nn) * static_cast<size_t>(stride);
            const uint8_t *mat = (off < n) ? data + off : nullptr;
            size_t have = (off < n) ? (n - off) : 0;
            if (have < nbytes) nbytes = have;
            matrix_fn_stats(mat, nn, stride, have, &pt);
            pt.valid = ((flags >> i) & 1u) != 0;
            off += nbytes;
        }
        return true;
    }
    if (cmd == kTouchCmdAdc || cmd == kTouchCmdForce || cmd == kTouchCmdCombined) {
        if (n < 8) return false;
        out->kind = TouchKind::Single;
        out->seq = rd_u16(data + 6);
        int rows = data[4];
        int cols = data[5];
        int stride = (cmd == kTouchCmdCombined) ? 6 : 2;
        int nn = rows * cols;
        TouchPart &pt = out->parts[0];
        pt.rows = rows;
        pt.cols = cols;
        pt.valid = true;
        matrix_fn_stats(data + 8, nn, stride, n - 8, &pt);
        out->parts[1] = TouchPart{};
        out->parts[2] = TouchPart{};
        return true;
    }
    return false;
}

std::string decodeHomingOrTheta(float value) {
    uint8_t raw[4];
    std::memcpy(raw, &value, 4);
    static const char *spin[] = {"IDLE", "FWD", "REV", "DONE", "ERR"};
    if (homingBytesAreOpenSpin(raw)) {
        int remain = raw[1] | (raw[2] << 8);
        std::ostringstream os;
        os << spin[raw[0]] << " " << remain << " ms";
        return os.str();
    }
    if (raw[0] == 0 && raw[1] == 0 && raw[2] == 0) {
        static const char *names[] = {"WAIT", "SEEK", "DONE", "ERR"};
        if (raw[3] < 4) return names[raw[3]];
        return "home=" + std::to_string(raw[3]);
    }
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os.precision(3);
    os << "θ=" << value << " rad";
    return os.str();
}

std::string decodeTempCenti(int16_t centi) {
    if (centi == kTempInvalid) return "—";
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os.precision(2);
    os << (centi / 100.0) << " ℃";
    return os.str();
}

std::string decodeVinMv(uint16_t mv) {
    if (mv == kVinInvalid) return "—";
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os.precision(2);
    os << (mv / 1000.0) << " V";
    return os.str();
}

std::string decodeGravIq(int16_t ma) {
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os.precision(3);
    os << (ma / 1000.0) << " A";
    return os.str();
}

std::string fmtMitState(uint8_t st) {
    static const char *n[] = {"初始化", "电流校准", "编码器校准", "停止", "运行", "错误"};
    if (st < 6) return n[st];
    return "st=" + std::to_string(st);
}

std::string fmtMitCalib(uint8_t st) {
    static const char *n[] = {"空闲", "校准中", "完成", "错误"};
    if (st < 4) return n[st];
    return "cal=" + std::to_string(st);
}

std::string fmtMitMode(uint8_t mode) {
    if (mode == 0) return "MIT";
    if (mode == 1) return "速度";
    if (mode == 2) return "位置";
    return "mode";
}

std::string fmtMitType(uint8_t mtype) {
    if (mtype == 1) return "30mm";
    if (mtype == 2) return "25mm";
    if (mtype == 3) return "20mm";
    if (mtype == 4) return "16mm";
    return "type";
}

std::string fmtProtect(bool over_temp, bool over_current) {
    if (over_temp && over_current) return "过温+过流";
    if (over_temp) return "过温";
    if (over_current) return "过流";
    return "—";
}

std::string fmtMitFaults(uint32_t fault) {
    static const char *bits[] = {"驱动", "过压", "欠压", "过流", "编码器"};
    std::string s;
    for (int b = 0; b < 5; ++b) {
        if ((fault >> b) & 1u) {
            if (!s.empty()) s += "/";
            s += bits[b];
        }
    }
    return s;
}

std::string fmtTouchAck(uint8_t ack) {
    if (ack == 0) return "成功";
    if (ack == 1) return "失败";
    if (ack == 2) return "超时";
    if (ack == 3) return "未知 src";
    return "ack=" + std::to_string(ack);
}

} // namespace a3
