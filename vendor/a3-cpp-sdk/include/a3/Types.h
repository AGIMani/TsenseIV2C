#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace a3 {

constexpr int kSelfMotorNum = 18;
constexpr int kMitMotorNum = 2;
constexpr int kTouchSlotNum = 6;
constexpr int kTouchFingerNum = 5;

constexpr int16_t kTempInvalid = 0x7FFF;
constexpr uint16_t kVinInvalid = 0xFFFF;
constexpr uint16_t kInvalidU16 = 0xFFFF;

constexpr float kMitPosCounts = 2097152.0f;
/** 两路 MIT 输出轴换算都用 50:1。状态帧没有 MIT 温度。 */
constexpr float kMitGearRatio = 50.0f;

constexpr uint8_t kPwrDrv1 = 0x01;
constexpr uint8_t kPwrDrv2 = 0x02;
constexpr uint8_t kPwrDrvAll = 0x03;
constexpr uint8_t kCmdEnable = 1;
constexpr uint8_t kCmdDisable = 2;
constexpr uint8_t kCmdHome = 1;
constexpr uint8_t kCmdHomeReset = 2;
constexpr uint8_t kCmdSpinStart = 1;
constexpr uint8_t kCmdSpinStop = 2;
constexpr uint8_t kBusAll = 0xFF;
constexpr uint8_t kNodeAll = 0xFF;
constexpr uint8_t kTouchSrcAll = 0xFF;

constexpr uint8_t kTouchCmdInfo = 0x02;
constexpr uint8_t kTouchCmdMode = 0x11;
constexpr uint8_t kTouchCmdFreq = 0x14;
constexpr uint8_t kTouchCmdAdc = 0x20;
constexpr uint8_t kTouchCmdForce = 0x21;
constexpr uint8_t kTouchCmdCombined = 0x22;
constexpr uint8_t kTouchCmdMulti = 0x25;

/**
 * USB 槽位行程钳位，等于各连杆在 90° 的行程。
 * DIP / 拇指 DIP（F1）0–12.437477 mm，小拇指 PIP 0–11.761490 mm，
 * PIP / 拇指 MCP（F2）0–16.939022 mm，四指 MCP 0–19.627779 mm。
 */
constexpr float kLinearPosMinMm = 0.f;
constexpr float kDipPosMaxMm = 12.437477f;
constexpr float kPinkyPipPosMaxMm = 11.761490f;
constexpr float kPipPosMaxMm = 16.939022f;
constexpr float kMcpPosMaxMm = 19.627779f;
constexpr float kRotaryPosMinDeg = -20.f;
constexpr float kRotaryPosMaxDeg = 20.f;
constexpr float kMitPosMinDeg = -180.f;
constexpr float kMitPosMaxDeg = 180.f;

/** 本库主机协议，须与 H7 FUN 0x01 回复的 proto_major 一致。 */
constexpr uint8_t kHostProtoMajor = 1;
constexpr uint8_t kHostProtoMinor = 0;
constexpr uint8_t kMinAppMajor = 1;

enum class Bus : uint8_t { Can1 = 0, Can2 = 1, Can3 = 2, All = 0xFF };

/** H7 Flash 中的左右手。0=未设置，1=右手，2=左手。SDK 只读。 */
enum class HandSide : uint8_t { Unset = 0, Right = 1, Left = 2 };

enum class Err : int {
    Ok = 0,
    NotConnected,
    Disconnected,
    Timeout,
    HomeFailed,
    InvalidSlot,
    InvalidArg,
    Io,
    NoPort,
    FwTimeout,
    FwIncompatible,
    NoEth,
    CallbackFailed,
};

inline const char *errMessage(Err e) noexcept {
    switch (e) {
    case Err::Ok: return "OK";
    case Err::NotConnected: return "not connected";
    case Err::Disconnected: return "link lost";
    case Err::Timeout: return "wait timed out";
    case Err::HomeFailed: return "homing failed";
    case Err::InvalidSlot: return "invalid slot";
    case Err::InvalidArg: return "invalid argument";
    case Err::Io: return "I/O failed";
    case Err::NoPort: return "no H7 USB CDC port";
    case Err::FwTimeout: return "firmware version query timed out (upgrade H7 APP)";
    case Err::FwIncompatible: return "firmware protocol incompatible";
    case Err::NoEth: return "no TCP 5001 listener on this subnet (H7 address is DHCP)";
    case Err::CallbackFailed: return "user callback threw an exception";
    }
    return "error";
}

/** 直线槽反馈电流：安培 = 码值 × 该系数。 */
constexpr double kLinearIqAmpPerCount = 0.001374;

/** 旋转槽 #06/#07/#14/#15 反馈电流：安培 =（原始码 − 零点）/ 819。819 = 0.6 × 4095 / 3。遥测里的码值已减去零点。 */
constexpr double kRotaryIqCountsPerAmp = 819.0;

/** 回零 Iq 为电机码值，不是安培。典型直线约 80。 */
struct IqSeekCounts {
    float counts = 80.f;
    IqSeekCounts() = default;
    explicit IqSeekCounts(float c) : counts(c) {}
};

struct FwInfo {
    bool valid = false;
    uint8_t proto_major = 0;
    uint8_t proto_minor = 0;
    uint8_t app_major = 0;
    uint8_t app_minor = 0;
    uint8_t app_patch = 0;
    uint8_t app_build = 0;
    /** FUN 0x01 应答 byte6。旧固件该字节为 0，与未设置相同。 */
    HandSide hand_side = HandSide::Unset;
};

inline bool fwCompatible(const FwInfo &fw) {
    return fw.valid && fw.proto_major == kHostProtoMajor && fw.app_major == kMinAppMajor;
}

/** bit i = USB slot #i。完整 18 槽手才用这个 mask。 */
constexpr uint32_t kSelfOnlineMaskAll = (1u << kSelfMotorNum) - 1u;

/**
 * USB slots:
 *   #00–#05 CAN1 node 1–6 linear mm; #06–#07 CAN1 node 7–8 rotary deg
 *   #08–#13 CAN2 node 1–6 linear mm; #14–#15 CAN2 node 7–8 rotary deg
 *   #16–#17 CAN3 USB node 1–2 linear mm (#17 on CAN is node 5 / 0x305)
 */
inline bool slotIsRotary(int slot) {
    return slot == 6 || slot == 7 || slot == 14 || slot == 15;
}

/** 小拇指/无名指/中指/食指 DIP，以及大拇指 DIP（#16）。 */
inline bool slotIsDip(int slot) {
    return slot == 0 || slot == 3 || slot == 8 || slot == 11 || slot == 16;
}

/** 四指 PIP。大拇指没有 PIP。 */
inline bool slotIsPip(int slot) {
    return slot == 1 || slot == 4 || slot == 9 || slot == 12;
}

/** 四指 MCP，以及大拇指 MCP（#17）。 */
inline bool slotIsMcp(int slot) {
    return slot == 2 || slot == 5 || slot == 10 || slot == 13 || slot == 17;
}

inline float linearPosMaxMm(int slot) {
    if (slot == 1) return kPinkyPipPosMaxMm;
    if (slotIsDip(slot)) return kDipPosMaxMm;
    if (slot == 17 || slotIsPip(slot)) return kPipPosMaxMm;
    return kMcpPosMaxMm;
}

inline const char *slotUnit(int slot) { return slotIsRotary(slot) ? "deg" : "mm"; }

inline const char *slotKind(int slot) { return slotIsRotary(slot) ? "rotary" : "linear"; }

inline const char *slotBusName(int slot) {
    if (slot < 8) return "CAN1";
    if (slot < 16) return "CAN2";
    return "CAN3";
}

inline int slotLocalId(int slot) {
    if (slot < 8) return slot + 1;
    if (slot < 16) return slot - 7;
    return slot - 15;
}

inline float clampSlotPos(int slot, float v) {
    const float lo = slotIsRotary(slot) ? kRotaryPosMinDeg : kLinearPosMinMm;
    const float hi = slotIsRotary(slot) ? kRotaryPosMaxDeg : linearPosMaxMm(slot);
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

inline float clampMitPos(float v) {
    if (v < kMitPosMinDeg) return kMitPosMinDeg;
    if (v > kMitPosMaxDeg) return kMitPosMaxDeg;
    return v;
}

/** USB slot #00..#17 → (bus 0/1/2, node_id). */
inline bool slotToBusNode(int slot, uint8_t *bus, uint8_t *node) {
    if (slot < 0 || slot > 17 || !bus || !node) return false;
    if (slot <= 7) {
        *bus = 0;
        *node = static_cast<uint8_t>(slot + 1);
        return true;
    }
    if (slot <= 15) {
        *bus = 1;
        *node = static_cast<uint8_t>(slot - 7);
        return true;
    }
    *bus = 2;
    *node = static_cast<uint8_t>(slot == 16 ? 1 : 2);
    return true;
}

/** Inverse of slotToBusNode. CAN3 node 5 is the same USB slot as node 2 (#17). */
inline int busNodeToSlot(uint8_t bus, uint8_t node) {
    if (bus <= 1 && node >= 1 && node <= 8) return int(bus) * 8 + int(node) - 1;
    if (bus == 2 && node == 1) return 16;
    if (bus == 2 && (node == 2 || node == 5)) return 17;
    return -1;
}

/** CAN1/2: linear 1–6, rotary 7–8; CAN3: both self motors are linear. */
inline std::vector<uint8_t> busKindNodes(int bus, bool rotary) {
    if (bus >= 0 && bus <= 1) {
        if (rotary) return {7, 8};
        return {1, 2, 3, 4, 5, 6};
    }
    if (bus == 2) {
        if (rotary) return {};
        return {1, 2};
    }
    return {};
}

/**
 * 0x02 每槽前 4 字节，没有单独的类型位。小端：
 * - byte0=byte1=byte2=0，byte3 为 0..4：回零态（WAIT/SEEK/DONE/ERR/RELEASE）。
 * - byte0 为 1..4 且 byte3 仍是 0..4：开环往复（FWD/REV/DONE/ERR），byte1–2 为剩余毫秒。
 * - 其余：电角度 float32，单位 rad。直线只在机械归零 DONE 之后才发这个 float。
 * 电角度的尾数低字节可以是 01–04，不能单凭 byte0 判开环。
 * 恰好为 0 的 float 与 WAIT 字节相同，按 WAIT。
 */
enum class HomingState : uint8_t {
    Wait = 0,
    Seek = 1,
    Done = 2,
    Err = 3,
    Release = 4,
    Theta = 0xFE,    /**< 254：已是电角度，直线上表示机械归零已完成 */
    OpenSpin = 0xFF  /**< 255：开环往复 overlay，不是电角度 */
};

inline bool homingBytesAreOpenSpin(const uint8_t raw[4]) noexcept {
    return raw[0] >= 1 && raw[0] <= 4 && raw[3] <= 4;
}

enum class TouchKind : uint8_t { None = 0, Single = 1, Multi = 2 };

/** USB FUN 0x06. Cleared by the reader after kProtectStaleMs without a new frame. */
struct ProtectEvent {
    bool active = false;
    bool over_temp = false;
    bool over_current = false;
    uint8_t bus = 0;
    uint8_t motor_id = 0;
    int slot = -1;
};

constexpr int kProtectStaleMs = 5000;

struct SelfTelem {
    float homing_or_theta = 0.f;
    HomingState homing = HomingState::Wait;
    float pos_act = 0.f;
    /**
     * 直线槽为安培：码值 × kLinearIqAmpPerCount。
     * 旋转槽 #06/#07/#14/#15 为安培：码值 / kRotaryIqCountsPerAmp。
     * 有符号，0 为无电流。回零 IqSeekCounts 仍是码值。
     */
    float iq_meas = 0.f;
    float iq_ref = 0.f;
    float pos_ref = 0.f;
    int16_t grav_iq_mA = 0;
    int16_t mcu_temp_centi = kTempInvalid;
    bool online = false;
    bool open_spin = false;
};

struct MitTelem {
    bool valid = false;
    bool online = false;
    float pos_out_deg = 0.f;
    float vel_out_rpm = 0.f;
    float iq_a = 0.f;
    float vbus_v = 0.f;
    uint32_t fault = 0;
    uint8_t state = 0;
    uint8_t calib = 0;
    uint8_t mode = 0;
    uint8_t mtype = 0;
};

struct BoardTelem {
    bool valid = false;
    int16_t ntc_c_centi = kTempInvalid;
    int16_t ntc_b_centi = kTempInvalid;
    uint16_t vin24_mV = kVinInvalid;
};

struct TouchPart {
    bool valid = false;
    int rows = 0;
    int cols = 0;
    int ok = 0;
    float mean = 0.f;
    float mx = 0.f;
    std::vector<float> vals; /* NaN = invalid sample */
};

struct TouchSlot {
    bool online = false;
    uint8_t cmd = 0;
    uint8_t bus = 0;
    uint16_t seq = 0;
    uint8_t ack = 0;
    uint8_t ack_cmd = 0;
    TouchKind kind = TouchKind::None;
    std::array<TouchPart, 3> parts{};
};

struct ComPort {
    std::string name;
    std::string desc;
    bool h7_cdc = false;
};

struct MitCmd {
    float pos_out_deg = 0.f;
    float vel_out_rpm = 0.f;
    float iq_a = 0.f;
};

constexpr int kTelemetryStaleMs = 1000;
struct TelemetrySnapshot {
    std::array<SelfTelem, kSelfMotorNum> self{};
    std::array<MitTelem, kMitMotorNum> mit{};
    std::array<ProtectEvent, kSelfMotorNum> protect{};
    BoardTelem board{};
    std::array<TouchSlot, kTouchSlotNum> touch{};
    uint32_t rx_mask = 0;
    uint16_t seq = 0;
    uint32_t tick_ms = 0;
    int64_t age_ms = -1;
    bool fresh = false;
};

inline HomingState homingState(float homing_or_theta) {
    uint8_t raw[4];
    std::memcpy(raw, &homing_or_theta, 4);
    if (homingBytesAreOpenSpin(raw)) return HomingState::OpenSpin;
    if (raw[0] == 0 && raw[1] == 0 && raw[2] == 0) {
        if (raw[3] == 0) return HomingState::Wait;
        if (raw[3] == 1) return HomingState::Seek;
        if (raw[3] == 2) return HomingState::Done;
        if (raw[3] == 3) return HomingState::Err;
        if (raw[3] == 4) return HomingState::Release;
        return HomingState::Wait;
    }
    return HomingState::Theta;
}

inline bool homingIsDone(const SelfTelem &s) {
    if (!s.online) return false;
    const HomingState st = homingState(s.homing_or_theta);
    return st == HomingState::Done || st == HomingState::Theta;
}

inline bool homingFailed(const SelfTelem &s) {
    return s.online && homingState(s.homing_or_theta) == HomingState::Err;
}

} // namespace a3
