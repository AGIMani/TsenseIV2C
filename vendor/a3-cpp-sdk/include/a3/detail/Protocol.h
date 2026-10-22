#pragma once

#include "a3/A3Export.h"
#include "a3/Types.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace a3 {

constexpr uint8_t kDevTelem = 3;
constexpr uint8_t kFunHostInfo = 0x01;
constexpr uint8_t kFunHandTelem = 0x02;
constexpr uint8_t kFunBoardTelem = 0x03;
constexpr uint8_t kFunTouchData = 0x04;
constexpr uint8_t kFunTouchAck = 0x05;
constexpr uint8_t kFunProtect = 0x06;
constexpr uint8_t kFunHandSide = 0x10; /* 只读：payload op=0 */
constexpr uint8_t kFunTouchCmd = 0x30;
constexpr uint8_t kFunHandPosBatch = 0x20;
constexpr uint8_t kFunSelfPwr = 0x21;
constexpr uint8_t kFunSelfHoming = 0x22;
constexpr uint8_t kFunSelfPosPid = 0x23;
constexpr uint8_t kFunSelfCurPid = 0x24;
constexpr uint8_t kFunSelfOpenSpin = 0x25;

constexpr uint16_t kHandBatchPayloadLen = 100;
constexpr uint16_t kHandBatchWireLen = 108;
constexpr uint16_t kHandTelemHdrLen = 12;
constexpr uint16_t kHandTelemSlotBytes = 24;
constexpr uint16_t kHandTelemSelfLen = 444;
constexpr uint16_t kHandTelemPayloadLen = 484;
constexpr uint16_t kBoardTelemPayloadLen = 8;
constexpr uint16_t kProtectPayloadLen = 8;
constexpr uint16_t kMitStatusBytes = 20;
constexpr uint16_t kHostInfoPayloadLen = 8;

inline float mitOutToMotor(float out_val) { return out_val * kMitGearRatio; }
inline float mitMotorToOut(float motor_val) { return motor_val / kMitGearRatio; }

A3_API uint16_t crc16Modbus(const uint8_t *data, size_t len);

A3_API std::vector<uint8_t> packAa55(uint8_t fun, const uint8_t *payload, size_t plen);

inline std::vector<uint8_t> packAa55(uint8_t fun, const std::vector<uint8_t> &payload) {
    return packAa55(fun, payload.empty() ? nullptr : payload.data(), payload.size());
}

A3_API std::vector<uint8_t> packHostInfo();

/** FUN 0x10 读取左右手。只组 op=0，不会写入 Flash。 */
A3_API std::vector<uint8_t> packHandSideQuery();

/** 解析 FUN 0x10 应答。status=0 且 side 为 0/1/2 时成功。 */
A3_API bool parseHandSide(const uint8_t *payload, size_t plen, HandSide *side, uint8_t *status = nullptr);

/** Pack 108B 0x20. mit_* are motor-side (output-axis values must already be ×50). */
A3_API void packHandBatch(uint8_t *wire108,
                          uint8_t seq,
                          const float self_pos[kSelfMotorNum],
                          const float mit_pos_motor_deg[kMitMotorNum],
                          const float mit_vel_motor_rpm[kMitMotorNum],
                          const float mit_iq_a[kMitMotorNum]);

A3_API std::vector<uint8_t> packSelfPwr(uint8_t bus, uint8_t node, uint8_t cmd, uint8_t drv = kPwrDrvAll);
A3_API std::vector<uint8_t> packSelfHoming(uint8_t bus, uint8_t node, uint8_t cmd, float iq_seek);
A3_API std::vector<uint8_t> packSelfPosPid(uint8_t bus, uint8_t node, float kp, float ki, float kd);
A3_API std::vector<uint8_t> packSelfCurPid(uint8_t bus, uint8_t node,
                                           float id_kp, float id_ki, float id_kd,
                                           float iq_kp, float iq_ki, float iq_kd);
A3_API std::vector<uint8_t> packSelfOpenSpin(uint8_t bus, uint8_t node, uint8_t cmd,
                                             float freq_hz, float half_s);
A3_API std::vector<uint8_t> packTouchCmd(uint8_t src, uint8_t cmd, const uint8_t *data, size_t n);

A3_API std::vector<std::vector<uint8_t>> popAa55Frames(std::vector<uint8_t> &buf,
                                                       size_t max_total = 2048);

struct FrameView {
    uint16_t plen = 0;
    uint8_t dev = 0;
    uint8_t fun = 0;
    const uint8_t *payload = nullptr;
};

A3_API bool parseFrame(const std::vector<uint8_t> &raw, FrameView *out);

A3_API bool parseHostInfo(const uint8_t *payload, size_t plen, FwInfo *out);

A3_API bool parseHandTelem(const uint8_t *payload, size_t plen,
                           std::array<SelfTelem, kSelfMotorNum> *self,
                           std::array<MitTelem, kMitMotorNum> *mit,
                           uint16_t *seq, uint32_t *tick_ms);

A3_API bool parseBoardTelem(const uint8_t *payload, size_t plen, BoardTelem *out);
A3_API bool parseProtect(const uint8_t *payload, size_t plen, ProtectEvent *out);

A3_API bool parseTouchData(uint8_t cmd, const uint8_t *data, size_t n, TouchSlot *out);

A3_API std::string decodeHomingOrTheta(float value);
A3_API std::string decodeTempCenti(int16_t centi);
A3_API std::string decodeVinMv(uint16_t mv);
A3_API std::string decodeGravIq(int16_t ma);
A3_API std::string fmtMitState(uint8_t st);
A3_API std::string fmtMitCalib(uint8_t st);
A3_API std::string fmtMitMode(uint8_t mode);
A3_API std::string fmtMitType(uint8_t mtype);
A3_API std::string fmtMitFaults(uint32_t fault);
A3_API std::string fmtProtect(bool over_temp, bool over_current);
A3_API std::string fmtTouchAck(uint8_t ack);

} // namespace a3
