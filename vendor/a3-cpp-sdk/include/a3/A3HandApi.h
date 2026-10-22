#pragma once

#include "a3/A3Export.h"
#include "a3/ITransport.h"
#include "a3/Stroke.h"
#include "a3/Types.h"
#include "a3/Version.h"

#include <array>
#include <functional>
#include <memory>
#include <string>

namespace a3 {

class A3_API A3HandApi {
public:
    using TelemCallback = std::function<void()>;
    using DisconnectCallback = std::function<void(Err code, const std::string &reason)>;

    A3HandApi();
    ~A3HandApi();
    A3HandApi(const A3HandApi &) = delete;
    A3HandApi &operator=(const A3HandApi &) = delete;
    A3HandApi(A3HandApi &&) = delete;
    A3HandApi &operator=(A3HandApi &&) = delete;

    bool connectUsb(const std::string &port, std::string *err = nullptr);
    /** First H7 CDC port (VID 0483:5740 / "USB Serial Device"). */
    bool connectUsbAuto(std::string *err = nullptr);
    bool connectEth(const std::string &host, uint16_t port = 5001, std::string *err = nullptr);
    /**
     * Scan local IPv4 subnets for TCP `port` and connect the first H7 that answers FUN 0x01.
     * The hand address comes from DHCP. Pass a host from listEthHosts() to connectEth() when
     * more than one device is on the subnet.
     */
    bool connectEthAuto(uint16_t port = 5001, std::string *err = nullptr);
    /** Custom transport (sim / replay). Does not query firmware. */
    bool attach(std::unique_ptr<ITransport> t, std::string *err = nullptr);
    void disconnect();
    bool connected() const;

    /**
     * Query H7 FUN 0x01. connectUsb / connectEth already perform this query.
     * attach() skips this. Do not call from a telemetry callback.
     */
    bool handshake(int timeout_ms = 1500, std::string *err = nullptr);
    FwInfo firmware() const;

    /**
     * Read the left/right flag stored on the H7 (FUN 0x10, op=0).
     * Does not write Flash. Do not call from a telemetry callback.
     * 0=unset, 1=right, 2=left. After handshake, firmware().hand_side is the
     * same flag from FUN 0x01 byte 6.
     */
    bool queryHandSide(HandSide *side, int timeout_ms = 1500, std::string *err = nullptr);

    /** Linear mm / rotary deg; finite values are clamped. Invalid input leaves targets unchanged.
     * Setters return false and set lastErr() on invalid arguments. Success clears lastErr().
     */
    /**
     * 20 个机构关节角（度）先换成行程，再写入自研槽和两台外置电机。
     * 直线为 mm，旋转为 deg。越界返回 false，已有目标不变。
     * 外置电机的速度和电流前馈写 0。
     */
    bool setJointAnglesDeg(const float angle_deg[kJointNum]);
    bool setPositions(const float self_pos[kSelfMotorNum]);
    /** Output-axis units (deg / rpm). Gear ×50 is applied internally. */
    bool setMit(int idx, float pos_out_deg, float vel_out_rpm, float iq_a);
    bool setMitAll(const MitCmd mit[kMitMotorNum]);

    bool sendOnce(std::string *err = nullptr);
    bool startStream(int hz = 500, std::string *err = nullptr);
    void stopStream();
    bool streaming() const;
    int streamHz() const;
    double txHz() const;
    double rxHz() const;

    bool enable(uint8_t bus, uint8_t node, bool on, uint8_t drv = kPwrDrvAll, std::string *err = nullptr);
    bool enableSlot(int slot, bool on, uint8_t drv = kPwrDrvAll, std::string *err = nullptr);
    bool enableBusKind(int bus, bool rotary, bool on, uint8_t drv = kPwrDrvAll, std::string *err = nullptr);
    bool home(uint8_t bus, uint8_t node, IqSeekCounts iq, std::string *err = nullptr);
    bool homeSlot(int slot, IqSeekCounts iq, std::string *err = nullptr);
    bool homeBusKind(int bus, bool rotary, IqSeekCounts iq, std::string *err = nullptr);
    bool homeReset(uint8_t bus, uint8_t node, std::string *err = nullptr);
    bool homeResetSlot(int slot, std::string *err = nullptr);
    bool setPosPid(int slot, float kp, float ki, float kd, std::string *err = nullptr);
    bool setCurPid(int slot,
                   float id_kp, float id_ki, float id_kd,
                   float iq_kp, float iq_ki, float iq_kd,
                   std::string *err = nullptr);
    bool openSpin(int slot, bool start, float freq_hz, float half_s, std::string *err = nullptr);
    bool sendTouch(uint8_t src, uint8_t cmd, const uint8_t *data, size_t n, std::string *err = nullptr);

    /**
     * Wait until rx_mask covers every bit in mask. Use the slots your hand actually has,
     * not kSelfOnlineMaskAll unless all 18 motors are present. timeout_ms=0 is a poll.
     * Do not call from a telemetry callback.
     */
    bool waitOnline(uint32_t mask, int timeout_ms, std::string *err = nullptr);
    bool waitSlotOnline(int slot, int timeout_ms, std::string *err = nullptr);
    /**
     * Wait until this command's homing finishes: packed DONE, or electrical-angle
     * mode (Theta). On linear joints Theta is sent only after mechanical homing
     * locks the zero and enables the position loop. Rotary joints keep packed DONE.
     * After home / homeSlot / homeBusKind, a SEEK or RELEASE sample from this command
     * must arrive first. A SEEK already in progress does not count until the slot
     * leaves SEEK/RELEASE and enters it again. A DONE or theta value already on the
     * link does not count. After a successful homeReset, the homing state must change
     * before DONE counts.
     * ERR in telemetry fails immediately. Do not call from a telemetry callback.
     */
    bool waitHomeDone(int slot, int timeout_ms, std::string *err = nullptr);
    /**
     * Stop the 0x20 stream and broadcast driver-off (0x21 cmd=2, bus=node=0xFF).
     * This is "disable sent", not "motion settled" and not a homing reset.
     * Rotary firmware stops the bridge and, if it was SEEK, returns the status to WAIT.
     * Linear firmware only raises DRVOFF; the homing state machine keeps running and
     * can still reach DONE. Call homeResetSlot() before homing again. 0x02 keeps
     * arriving while the link is up; there is no separate disabled bit.
     * If the link is already dead, motors keep the last command.
     */
    bool estop(std::string *err = nullptr);

    std::array<SelfTelem, kSelfMotorNum> selfTelem() const;
    /** Atomic copy; stale data retains values but online/valid flags are cleared. */
    TelemetrySnapshot telemetrySnapshot() const;
    /** Last 0x06 per slot. Inactive when no frame for kProtectStaleMs. */
    std::array<ProtectEvent, kSelfMotorNum> protectTelem() const;
    std::array<MitTelem, kMitMotorNum> mitTelem() const;
    BoardTelem boardTelem() const;
    std::array<TouchSlot, kTouchSlotNum> touch() const;
    uint32_t rxMask() const;
    uint16_t telemSeq() const;
    uint32_t tickMs() const;
    uint64_t rxOk() const;
    Err lastErr() const;
    std::string lastError() const;
    static const char *version() noexcept { return sdkVersion(); }

    std::array<float, kSelfMotorNum> targetSelf() const;
    std::array<MitCmd, kMitMotorNum> targetMit() const;
    std::string linkName() const;

    /**
     * Called on the RX thread after 0x02 hand telemetry.
     * Only getters, target setters and callback setters may be called here.
     * Commands, lifecycle operations and waits are rejected while a callback is
     * running, including calls from another thread. Exceptions are caught.
     */
    void setTelemCallback(TelemCallback cb);
    /** Unexpected link loss; runs on RX, TX or sending caller. Same restrictions as telemetry callbacks.
     * Callback exceptions are caught and reported as Err::CallbackFailed.
     * Keep callbacks short; do not destroy the hand inside a callback.
     */
    void setDisconnectCallback(DisconnectCallback cb);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace a3
