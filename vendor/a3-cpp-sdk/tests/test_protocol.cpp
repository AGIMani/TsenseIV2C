#include "a3/A3.h"
#include "a3/detail/Protocol.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace a3;

namespace {

int g_fails = 0;

#define CHECK(cond)                                                                                  \
    do {                                                                                             \
        if (!(cond)) {                                                                               \
            std::cerr << "FAIL " << __FILE__ << ":" << __LINE__ << "  " << #cond << "\n";            \
            ++g_fails;                                                                               \
        }                                                                                            \
    } while (0)

void wr_u16(uint8_t *p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

void wr_u32(uint8_t *p, uint32_t v) {
    wr_u16(p, static_cast<uint16_t>(v));
    wr_u16(p + 2, static_cast<uint16_t>(v >> 16));
}

void wr_f32(uint8_t *p, float v) { std::memcpy(p, &v, 4); }

float packed_home(uint8_t st) {
    uint8_t raw[4] = {0, 0, 0, st};
    float v = 0.f;
    std::memcpy(&v, raw, 4);
    return v;
}

class ReplayTransport : public ITransport {
public:
    bool open() override {
        std::lock_guard<std::mutex> lock(mu_);
        open_ = true;
        return true;
    }
    void close() override {
        std::lock_guard<std::mutex> lock(mu_);
        open_ = false;
    }
    bool send(const uint8_t *p, size_t n) override {
        std::lock_guard<std::mutex> lock(mu_);
        if (!open_) return false;
        if (fail_tx_at_ > 0 && tx_n_ + 1 == fail_tx_at_) {
            ++tx_n_;
            return false;
        }
        last_tx_.assign(p, p + n);
        ++tx_n_;
        return true;
    }
    size_t recv(uint8_t *p, size_t n, int) override {
        std::lock_guard<std::mutex> lock(mu_);
        if (!open_ || rx_.empty() || n == 0) return 0;
        const size_t k = std::min(n, rx_.size());
        std::memcpy(p, rx_.data(), k);
        rx_.erase(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(k));
        return k;
    }
    bool isOpen() const override {
        std::lock_guard<std::mutex> lock(mu_);
        return open_;
    }
    std::string name() const override { return "replay"; }

    void push(const std::vector<uint8_t> &f) {
        std::lock_guard<std::mutex> lock(mu_);
        rx_.insert(rx_.end(), f.begin(), f.end());
    }
    std::vector<uint8_t> lastTx() const {
        std::lock_guard<std::mutex> lock(mu_);
        return last_tx_;
    }
    int txCount() const {
        std::lock_guard<std::mutex> lock(mu_);
        return tx_n_;
    }
    /** 1-based index of a send that should fail. 0 disables it. */
    void failSendNumber(int n) {
        std::lock_guard<std::mutex> lock(mu_);
        fail_tx_at_ = n;
    }

private:
    mutable std::mutex mu_;
    bool open_ = false;
    std::vector<uint8_t> rx_;
    std::vector<uint8_t> last_tx_;
    int tx_n_ = 0;
    int fail_tx_at_ = 0;
};

std::vector<uint8_t> make_hand_telem(uint32_t rx_mask, int slot, float homing, float pos = 1.25f) {
    std::vector<uint8_t> pl(kHandTelemSelfLen, 0);
    wr_u16(pl.data(), 7);
    wr_u16(pl.data() + 2, 0);
    wr_u32(pl.data() + 4, 1234);
    wr_u32(pl.data() + 8, rx_mask);
    uint8_t *off = pl.data() + kHandTelemHdrLen + slot * kHandTelemSlotBytes;
    wr_f32(off, homing);
    wr_f32(off + 4, pos);
    wr_f32(off + 8, 0.1f);
    wr_f32(off + 12, 0.2f);
    wr_f32(off + 16, pos);
    wr_u16(off + 20, 0);
    wr_u16(off + 22, 0x7FFF);
    return packAa55(kFunHandTelem, pl);
}

void test_protect() {
    uint8_t bad[] = {0, 5, 0, 0, 4, 0, 0, 0};
    uint8_t undef[] = {0, 5, 2, 1, 4, 0, 0, 0};
    uint8_t ok[] = {0, 5, 1, 0, 4, 0, 0, 0};
    uint8_t both[] = {2, 5, 1, 1, 0xFF, 0, 0, 0};
    ProtectEvent ev;
    CHECK(!parseProtect(bad, sizeof(bad), &ev));
    CHECK(!parseProtect(undef, sizeof(undef), &ev));
    CHECK(parseProtect(ok, sizeof(ok), &ev));
    CHECK(ev.active && ev.over_temp && !ev.over_current && ev.slot == 4 && ev.motor_id == 5);
    CHECK(fmtProtect(true, false) == "过温");
    CHECK(parseProtect(both, sizeof(both), &ev));
    CHECK(ev.slot == 17 && ev.over_temp && ev.over_current);
    CHECK(fmtProtect(true, true) == "过温+过流");

    A3HandApi hand;
    auto tr = std::make_unique<ReplayTransport>();
    auto *raw = tr.get();
    CHECK(hand.attach(std::move(tr)));
    raw->push(packAa55(kFunProtect, std::vector<uint8_t>(ok, ok + sizeof(ok))));
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    auto got = hand.protectTelem();
    CHECK(got[4].active && got[4].over_temp && !got[0].active);
    hand.disconnect();
}

void test_slots() {
    CHECK(slotIsRotary(6) && slotIsRotary(7) && slotIsRotary(14) && slotIsRotary(15));
    CHECK(!slotIsRotary(0) && !slotIsRotary(5) && !slotIsRotary(8) && !slotIsRotary(16));
    CHECK(std::string(slotUnit(0)) == "mm");
    CHECK(std::string(slotUnit(6)) == "deg");
    CHECK(std::string(slotKind(16)) == "linear");
    uint8_t bus = 0xFF, node = 0xFF;
    CHECK(slotToBusNode(0, &bus, &node) && bus == 0 && node == 1);
    CHECK(slotToBusNode(5, &bus, &node) && bus == 0 && node == 6);
    CHECK(slotToBusNode(6, &bus, &node) && bus == 0 && node == 7);
    CHECK(slotToBusNode(8, &bus, &node) && bus == 1 && node == 1);
    CHECK(slotToBusNode(14, &bus, &node) && bus == 1 && node == 7);
    CHECK(slotToBusNode(16, &bus, &node) && bus == 2 && node == 1);
    CHECK(slotToBusNode(17, &bus, &node) && bus == 2 && node == 2);
    CHECK(busNodeToSlot(0, 1) == 0);
    CHECK(busNodeToSlot(1, 8) == 15);
    CHECK(busNodeToSlot(2, 1) == 16);
    CHECK(busNodeToSlot(2, 2) == 17);
    CHECK(busNodeToSlot(2, 5) == 17);
    CHECK(busNodeToSlot(2, 3) < 0);
    CHECK(!slotToBusNode(18, &bus, &node));
    auto lin1 = busKindNodes(0, false);
    CHECK(lin1.size() == 6 && lin1[0] == 1 && lin1[5] == 6);
    auto rot1 = busKindNodes(0, true);
    CHECK(rot1.size() == 2 && rot1[0] == 7 && rot1[1] == 8);
    CHECK(busKindNodes(2, true).empty());
    CHECK(busKindNodes(2, false).size() == 2);
}

void test_homing() {
    SelfTelem s;
    s.online = true;
    s.homing_or_theta = packed_home(0);
    CHECK(homingState(s.homing_or_theta) == HomingState::Wait);
    CHECK(!homingIsDone(s) && !homingFailed(s));
    s.homing_or_theta = packed_home(1);
    CHECK(homingState(s.homing_or_theta) == HomingState::Seek);
    s.homing_or_theta = packed_home(2);
    CHECK(homingIsDone(s));
    s.homing_or_theta = packed_home(3);
    CHECK(homingFailed(s) && !homingIsDone(s));
    s.homing_or_theta = 1.25f;
    CHECK(homingState(s.homing_or_theta) == HomingState::Theta);
    CHECK(homingIsDone(s));
    float theta_lsb = 0.f;
    const uint8_t theta_bytes[4] = {0x01, 0x00, 0x80, 0x3F};
    std::memcpy(&theta_lsb, theta_bytes, 4);
    CHECK(homingState(theta_lsb) == HomingState::Theta);
    CHECK(decodeHomingOrTheta(theta_lsb).find("θ=") == 0);
    float spin = 0.f;
    const uint8_t spin_bytes[4] = {0x01, 100, 0x00, 0x02};
    std::memcpy(&spin, spin_bytes, 4);
    s.homing_or_theta = spin;
    CHECK(homingState(s.homing_or_theta) == HomingState::OpenSpin);
    CHECK(!homingIsDone(s));
    s.online = false;
    CHECK(!homingIsDone(s));
    CHECK(decodeHomingOrTheta(packed_home(0)) == "WAIT");
    CHECK(decodeHomingOrTheta(packed_home(2)) == "DONE");
}

void test_crc_and_pack() {
    const char *abc = "123456789";
    CHECK(crc16Modbus(reinterpret_cast<const uint8_t *>(abc), 9) == 0x4B37);

    auto pwr = packSelfPwr(0, 3, kCmdEnable, kPwrDrvAll);
    CHECK(pwr.size() == 12);
    CHECK(pwr[0] == 0xAA && pwr[1] == 0x55);
    CHECK(pwr[4] == kDevTelem && pwr[5] == kFunSelfPwr);
    CHECK(pwr[6] == 0 && pwr[7] == 3 && pwr[8] == 1 && pwr[9] == 3);
    CHECK(crc16Modbus(pwr.data(), pwr.size() - 2) ==
          static_cast<uint16_t>(pwr[pwr.size() - 2] | (pwr[pwr.size() - 1] << 8)));

    auto home = packSelfHoming(1, 8, kCmdHome, 80.f);
    CHECK(home.size() == 20 && home[5] == kFunSelfHoming && home[8] == kCmdHome);
    float iq = 0.f;
    std::memcpy(&iq, home.data() + 10, 4);
    CHECK(iq == 80.f);

    auto rst = packSelfHoming(0, 1, kCmdHomeReset, 0.f);
    CHECK(rst[8] == kCmdHomeReset);

    auto pid = packSelfPosPid(0, 7, 400.f, 0.f, 1.f);
    CHECK(pid.size() == 24 && pid[5] == kFunSelfPosPid);
    auto cur = packSelfCurPid(0, 1, 0.4f, 0.f, 0.f, 0.4f, 0.f, 0.f);
    CHECK(cur.size() == 36 && cur[5] == kFunSelfCurPid);
    auto spin = packSelfOpenSpin(0, 2, kCmdSpinStart, 2.f, 1.f);
    CHECK(spin.size() == 20 && spin[5] == kFunSelfOpenSpin);
    uint8_t td[] = {1};
    auto touch = packTouchCmd(0, kTouchCmdMode, td, 1);
    CHECK(touch[5] == kFunTouchCmd && touch[6] == 0 && touch[7] == kTouchCmdMode);

    float self[kSelfMotorNum]{};
    self[6] = 12.5f;
    float mz[kMitMotorNum]{};
    uint8_t wire[kHandBatchWireLen];
    packHandBatch(wire, 9, self, mz, mz, mz);
    CHECK(wire[0] == 0xAA && wire[5] == kFunHandPosBatch);
    CHECK((wire[2] | (wire[3] << 8)) == kHandBatchPayloadLen);
    CHECK(wire[6] == 9);
    float slot6 = 0.f;
    std::memcpy(&slot6, wire + 7 + 6 * 4, 4);
    CHECK(slot6 == 12.5f);
    CHECK(crc16Modbus(wire, 106) == static_cast<uint16_t>(wire[106] | (wire[107] << 8)));
}

void test_pop_and_parse() {
    auto one = packSelfPwr(0, 1, kCmdDisable, kPwrDrvAll);
    std::vector<uint8_t> buf = {0x00, 0x11};
    buf.insert(buf.end(), one.begin(), one.end());
    buf.insert(buf.end(), one.begin(), one.end());
    auto frames = popAa55Frames(buf);
    CHECK(frames.size() == 2);
    CHECK(buf.empty());
    FrameView fv{};
    CHECK(parseFrame(frames[0], &fv) && fv.fun == kFunSelfPwr && fv.plen == 4);

    auto bad = one;
    bad[bad.size() - 1] ^= 0xFF;
    buf = bad;
    auto none = popAa55Frames(buf);
    CHECK(none.empty());

    uint8_t pl[kHandTelemSelfLen]{};
    wr_u16(pl, 3);
    wr_u32(pl + 8, 1u | (1u << 6));
    wr_f32(pl + kHandTelemHdrLen, packed_home(2));
    wr_f32(pl + kHandTelemHdrLen + 4, 3.5f);
    wr_f32(pl + kHandTelemHdrLen + 8, 80.f);
    wr_f32(pl + kHandTelemHdrLen + 12, -40.f);
    wr_f32(pl + kHandTelemHdrLen + 6 * kHandTelemSlotBytes + 8, 80.f);
    std::array<SelfTelem, kSelfMotorNum> self{};
    std::array<MitTelem, kMitMotorNum> mit{};
    uint16_t seq = 0;
    uint32_t tick = 0;
    CHECK(parseHandTelem(pl, sizeof(pl), &self, &mit, &seq, &tick));
    CHECK(seq == 3 && self[0].online && homingIsDone(self[0]));
    CHECK(self[0].pos_act == 3.5f);
    CHECK(std::fabs(self[0].iq_meas - static_cast<float>(80.0 * kLinearIqAmpPerCount)) < 1e-6f);
    CHECK(std::fabs(self[0].iq_ref - static_cast<float>(-40.0 * kLinearIqAmpPerCount)) < 1e-6f);
    CHECK(std::fabs(self[6].iq_meas - static_cast<float>(80.0 / kRotaryIqCountsPerAmp)) < 1e-6f);
    CHECK(!self[1].online);

    uint8_t board[8]{};
    wr_u16(board, 1);
    wr_u16(board + 2, 3650);
    wr_u16(board + 4, 3820);
    wr_u16(board + 6, 24120);
    BoardTelem b;
    CHECK(parseBoardTelem(board, 8, &b) && b.valid);
    CHECK(b.ntc_c_centi == 3650);
    CHECK(b.ntc_b_centi == 3820);
    CHECK(b.vin24_mV == 24120);
}

void test_wait_estop() {
    {
        A3HandApi hand;
        std::string err;
        CHECK(!hand.waitOnline(1, 20, &err));
        CHECK(!err.empty());
        CHECK(!hand.estop(&err));
    }
    {
        A3HandApi hand;
        auto tr = std::make_unique<ReplayTransport>();
        auto *raw = tr.get();
        CHECK(hand.attach(std::move(tr)));
        std::string err;
        CHECK(!hand.waitOnline(1u, 80, &err));
        raw->push(make_hand_telem(1u, 0, packed_home(2), 0.f));
        CHECK(hand.waitOnline(1u, 1000, &err));
        CHECK(hand.waitHomeDone(0, 1000, &err));
        CHECK(hand.estop(&err));
        CHECK(raw->txCount() >= 1);
        auto tx = raw->lastTx();
        FrameView fv{};
        CHECK(parseFrame(tx, &fv) && fv.fun == kFunSelfPwr);
        CHECK(fv.payload[0] == kBusAll && fv.payload[1] == kNodeAll && fv.payload[2] == kCmdDisable);
        CHECK(hand.homeResetSlot(6, &err));
        tx = raw->lastTx();
        CHECK(parseFrame(tx, &fv) && fv.fun == kFunSelfHoming && fv.payload[2] == kCmdHomeReset);
        CHECK(hand.homeSlot(0, IqSeekCounts{80.f}, &err));
        tx = raw->lastTx();
        CHECK(parseFrame(tx, &fv) && fv.fun == kFunSelfHoming);
        float iq = 0.f;
        std::memcpy(&iq, fv.payload + 4, 4);
        CHECK(iq == 80.f);
        hand.disconnect();
    }
    {
        A3HandApi hand;
        auto tr = std::make_unique<ReplayTransport>();
        auto *raw = tr.get();
        CHECK(hand.attach(std::move(tr)));
        raw->push(make_hand_telem(1u, 0, packed_home(3)));
        std::string err;
        CHECK(!hand.waitHomeDone(0, 1000, &err));
        CHECK(err.find("homing failed") != std::string::npos);
        hand.disconnect();
    }
}

std::vector<uint8_t> make_host_info(uint8_t proto_maj, uint8_t proto_min, uint8_t app_maj,
                                    uint8_t app_min = 0, uint8_t app_pat = 0, uint8_t app_bld = 0) {
    uint8_t pl[kHostInfoPayloadLen] = {proto_maj, proto_min, app_maj, app_min, app_pat, app_bld, 0, 0};
    return packAa55(kFunHostInfo, pl, kHostInfoPayloadLen);
}

void test_product_api() {
    CHECK(clampSlotPos(0, -1.f) == 0.f);
    CHECK(clampSlotPos(0, 99.f) == kDipPosMaxMm);
    CHECK(clampSlotPos(1, 99.f) == kPinkyPipPosMaxMm);
    CHECK(clampSlotPos(4, 99.f) == kPipPosMaxMm);
    CHECK(clampSlotPos(2, 99.f) == kMcpPosMaxMm);
    CHECK(clampSlotPos(16, 99.f) == kDipPosMaxMm);
    CHECK(clampSlotPos(17, 20.f) == kPipPosMaxMm);
    CHECK(clampSlotPos(6, 200.f) == kRotaryPosMaxDeg);
    CHECK(std::string(errMessage(Err::NotConnected)) == "not connected");
    {
        A3HandApi hand;
        float pos[kSelfMotorNum]{};
        pos[0] = 99.f;
        pos[1] = 99.f;
        pos[17] = 99.f;
        pos[6] = -200.f;
        hand.setPositions(pos);
        auto t = hand.targetSelf();
        CHECK(t[0] == kDipPosMaxMm);
        CHECK(t[1] == kPinkyPipPosMaxMm);
        CHECK(t[17] == kPipPosMaxMm);
        CHECK(t[6] == kRotaryPosMinDeg);
        float ang[kJointNum];
        for (int i = 0; i < kJointNum; ++i) ang[i] = 20.f;
        float act[kJointNum];
        CHECK(anglesDegToActuator(ang, act));
        CHECK(std::fabs(act[2] - 3.694408f) < 2e-4f);
        CHECK(std::fabs(act[4] - 4.669581f) < 2e-4f);
        CHECK(std::fabs(act[4] - act[8]) < 1e-4f);
        CHECK(std::fabs(act[6] - act[2]) < 1e-4f);
        CHECK(std::fabs(act[3] - act[19]) < 1e-4f);
        CHECK(std::fabs(act[19] - 2.858646f) < 2e-4f);
        CHECK(std::fabs(act[7] - act[19]) < 1e-4f);
        CHECK(std::fabs(act[18] - 2.858649f) < 2e-4f);
        CHECK(std::fabs(act[5] - 20.f) < 1e-3f);
        float ang90[kJointNum]{};
        const int linear_joints[] = {2, 3, 4, 6, 7, 8, 10, 11, 12, 14, 15, 16, 18, 19};
        for (int idx : linear_joints) ang90[idx] = 90.f;
        float act90[kJointNum];
        CHECK(anglesDegToActuator(ang90, act90));
        CHECK(std::fabs(act90[19] - 12.437477f) < 1e-4f);
        CHECK(std::fabs(act90[11] - act90[19]) < 1e-4f);
        CHECK(std::fabs(act90[3] - act90[19]) < 1e-4f);
        CHECK(std::fabs(act90[18] - 11.761490f) < 1e-4f);
        CHECK(std::fabs(act90[2] - 16.939022f) < 1e-4f);
        CHECK(std::fabs(act90[6] - act90[2]) < 1e-4f);
        CHECK(std::fabs(act90[4] - 19.627779f) < 1e-4f);
        CHECK(act90[19] <= kDipPosMaxMm + 1e-4f);
        CHECK(act90[18] <= kPinkyPipPosMaxMm + 1e-4f);
        CHECK(act90[2] <= kPipPosMaxMm + 1e-4f);
        CHECK(act90[4] <= kMcpPosMaxMm + 1e-4f);
        ang90[2] = 120.f;
        float act_over[kJointNum];
        CHECK(anglesDegToActuator(ang90, act_over));
        CHECK(std::fabs(act_over[2] - 16.939022f) < 1e-4f);
        float ang_edge[kJointNum]{};
        for (int idx : linear_joints) ang_edge[idx] = 0.f;
        float act_zero[kJointNum];
        CHECK(anglesDegToActuator(ang_edge, act_zero));
        CHECK(act_zero[2] == 0.f);
        CHECK(act_zero[18] == 0.f);
        CHECK(act_zero[19] == 0.f);
        for (int idx : linear_joints) ang_edge[idx] = 0.005f;
        float act_min[kJointNum];
        CHECK(anglesDegToActuator(ang_edge, act_min));
        CHECK(act_min[2] > act_zero[2]);
        CHECK(act_min[19] > act_zero[19]);
        float back[kJointNum];
        CHECK(actuatorToAnglesDeg(act, back));
        CHECK(std::fabs(back[2] - 20.f) < 1e-3f);
        CHECK(std::fabs(back[18] - 20.f) < 1e-3f);
        CHECK(std::fabs(back[19] - 20.f) < 1e-3f);
        CHECK(std::fabs(back[5] - 20.f) < 1e-3f);
        float fb[kJointNum]{};
        float recovered[kJointNum];
        CHECK(actuatorToAnglesDeg(fb, recovered));
        CHECK(std::fabs(recovered[2]) < 1e-3f);
        CHECK(std::fabs(recovered[6]) < 1e-3f);
        CHECK(std::fabs(recovered[18]) < 1e-3f);
        CHECK(std::fabs(recovered[19]) < 1e-3f);
        fb[2] = kPipPosMaxMm;
        fb[6] = kPipPosMaxMm;
        fb[18] = kPinkyPipPosMaxMm;
        fb[19] = kDipPosMaxMm;
        CHECK(actuatorToAnglesDeg(fb, recovered));
        CHECK(std::fabs(recovered[2] - 90.f) < 1e-3f);
        CHECK(std::fabs(recovered[6] - 90.f) < 1e-3f);
        CHECK(std::fabs(recovered[18] - 90.f) < 1e-3f);
        CHECK(std::fabs(recovered[19] - 90.f) < 1e-3f);
        const float kept = recovered[6];
        fb[6] = kPipPosMaxMm + 0.05f;
        CHECK(!actuatorToAnglesDeg(fb, recovered));
        CHECK(recovered[6] == kept);
        fb[6] = kPipPosMaxMm;
        fb[2] = -1e-5f;
        CHECK(actuatorToAnglesDeg(fb, recovered));
        fb[2] = -2e-4f;
        CHECK(!actuatorToAnglesDeg(fb, recovered));
        ang[5] = 40.f;
        CHECK(!anglesDegToActuator(ang, act));
        ang[5] = 20.f;
        CHECK(hand.setJointAnglesDeg(ang));
        t = hand.targetSelf();
        CHECK(std::fabs(t[0] - 2.858646f) < 2e-4f);
        CHECK(std::fabs(hand.targetMit()[1].pos_out_deg - 20.f) < 1e-3f);
        std::string err;
        CHECK(!hand.startStream(500, &err));
        CHECK(hand.lastErr() == Err::NotConnected);
    }
    {
        A3HandApi hand;
        auto tr = std::make_unique<ReplayTransport>();
        auto *raw = tr.get();
        CHECK(hand.attach(std::move(tr)));
        std::string err;
        CHECK(!hand.handshake(80, &err));
        CHECK(hand.lastErr() == Err::FwTimeout);
        raw->push(make_host_info(1, 0, 1, 0, 0, 0));
        CHECK(hand.handshake(1000, &err));
        CHECK(fwCompatible(hand.firmware()));
        CHECK(hand.firmware().hand_side == HandSide::Unset);
        CHECK(hand.startStream(200, &err));
        CHECK(hand.streaming());
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        hand.stopStream();
        CHECK(!hand.streaming());
        hand.disconnect();
        CHECK(!hand.startStream(200, &err));
    }
    {
        A3HandApi hand;
        auto tr = std::make_unique<ReplayTransport>();
        auto *raw = tr.get();
        CHECK(hand.attach(std::move(tr)));
        uint8_t info[kHostInfoPayloadLen] = {1, 0, 1, 0, 0, 0, 2, 0};
        raw->push(packAa55(kFunHostInfo, info, kHostInfoPayloadLen));
        std::string err;
        CHECK(hand.handshake(1000, &err));
        CHECK(hand.firmware().hand_side == HandSide::Left);
        HandSide side = HandSide::Unset;
        CHECK(!hand.queryHandSide(&side, 80, &err));
        CHECK(hand.lastErr() == Err::Timeout);
        const int tx_before = raw->txCount();
        std::thread reply([&] {
            for (int i = 0; i < 100; ++i) {
                if (raw->txCount() > tx_before) {
                    uint8_t rsp[2] = {0, 1};
                    raw->push(packAa55(kFunHandSide, rsp, 2));
                    return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        });
        CHECK(hand.queryHandSide(&side, 1000, &err));
        reply.join();
        CHECK(side == HandSide::Right);
        CHECK(hand.firmware().hand_side == HandSide::Right);
        FrameView fv;
        CHECK(parseFrame(raw->lastTx(), &fv));
        CHECK(fv.fun == kFunHandSide);
        CHECK(fv.plen == 2 && fv.payload[0] == 0);
        hand.disconnect();
    }
    {
        A3HandApi hand;
        auto tr = std::make_unique<ReplayTransport>();
        auto *raw = tr.get();
        CHECK(hand.attach(std::move(tr)));
        raw->push(make_host_info(2, 0, 1));
        std::string err;
        CHECK(!hand.handshake(1000, &err));
        CHECK(hand.lastErr() == Err::FwIncompatible);
        hand.disconnect();
    }
}

void test_reconnect_and_home_cycle() {
    A3HandApi hand;
    auto tr = std::make_unique<ReplayTransport>();
    auto *raw = tr.get();
    CHECK(hand.attach(std::move(tr)));
    raw->push(make_hand_telem(1u, 0, 1.25f));
    std::string err;
    CHECK(hand.waitOnline(1u, 1000, &err));
    CHECK(hand.rxMask() == 1u);
    hand.disconnect();
    CHECK(hand.rxMask() == 0u);
    CHECK(!hand.selfTelem()[0].online);

    auto tr2 = std::make_unique<ReplayTransport>();
    auto *raw2 = tr2.get();
    CHECK(hand.attach(std::move(tr2)));
    CHECK(hand.rxMask() == 0u);
    CHECK(!hand.waitOnline(1u, 40, &err));
    raw2->push(make_hand_telem(1u, 0, 1.25f));
    CHECK(hand.waitOnline(1u, 1000, &err));
    CHECK(hand.homeSlot(0, IqSeekCounts{80.f}, &err));
    CHECK(!hand.waitHomeDone(0, 60, &err));
    raw2->push(make_hand_telem(1u, 0, packed_home(1)));
    raw2->push(make_hand_telem(1u, 0, packed_home(2)));
    CHECK(hand.waitHomeDone(0, 1000, &err));
    hand.disconnect();
}

void test_touch_short_payload() {
    uint8_t data[16]{};
    data[4] = 10;
    data[5] = 10;
    data[8] = 1;
    data[9] = 0;
    TouchSlot slot;
    CHECK(parseTouchData(kTouchCmdForce, data, sizeof(data), &slot));
    CHECK(slot.parts[0].rows == 10 && slot.parts[0].cols == 10);
    CHECK(slot.parts[0].vals.size() == 100);
    CHECK(slot.parts[0].ok == 4);
    CHECK(slot.parts[0].vals[0] == 1.f);
    CHECK(std::isnan(slot.parts[0].vals[4]));
}

class FastHomeTransport : public ReplayTransport {
public:
    std::atomic<bool> entered{false}, ready{false};
    size_t recv(uint8_t *p, size_t n, int timeout) override {
        entered = true;
        while (!ready.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return ReplayTransport::recv(p, n, timeout);
    }
    bool send(const uint8_t *p, size_t n) override {
        bool ok = ReplayTransport::send(p, n);
        if (n > 5 && p[5] == kFunSelfHoming) {
            push(make_hand_telem(1u, 0, packed_home(1)));
            push(make_hand_telem(1u, 0, packed_home(2)));
            ready = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
        }
        return ok;
    }
};

void test_fault_regressions() {
    auto valid = packSelfPwr(0, 1, kCmdDisable, kPwrDrvAll);
    std::vector<uint8_t> buf = {0xAA, 0x55, 12, 0, 0, 0, 0, 0};
    buf.insert(buf.end(), valid.begin(), valid.end());
    auto frames = popAa55Frames(buf);
    CHECK(frames.size() == 1 && frames[0] == valid);
    FrameView fv;
    valid[0] = 0;
    CHECK(!parseFrame(valid, &fv));
    CHECK(std::string(sdkVersion()) == "1.1.15");
    {
        A3HandApi hand;
        float pos[kSelfMotorNum]{}; pos[0] = 1.f;
        CHECK(hand.setPositions(pos));
        pos[1] = std::numeric_limits<float>::quiet_NaN();
        CHECK(!hand.setPositions(pos));
        CHECK(hand.lastErr() == Err::InvalidArg);
        CHECK(hand.targetSelf()[0] == 1.f && hand.targetSelf()[1] == 0.f);
        pos[1] = 1.f;
        CHECK(hand.setPositions(pos));
        CHECK(hand.lastErr() == Err::Ok);
        CHECK(hand.targetSelf()[0] == 1.f && hand.targetSelf()[1] == 1.f);
        CHECK(!hand.setMit(-1, 0, 0, 0));
        CHECK(!hand.setMit(0, 0, std::numeric_limits<float>::infinity(), 0));
        CHECK(!hand.setPosPid(0, -1, 0, 0));
        CHECK(!hand.homeSlot(0, IqSeekCounts{-1}));
        CHECK(!hand.sendTouch(0, 2, nullptr, 1));
        CHECK(!hand.sendTouch(0, 2, nullptr, 65532));
        CHECK(!hand.enableBusKind(-1, false, true));
    }
    {
        A3HandApi hand;
        auto tr = std::make_unique<ReplayTransport>(); auto *raw = tr.get();
        CHECK(hand.attach(std::move(tr)));
        raw->push(make_hand_telem(1u, 0, packed_home(3)));
        CHECK(hand.waitOnline(1u, 1000));
        CHECK(hand.homeSlot(0, IqSeekCounts{80}));
        CHECK(!hand.waitHomeDone(0, 20));
        CHECK(hand.lastErr() == Err::Timeout);
        raw->push(make_hand_telem(1u, 0, packed_home(1)));
        raw->push(make_hand_telem(1u, 0, packed_home(2)));
        CHECK(hand.waitHomeDone(0, 1000));
        auto snap = hand.telemetrySnapshot();
        CHECK(snap.fresh && snap.seq == 7 && snap.rx_mask == 1u && snap.self[0].online);
        std::this_thread::sleep_for(std::chrono::milliseconds(kTelemetryStaleMs + 30));
        CHECK(!hand.telemetrySnapshot().fresh && hand.rxMask() == 0 && !hand.selfTelem()[0].online);
        CHECK(!hand.waitHomeDone(0, 0));
        raw->close();
        for (int i=0;i<1000 && hand.connected();++i) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        CHECK(!hand.selfTelem()[0].online);
    }
    {
        A3HandApi hand;
        auto tr = std::make_unique<FastHomeTransport>(); auto *raw = tr.get();
        CHECK(hand.attach(std::move(tr)));
        for (int i=0;i<1000 && !raw->entered.load();++i) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        CHECK(raw->entered.load());
        CHECK(hand.homeSlot(0, IqSeekCounts{80}));
        CHECK(hand.waitHomeDone(0, 1000));
    }
    {
        A3HandApi hand;
        auto tr = std::make_unique<ReplayTransport>(); auto *raw = tr.get();
        CHECK(hand.attach(std::move(tr)));
        hand.setTelemCallback([] { throw std::runtime_error("callback"); });
        raw->push(make_hand_telem(1u, 0, packed_home(2)));
        for (int i=0;i<1000 && hand.lastErr()!=Err::CallbackFailed;++i) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        CHECK(hand.lastErr() == Err::CallbackFailed && hand.connected());
        hand.setTelemCallback([&hand] { hand.disconnect(); });
        raw->push(make_hand_telem(1u, 0, packed_home(2)));
        for (int i=0;i<1000 && hand.lastErr()!=Err::InvalidArg;++i) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        CHECK(hand.lastErr() == Err::InvalidArg && hand.connected());
        hand.setTelemCallback({});
        std::atomic<bool> failed{false};
        auto cycle = [&] {
            for (int i=0;i<30;++i) { if (!hand.startStream(500)) failed=true; hand.stopStream(); }
        };
        std::thread a(cycle), b(cycle); a.join(); b.join();
        CHECK(!failed.load() && !hand.streaming());
        CHECK(hand.startStream(1));
        auto stopped = std::chrono::steady_clock::now();
        hand.stopStream();
        CHECK(std::chrono::steady_clock::now() - stopped < std::chrono::milliseconds(500));
        std::atomic<bool> waiting{false};
        bool wait_result = true;
        std::thread waiter([&] { waiting = true; wait_result = hand.waitOnline(2u, 5000); });
        while (!waiting.load()) std::this_thread::yield();
        stopped = std::chrono::steady_clock::now();
        CHECK(hand.estop());
        hand.disconnect();
        waiter.join();
        CHECK(!wait_result && std::chrono::steady_clock::now() - stopped < std::chrono::milliseconds(500));
    }
    {
        A3HandApi hand;
        auto replace = [&] {
            for (int i=0;i<20;++i) { hand.attach(std::make_unique<ReplayTransport>()); hand.disconnect(); }
        };
        std::thread a(replace), b(replace); a.join(); b.join();
        CHECK(!hand.connected());
        auto tr = std::make_unique<ReplayTransport>(); auto *raw = tr.get();
        CHECK(hand.attach(std::move(tr)));
        hand.setDisconnectCallback([](Err, const std::string &) { throw std::runtime_error("disconnect callback"); });
        raw->close();
        for (int i=0;i<1000 && hand.lastErr()!=Err::CallbackFailed;++i) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        CHECK(hand.lastErr() == Err::CallbackFailed && !hand.connected());
    }
}

bool wait_home_state(A3HandApi &hand, HomingState want) {
    for (int i = 0; i < 1000; ++i) {
        auto s = hand.selfTelem()[0];
        if (s.online && homingState(s.homing_or_theta) == want) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

void test_home_and_stream_edges() {
    {
        A3HandApi hand;
        auto tr = std::make_unique<ReplayTransport>();
        auto *raw = tr.get();
        CHECK(hand.attach(std::move(tr)));
        raw->push(make_hand_telem(1u, 0, packed_home(1)));
        CHECK(wait_home_state(hand, HomingState::Seek));
        CHECK(hand.homeSlot(0, IqSeekCounts{80.f}));
        raw->push(make_hand_telem(1u, 0, packed_home(1)));
        raw->push(make_hand_telem(1u, 0, packed_home(2)));
        std::string err;
        CHECK(!hand.waitHomeDone(0, 80, &err));
        raw->push(make_hand_telem(1u, 0, packed_home(0)));
        raw->push(make_hand_telem(1u, 0, packed_home(1)));
        raw->push(make_hand_telem(1u, 0, packed_home(2)));
        CHECK(hand.waitHomeDone(0, 1000, &err));
        hand.disconnect();
    }
    {
        A3HandApi hand;
        auto tr = std::make_unique<ReplayTransport>();
        auto *raw = tr.get();
        CHECK(hand.attach(std::move(tr)));
        raw->push(make_hand_telem(1u, 0, packed_home(2)));
        CHECK(wait_home_state(hand, HomingState::Done));
        CHECK(hand.homeSlot(0, IqSeekCounts{80.f}));
        CHECK(hand.homeResetSlot(0));
        std::string err;
        CHECK(!hand.waitHomeDone(0, 40, &err));
        raw->push(make_hand_telem(1u, 0, packed_home(2)));
        CHECK(!hand.waitHomeDone(0, 40, &err));
        raw->push(make_hand_telem(1u, 0, packed_home(0)));
        raw->push(make_hand_telem(1u, 0, packed_home(2)));
        CHECK(hand.waitHomeDone(0, 1000, &err));
        hand.disconnect();
    }
    {
        A3HandApi hand;
        auto tr = std::make_unique<ReplayTransport>();
        auto *raw = tr.get();
        CHECK(hand.attach(std::move(tr)));
        raw->failSendNumber(2);
        std::string err;
        CHECK(!hand.homeBusKind(0, true, IqSeekCounts{80.f}, &err));
        CHECK(raw->txCount() == 3);
        auto tx = raw->lastTx();
        FrameView fv{};
        CHECK(parseFrame(tx, &fv) && fv.fun == kFunSelfHoming && fv.payload[2] == kCmdHomeReset);
        CHECK(fv.payload[0] == 0 && fv.payload[1] == 7);
        hand.disconnect();
    }
    {
        A3HandApi hand;
        auto tr = std::make_unique<ReplayTransport>();
        auto *raw = tr.get();
        CHECK(hand.attach(std::move(tr)));
        std::atomic<bool> rejected{false};
        hand.setTelemCallback([&] {
            std::thread child([&] {
                hand.disconnect();
                rejected = hand.lastErr() == Err::InvalidArg;
            });
            child.join();
        });
        raw->push(make_hand_telem(1u, 0, packed_home(2)));
        for (int i = 0; i < 1000 && !rejected.load(); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        CHECK(rejected.load() && hand.connected());
        hand.setTelemCallback({});
        hand.disconnect();
    }
}

} // namespace

int main() {
    test_protect();
    test_slots();
    test_homing();
    test_crc_and_pack();
    test_pop_and_parse();
    test_wait_estop();
    test_product_api();
    test_reconnect_and_home_cycle();
    test_touch_short_payload();
    test_fault_regressions();
    test_home_and_stream_edges();
    if (g_fails) {
        std::cerr << g_fails << " checks failed\n";
        return 1;
    }
    std::cout << "a3_test_protocol OK\n";
    return 0;
}
