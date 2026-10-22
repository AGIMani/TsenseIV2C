#include "a3/A3HandApi.h"

#include "a3/Stroke.h"
#include "a3/detail/Protocol.h"
#include "a3/Tcp5001.h"
#include "a3/UsbCdc.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <limits>
#include <cstring>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")
#endif

namespace a3 {
namespace {
thread_local bool in_callback = false;
thread_local int lifecycle_internal = 0;
thread_local bool keep_link_on_send_error = false;
struct InternalLifecycle {
    InternalLifecycle() { ++lifecycle_internal; }
    ~InternalLifecycle() { --lifecycle_internal; }
    InternalLifecycle(const InternalLifecycle &) = delete;
    InternalLifecycle &operator=(const InternalLifecycle &) = delete;
};
struct CallbackScope {
    std::atomic<int> *depth = nullptr;
    bool previous = in_callback;
    explicit CallbackScope(std::atomic<int> *d) : depth(d) {
        in_callback = true;
        if (depth) depth->fetch_add(1, std::memory_order_acq_rel);
    }
    ~CallbackScope() {
        in_callback = previous;
        if (depth) depth->fetch_sub(1, std::memory_order_acq_rel);
    }
    CallbackScope(const CallbackScope &) = delete;
    CallbackScope &operator=(const CallbackScope &) = delete;
};
bool homingActive(HomingState st) {
    return st == HomingState::Seek || st == HomingState::Release;
}
bool validAddress(uint8_t bus, uint8_t node, bool broadcast) {
    return (broadcast && bus == kBusAll && node == kNodeAll) ||
        (bus <= 2 && ((broadcast && node == kNodeAll) || busNodeToSlot(bus, node) >= 0));
}
bool finiteMit(const MitCmd &c) {
    return std::isfinite(c.pos_out_deg) && std::isfinite(c.vel_out_rpm) &&
        std::isfinite(c.iq_a) && std::isfinite(c.vel_out_rpm * kMitGearRatio);
}

class TimerRes {
public:
    TimerRes() {
#ifdef _WIN32
        timeBeginPeriod(1);
#endif
    }
    ~TimerRes() {
#ifdef _WIN32
        timeEndPeriod(1);
#endif
    }
};


} // namespace

struct A3HandApi::Impl {
    std::recursive_mutex lifecycle_mu_;
    std::mutex stream_wait_mu_;
    std::condition_variable stream_wait_cv_;
    std::atomic<uint64_t> session_{0};
    std::atomic<int> callback_depth_{0};
    mutable std::mutex io_mu_;
    mutable std::mutex state_mu_;
    std::mutex cb_mu_;
    std::unique_ptr<ITransport> transport_;
    std::thread rx_th_;
    std::thread tx_th_;
    std::atomic<bool> run_{false};
    std::atomic<bool> streaming_{false};
    std::atomic<bool> lost_{false};
    std::atomic<int> stream_hz_{500};
    std::atomic<uint64_t> rx_ok_{0};
    std::atomic<uint64_t> tx_ok_{0};
    std::atomic<double> rx_hz_{0};
    std::atomic<double> tx_hz_{0};
    std::atomic<uint8_t> seq_{0};

    std::array<float, kSelfMotorNum> self_pos_{};
    std::array<MitCmd, kMitMotorNum> mit_{};
    std::array<SelfTelem, kSelfMotorNum> self_telem_{};
    std::array<MitTelem, kMitMotorNum> mit_telem_{};
    std::array<ProtectEvent, kSelfMotorNum> protect_{};
    std::array<std::chrono::steady_clock::time_point, kSelfMotorNum> protect_t_{};
    BoardTelem board_{};
    std::array<TouchSlot, kTouchSlotNum> touch_{};
    FwInfo fw_{};
    HandSide hand_side_ = HandSide::Unset;
    uint8_t hand_side_status_ = 0;
    uint32_t hand_query_epoch_ = 0;
    uint32_t hand_reply_epoch_ = 0;
    std::chrono::steady_clock::time_point hand_t_{}, board_t_{};
    std::array<std::chrono::steady_clock::time_point, kTouchSlotNum> touch_t_{};
    uint64_t hand_generation_ = 0;
    std::array<uint64_t, kSelfMotorNum> home_generation_{};
    std::array<bool, kSelfMotorNum> home_need_seek_{};
    std::array<bool, kSelfMotorNum> home_seen_seek_{};
    /** Previous SEEK/RELEASE must end before the next one counts for this command. */
    std::array<bool, kSelfMotorNum> home_need_gap_{};
    /** After a successful reset, ignore telemetry until the homing state changes. */
    std::array<bool, kSelfMotorNum> home_reset_hold_{};
    std::array<uint64_t, kSelfMotorNum> home_reset_gen_{};
    std::array<HomingState, kSelfMotorNum> home_reset_state_{};
    uint32_t rx_mask_ = 0;
    uint16_t telem_seq_ = 0;
    uint32_t tick_ms_ = 0;
    Err last_code_ = Err::Ok;
    std::string last_err_;
    std::string link_name_;
    std::vector<uint8_t> rx_buf_;
    TelemCallback telem_cb_;
    DisconnectCallback disconnect_cb_;

    bool fail(std::string *err, Err code, std::string extra = {}) {
        std::lock_guard<std::mutex> lock(state_mu_);
        last_code_ = code;
        last_err_ = extra.empty() ? std::string(errMessage(code)) : std::move(extra);
        if (err) *err = last_err_;
        return false;
    }

    void clearError() {
        std::lock_guard<std::mutex> lock(state_mu_);
        last_code_ = Err::Ok;
        last_err_.clear();
    }

    /** Drop online/telemetry from the previous link. Position targets stay. */
    void clearLinkState() {
        std::lock_guard<std::mutex> lock(state_mu_);
        self_telem_ = {};
        mit_telem_ = {};
        protect_ = {};
        protect_t_ = {};
        board_ = {};
        touch_ = {};
        fw_ = {};
        hand_side_ = HandSide::Unset;
        hand_side_status_ = 0;
        hand_query_epoch_ = 0;
        hand_reply_epoch_ = 0;
        hand_t_ = {}; board_t_ = {}; touch_t_ = {};
        home_generation_ = {};
        hand_generation_ = 0;
        home_need_seek_ = {};
        home_seen_seek_ = {};
        home_need_gap_ = {};
        home_reset_hold_ = {};
        home_reset_gen_ = {};
        home_reset_state_ = {};
        rx_mask_ = 0;
        telem_seq_ = 0;
        tick_ms_ = 0;
    }

    void noteHome(uint8_t bus, uint8_t node) {
        const int slot = busNodeToSlot(bus, node);
        if (slot < 0) return;
        std::lock_guard<std::mutex> lock(state_mu_);
        const size_t i = static_cast<size_t>(slot);
        const HomingState st = homingState(self_telem_[i].homing_or_theta);
        home_need_seek_[i] = true;
        home_seen_seek_[i] = false;
        home_need_gap_[i] = self_telem_[i].online && homingActive(st);
        home_reset_hold_[i] = false;
        home_generation_[i] = hand_generation_;
    }

    /** Caller holds state_mu_. A SEEK already running at noteHome() does not count. */
    void observeHomeLocked(int slot) {
        const size_t i = static_cast<size_t>(slot);
        if (home_reset_hold_[i] || !self_telem_[i].online) return;
        if (home_need_seek_[i] && hand_generation_ <= home_generation_[i]) return;
        const HomingState st = homingState(self_telem_[i].homing_or_theta);
        const bool active = homingActive(st);
        if (home_need_gap_[i]) {
            if (active) return;
            home_need_gap_[i] = false;
        }
        if (!home_need_seek_[i] || !active) return;
        home_need_seek_[i] = false;
        home_seen_seek_[i] = true;
    }

    void noteReset(uint8_t bus, uint8_t node) {
        const int slot = busNodeToSlot(bus, node);
        if (slot < 0) return;
        std::lock_guard<std::mutex> lock(state_mu_);
        const size_t i = static_cast<size_t>(slot);
        home_need_seek_[i] = false;
        home_seen_seek_[i] = false;
        home_need_gap_[i] = false;
        home_reset_hold_[i] = true;
        home_reset_gen_[i] = hand_generation_;
        home_reset_state_[i] = homingState(self_telem_[i].homing_or_theta);
    }

    /** 1 = done, 0 = keep waiting, -1 = homing failed. */
    int pollHome(int slot) {
        std::lock_guard<std::mutex> lock(state_mu_);
        const auto &s = self_telem_[static_cast<size_t>(slot)];
        const size_t i = static_cast<size_t>(slot);
        if (lost_.load() || hand_t_ == std::chrono::steady_clock::time_point{} ||
            std::chrono::steady_clock::now() - hand_t_ > std::chrono::milliseconds(kTelemetryStaleMs)) return 0;
        if (home_reset_hold_[i]) {
            if (hand_generation_ <= home_reset_gen_[i] || !s.online) return 0;
            if (homingState(s.homing_or_theta) == home_reset_state_[i]) return 0;
            home_reset_hold_[i] = false;
        }
        if (home_need_seek_[i] && hand_generation_ <= home_generation_[i]) return 0;
        if (homingFailed(s)) {
            home_need_seek_[i] = false;
            home_seen_seek_[i] = false;
            home_need_gap_[i] = false;
            last_code_ = Err::HomeFailed;
            last_err_ = errMessage(Err::HomeFailed);
            return -1;
        }
        if (!s.online) return 0;
        observeHomeLocked(slot);
        if (home_seen_seek_[i]) {
            const HomingState st = homingState(s.homing_or_theta);
            if (st != HomingState::Done && st != HomingState::Theta) return 0;
            home_seen_seek_[i] = false;
            return 1;
        }
        if (home_need_seek_[i]) return 0;
        return homingIsDone(s) ? 1 : 0;
    }

    void notifyDisconnect(Err code, const std::string &reason) {
        if (lost_.exchange(true)) return;
        streaming_ = false;
        clearLinkState();
        tx_hz_ = 0; rx_hz_ = 0;
        {
            std::lock_guard<std::mutex> lock(state_mu_);
            last_code_ = code;
            last_err_ = reason;
        }
        {
            std::lock_guard<std::mutex> lock(io_mu_);
            if (transport_) transport_->close();
        }
        DisconnectCallback cb;
        {
            std::lock_guard<std::mutex> lock(cb_mu_);
            cb = disconnect_cb_;
        }
        if (cb) {
            CallbackScope scope(&callback_depth_);
            try { cb(code, reason); }
            catch (...) { fail(nullptr, Err::CallbackFailed); }
        }
    }

    bool transportOpen() const {
        std::lock_guard<std::mutex> lock(io_mu_);
        return transport_ && transport_->isOpen();
    }

    bool sendRaw(const std::vector<uint8_t> &frame, std::string *err) {
        std::lock_guard<std::mutex> lock(io_mu_);
        if (!transport_ || !transport_->isOpen()) {
            return fail(err, Err::NotConnected);
        }
        if (!transport_->send(frame.data(), frame.size())) {
            if (!keep_link_on_send_error) transport_->close();
            return fail(err, Err::Io);
        }
        clearError();
        return true;
    }

    bool packAndSendBatch(std::string *err) {
        float self[kSelfMotorNum];
        float mpos[kMitMotorNum];
        float mvel[kMitMotorNum];
        float miq[kMitMotorNum];
        {
            std::lock_guard<std::mutex> lock(state_mu_);
            std::memcpy(self, self_pos_.data(), sizeof(self));
            for (int i = 0; i < kMitMotorNum; ++i) {
                mpos[i] = mitOutToMotor(mit_[static_cast<size_t>(i)].pos_out_deg);
                mvel[i] = mitOutToMotor(mit_[static_cast<size_t>(i)].vel_out_rpm);
                miq[i] = mit_[static_cast<size_t>(i)].iq_a;
            }
        }
        uint8_t wire[kHandBatchWireLen];
        packHandBatch(wire, seq_++, self, mpos, mvel, miq);
        std::lock_guard<std::mutex> lock(io_mu_);
        if (!transport_ || !transport_->isOpen()) {
            return fail(err, Err::NotConnected);
        }
        if (!transport_->send(wire, kHandBatchWireLen)) {
            transport_->close();
            return fail(err, Err::Io);
        }
        tx_ok_++;
        return true;
    }

    bool waitUntil(const std::function<int()> &step, int timeout_ms, Err timeout_code, std::string *err) {
        const auto session = session_.load();
        if (!transportOpen()) return fail(err, Err::NotConnected);
        if (timeout_ms < 0) timeout_ms = 0;
        const auto t0 = std::chrono::steady_clock::now();
        for (;;) {
            if (session_.load() != session || lost_.load()) return fail(err, Err::Disconnected);
            const int r = step();
            if (session_.load() != session || lost_.load()) return fail(err, Err::Disconnected);
            if (r > 0) {
                clearError();
                return true;
            }
            if (r < 0) {
                if (err) {
                    std::lock_guard<std::mutex> lock(state_mu_);
                    *err = last_err_;
                }
                return false;
            }
            if (!transportOpen()) return fail(err, Err::Disconnected);
            const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - t0)
                                .count();
            if (dt >= timeout_ms) return fail(err, timeout_code);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void handleFrame(const std::vector<uint8_t> &raw) {
        FrameView fv;
        if (!parseFrame(raw, &fv) || fv.dev != kDevTelem) return;
        if (fv.fun == kFunHostInfo) {
            FwInfo fw;
            if (parseHostInfo(fv.payload, fv.plen, &fw)) {
                std::lock_guard<std::mutex> lock(state_mu_);
                fw_ = fw;
                hand_side_ = fw.hand_side;
            }
            return;
        }
        if (fv.fun == kFunHandSide) {
            HandSide side = HandSide::Unset;
            uint8_t status = 0;
            if (!parseHandSide(fv.payload, fv.plen, &side, &status)) return;
            std::lock_guard<std::mutex> lock(state_mu_);
            hand_side_ = side;
            hand_side_status_ = status;
            hand_reply_epoch_ = hand_query_epoch_;
            if (status == 0) fw_.hand_side = side;
            return;
        }
        if (fv.fun == kFunHandTelem) {
            std::array<SelfTelem, kSelfMotorNum> self{};
            std::array<MitTelem, kMitMotorNum> mit{};
            uint16_t seq = 0;
            uint32_t tick = 0;
            if (!parseHandTelem(fv.payload, fv.plen, &self, &mit, &seq, &tick)) return;
            uint32_t mask = 0;
            if (fv.plen >= 12) {
                mask = static_cast<uint32_t>(fv.payload[8]) |
                       (static_cast<uint32_t>(fv.payload[9]) << 8) |
                       (static_cast<uint32_t>(fv.payload[10]) << 16) |
                       (static_cast<uint32_t>(fv.payload[11]) << 24);
            }
            {
                std::lock_guard<std::mutex> lock(state_mu_);
                self_telem_ = self;
                mit_telem_ = mit;
                telem_seq_ = seq;
                tick_ms_ = tick;
                rx_mask_ = mask;
                rx_ok_++;
                ++hand_generation_;
                hand_t_ = std::chrono::steady_clock::now();
                for (int i = 0; i < kSelfMotorNum; ++i) {
                    const size_t n = static_cast<size_t>(i);
                    if (!home_reset_hold_[n] || !self_telem_[n].online) continue;
                    if (homingState(self_telem_[n].homing_or_theta) == home_reset_state_[n]) continue;
                    home_reset_hold_[n] = false;
                }
                for (int i = 0; i < kSelfMotorNum; ++i) observeHomeLocked(i);
            }
            TelemCallback cb;
            {
                std::lock_guard<std::mutex> lock(cb_mu_);
                cb = telem_cb_;
            }
            if (cb) {
                CallbackScope scope(&callback_depth_);
                try { cb(); }
                catch (...) { fail(nullptr, Err::CallbackFailed); }
            }
            return;
        }
        if (fv.fun == kFunProtect) {
            ProtectEvent ev;
            if (!parseProtect(fv.payload, fv.plen, &ev)) return;
            std::lock_guard<std::mutex> lock(state_mu_);
            protect_[static_cast<size_t>(ev.slot)] = ev;
            protect_t_[static_cast<size_t>(ev.slot)] = std::chrono::steady_clock::now();
            return;
        }
        if (fv.fun == kFunBoardTelem) {
            BoardTelem b;
            if (parseBoardTelem(fv.payload, fv.plen, &b)) {
                std::lock_guard<std::mutex> lock(state_mu_);
                board_ = b;
                board_t_ = std::chrono::steady_clock::now();
            }
            return;
        }
        if (fv.fun == kFunTouchData && fv.plen >= 6) {
            uint8_t src = fv.payload[0];
            uint8_t cmd = fv.payload[2];
            uint16_t dlen = static_cast<uint16_t>(fv.payload[4] | (fv.payload[5] << 8));
            if (src >= kTouchSlotNum) return;
            size_t n = dlen;
            if (n > fv.plen - 6) n = fv.plen - 6;
            TouchSlot slot;
            slot.bus = fv.payload[1];
            slot.online = true;
            if (parseTouchData(cmd, fv.payload + 6, n, &slot)) {
                std::lock_guard<std::mutex> lock(state_mu_);
                slot.ack = touch_[src].ack;
                slot.ack_cmd = touch_[src].ack_cmd;
                touch_[src] = std::move(slot);
                touch_[src].online = true;
                touch_t_[src] = std::chrono::steady_clock::now();
            }
            return;
        }
        if (fv.fun == kFunTouchAck && fv.plen >= 6) {
            uint8_t src = fv.payload[0];
            uint8_t cmd = fv.payload[1];
            uint8_t ack = fv.payload[2];
            if (src < kTouchSlotNum) {
                std::lock_guard<std::mutex> lock(state_mu_);
                touch_[src].ack_cmd = cmd;
                touch_[src].ack = ack;
            }
            return;
        }
    }

    void rxLoop() {
        uint8_t tmp[4096];
        auto t0 = std::chrono::steady_clock::now();
        uint64_t n0 = 0;
        while (run_.load()) {
            ITransport *t = nullptr;
            bool lost_now = false;
            {
                std::lock_guard<std::mutex> lock(io_mu_);
                t = transport_.get();
                lost_now = !t || !t->isOpen();
            }
            if (lost_now) {
                if (run_.load() && !lost_.load())
                    notifyDisconnect(Err::Disconnected, errMessage(Err::Disconnected));
                break;
            }
            const size_t n = t->recv(tmp, sizeof(tmp), 0);
            if (n) {
                rx_buf_.insert(rx_buf_.end(), tmp, tmp + n);
                if (rx_buf_.size() > 65536) rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + 32768);
                auto frames = popAa55Frames(rx_buf_);
                for (const auto &f : frames) {
                    if (lost_.load()) break;
                    handleFrame(f);
                }
            } else if (!t->isOpen()) {
                if (run_.load() && !lost_.load())
                    notifyDisconnect(Err::Disconnected, errMessage(Err::Disconnected));
                break;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - t0).count();
            if (dt >= 0.5) {
                uint64_t n1 = rx_ok_.load();
                rx_hz_ = static_cast<double>(n1 - n0) / dt;
                n0 = n1;
                t0 = now;
            }
        }
    }

    void txLoop() {
        TimerRes res;
        auto next = std::chrono::steady_clock::now();
        auto t0 = next;
        uint64_t n0 = tx_ok_.load();
        while (streaming_.load() && run_.load() && !lost_.load()) {
            if (!packAndSendBatch(nullptr)) {
                if (run_.load() && !lost_.load())
                    notifyDisconnect(Err::Io, errMessage(Err::Io));
                break;
            }
            if (!transportOpen() && run_.load()) {
                notifyDisconnect(Err::Disconnected, errMessage(Err::Disconnected));
                break;
            }
            int hz = stream_hz_.load();
            next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / hz));
            {
                std::unique_lock<std::mutex> wait_lock(stream_wait_mu_);
                stream_wait_cv_.wait_until(wait_lock, next, [this] { return !streaming_.load() || !run_.load() || lost_.load(); });
            }
            if (next < std::chrono::steady_clock::now()) next = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - t0).count();
            if (dt >= 0.5) {
                uint64_t n1 = tx_ok_.load();
                tx_hz_ = static_cast<double>(n1 - n0) / dt;
                n0 = n1;
                t0 = now;
            }
        }
    }
};

A3HandApi::A3HandApi() : impl_(std::make_unique<Impl>()) {}

A3HandApi::~A3HandApi() {
    InternalLifecycle nested;
    disconnect();
}

bool A3HandApi::attach(std::unique_ptr<ITransport> t, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    InternalLifecycle nested;
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    disconnect();
    if (!t) return impl_->fail(err, Err::InvalidArg, errMessage(Err::InvalidArg));
    const std::string name = t->name();
    if (!t->open()) return impl_->fail(err, Err::Io, std::string(errMessage(Err::Io)) + ": " + name);
    {
        std::lock_guard<std::mutex> lock(impl_->io_mu_);
        impl_->link_name_ = name;
        impl_->transport_ = std::move(t);
        impl_->run_ = true;
        impl_->lost_ = false;
        impl_->streaming_ = false;
        impl_->rx_buf_.clear();
        impl_->rx_ok_ = 0;
        impl_->tx_ok_ = 0;
        impl_->rx_hz_ = 0;
        impl_->tx_hz_ = 0;
    }
    impl_->clearLinkState();
    impl_->clearError();
    impl_->rx_th_ = std::thread([this] { impl_->rxLoop(); });
    return true;
}

bool A3HandApi::handshake(int timeout_ms, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    { std::lock_guard<std::mutex> lock(impl_->state_mu_); impl_->fw_ = {}; }
    if (!impl_->sendRaw(packHostInfo(), err)) return false;
    return impl_->waitUntil(
        [this]() {
            const FwInfo fw = firmware();
            if (!fw.valid) return 0;
            if (!fwCompatible(fw)) {
                impl_->fail(nullptr, Err::FwIncompatible);
                return -1;
            }
            return 1;
        },
        timeout_ms, Err::FwTimeout, err);
}

bool A3HandApi::connectUsb(const std::string &port, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    InternalLifecycle nested;
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    if (!attach(std::make_unique<UsbCdc>(port), err)) return false;
    if (!handshake(1500, err)) {
        disconnect();
        return false;
    }
    return true;
}

bool A3HandApi::connectUsbAuto(std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    InternalLifecycle nested;
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    const auto ports = listUsbPorts();
    for (const auto &p : ports) {
        if (!p.h7_cdc) continue;
        return connectUsb(p.name, err);
    }
    return impl_->fail(err, Err::NoPort);
}

bool A3HandApi::connectEth(const std::string &host, uint16_t port, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    InternalLifecycle nested;
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    if (!attach(std::make_unique<Tcp5001>(host, port), err)) return false;
    if (!handshake(1500, err)) {
        disconnect();
        return false;
    }
    return true;
}

bool A3HandApi::connectEthAuto(uint16_t port, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    InternalLifecycle nested;
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    if (port == 0) port = 5001;
    const auto hosts = listEthHosts(port);
    if (hosts.empty()) return impl_->fail(err, Err::NoEth);
    for (const auto &host : hosts) {
        if (connectEth(host, port, err)) return true;
    }
    return false;
}

void A3HandApi::disconnect() {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) { impl_->fail(nullptr, Err::InvalidArg, "lifecycle operations are not allowed in callbacks"); return; }
    InternalLifecycle nested;
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    ++impl_->session_;
    impl_->lost_ = true;
    stopStream();
    impl_->run_ = false;
    impl_->lost_ = true;
    if (impl_->rx_th_.joinable()) impl_->rx_th_.join();
    {
        std::lock_guard<std::mutex> lock(impl_->io_mu_);
        if (impl_->transport_) {
            impl_->transport_->close();
            impl_->transport_.reset();
        }
        impl_->link_name_.clear();
        impl_->streaming_ = false;
    }
    impl_->clearLinkState();
}

bool A3HandApi::connected() const { return impl_->transportOpen() && !impl_->lost_.load(); }

FwInfo A3HandApi::firmware() const {
    std::lock_guard<std::mutex> lock(impl_->state_mu_);
    return impl_->fw_;
}

bool A3HandApi::queryHandSide(HandSide *side, int timeout_ms, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    if (!side) return impl_->fail(err, Err::InvalidArg);
    uint32_t epoch = 0;
    {
        std::lock_guard<std::mutex> lock(impl_->state_mu_);
        epoch = ++impl_->hand_query_epoch_;
    }
    if (!impl_->sendRaw(packHandSideQuery(), err)) return false;
    const bool ok = impl_->waitUntil(
        [this, epoch]() {
            uint8_t status = 0;
            bool ready = false;
            {
                std::lock_guard<std::mutex> lock(impl_->state_mu_);
                if (impl_->hand_reply_epoch_ != epoch) return 0;
                status = impl_->hand_side_status_;
                ready = true;
            }
            if (!ready) return 0;
            if (status != 0) {
                impl_->fail(nullptr, Err::Io, "hand side read failed");
                return -1;
            }
            return 1;
        },
        timeout_ms, Err::Timeout, err);
    if (!ok) return false;
    std::lock_guard<std::mutex> lock(impl_->state_mu_);
    *side = impl_->hand_side_;
    return true;
}

bool A3HandApi::setJointAnglesDeg(const float angle_deg[kJointNum]) {
    if (!angle_deg) return impl_->fail(nullptr, Err::InvalidArg);
    float actuator[kJointNum];
    float self[kSelfMotorNum];
    float mit[kMitMotorNum];
    if (!anglesDegToActuator(angle_deg, actuator) || !actuatorToHand(actuator, self, mit))
        return impl_->fail(nullptr, Err::InvalidArg);
    if (!setPositions(self)) return false;
    for (int i = 0; i < kMitMotorNum; ++i) {
        if (!setMit(i, mit[i], 0.f, 0.f)) return false;
    }
    return true;
}

bool A3HandApi::setPositions(const float self_pos[kSelfMotorNum]) {
    if (!self_pos) return impl_->fail(nullptr, Err::InvalidArg);
    for (int i = 0; i < kSelfMotorNum; ++i)
        if (!std::isfinite(self_pos[i])) return impl_->fail(nullptr, Err::InvalidArg);
    std::lock_guard<std::mutex> lock(impl_->state_mu_);
    for (int i = 0; i < kSelfMotorNum; ++i)
        impl_->self_pos_[static_cast<size_t>(i)] = clampSlotPos(i, self_pos[i]);
    impl_->last_code_ = Err::Ok;
    impl_->last_err_.clear();
    return true;
}

bool A3HandApi::setMit(int idx, float pos_out_deg, float vel_out_rpm, float iq_a) {
    if (idx < 0 || idx >= kMitMotorNum || !finiteMit({pos_out_deg, vel_out_rpm, iq_a}))
        return impl_->fail(nullptr, Err::InvalidArg);
    std::lock_guard<std::mutex> lock(impl_->state_mu_);
    impl_->mit_[static_cast<size_t>(idx)] = {clampMitPos(pos_out_deg), vel_out_rpm, iq_a};
    impl_->last_code_ = Err::Ok;
    impl_->last_err_.clear();
    return true;
}

bool A3HandApi::setMitAll(const MitCmd mit[kMitMotorNum]) {
    if (!mit) return impl_->fail(nullptr, Err::InvalidArg);
    for (int i = 0; i < kMitMotorNum; ++i)
        if (!finiteMit(mit[i])) return impl_->fail(nullptr, Err::InvalidArg);
    std::lock_guard<std::mutex> lock(impl_->state_mu_);
    for (int i = 0; i < kMitMotorNum; ++i) {
        MitCmd c = mit[i];
        c.pos_out_deg = clampMitPos(c.pos_out_deg);
        impl_->mit_[static_cast<size_t>(i)] = c;
    }
    impl_->last_code_ = Err::Ok;
    impl_->last_err_.clear();
    return true;
}

bool A3HandApi::sendOnce(std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    if (!impl_->packAndSendBatch(err)) {
        if (lastErr() == Err::Io && impl_->run_.load() && !impl_->lost_.load())
            impl_->notifyDisconnect(Err::Io, lastError());
        return false;
    }
    return true;
}

bool A3HandApi::startStream(int hz, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    if (!connected()) return impl_->fail(err, Err::NotConnected);
    if (hz < 1) hz = 1;
    if (hz > 1000) hz = 1000;
    impl_->stream_hz_ = hz;
    if (impl_->streaming_.exchange(true)) {
        impl_->clearError();
        return true;
    }
    if (impl_->tx_th_.joinable()) impl_->tx_th_.join();
    impl_->streaming_ = true;
    impl_->tx_th_ = std::thread([this] { impl_->txLoop(); });
    impl_->clearError();
    return true;
}

void A3HandApi::stopStream() {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) { impl_->fail(nullptr, Err::InvalidArg, "lifecycle operations are not allowed in callbacks"); return; }
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    impl_->streaming_ = false;
    impl_->stream_wait_cv_.notify_all();
    if (impl_->tx_th_.joinable()) impl_->tx_th_.join();
}

bool A3HandApi::streaming() const { return impl_->streaming_.load(); }
int A3HandApi::streamHz() const { return impl_->stream_hz_.load(); }
double A3HandApi::txHz() const { return impl_->tx_hz_.load(); }
double A3HandApi::rxHz() const { return impl_->rx_hz_.load(); }
uint64_t A3HandApi::rxOk() const { return impl_->rx_ok_.load(); }

bool A3HandApi::enable(uint8_t bus, uint8_t node, bool on, uint8_t drv, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    if (!validAddress(bus, node, true) || drv == 0 || (drv & ~kPwrDrvAll))
        return impl_->fail(err, Err::InvalidArg);
    return impl_->sendRaw(packSelfPwr(bus, node, on ? kCmdEnable : kCmdDisable, drv), err);
}

bool A3HandApi::enableSlot(int slot, bool on, uint8_t drv, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    InternalLifecycle nested;
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    uint8_t bus = 0, node = 0;
    if (!slotToBusNode(slot, &bus, &node)) return impl_->fail(err, Err::InvalidSlot);
    return enable(bus, node, on, drv, err);
}

bool A3HandApi::enableBusKind(int bus, bool rotary, bool on, uint8_t drv, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    InternalLifecycle nested;
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    auto nodes = busKindNodes(bus, rotary);
    if (nodes.empty()) return impl_->fail(err, Err::InvalidArg);
    for (uint8_t n : nodes) {
        if (!enable(static_cast<uint8_t>(bus), n, on, drv, err)) return false;
    }
    return true;
}

bool A3HandApi::home(uint8_t bus, uint8_t node, IqSeekCounts iq, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    if (!validAddress(bus, node, false) || !std::isfinite(iq.counts) || iq.counts <= 0)
        return impl_->fail(err, Err::InvalidArg);
    impl_->noteHome(bus, node);
    if (!impl_->sendRaw(packSelfHoming(bus, node, kCmdHome, iq.counts), err)) {
        std::lock_guard<std::mutex> lock(impl_->state_mu_);
        const auto slot = static_cast<size_t>(busNodeToSlot(bus, node));
        impl_->home_need_seek_[slot] = false;
        impl_->home_seen_seek_[slot] = false;
        impl_->home_need_gap_[slot] = false;
        return false;
    }
    return true;
}

bool A3HandApi::homeSlot(int slot, IqSeekCounts iq, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    InternalLifecycle nested;
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    uint8_t bus = 0, node = 0;
    if (!slotToBusNode(slot, &bus, &node)) return impl_->fail(err, Err::InvalidSlot);
    return home(bus, node, iq, err);
}

bool A3HandApi::homeBusKind(int bus, bool rotary, IqSeekCounts iq, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    auto nodes = busKindNodes(bus, rotary);
    if (nodes.empty()) return impl_->fail(err, Err::InvalidArg);
    InternalLifecycle nested;
    struct KeepLink {
        bool previous = keep_link_on_send_error;
        KeepLink() { keep_link_on_send_error = true; }
        ~KeepLink() { keep_link_on_send_error = previous; }
    } keep;
    std::vector<uint8_t> sent;
    for (uint8_t n : nodes) {
        if (!home(static_cast<uint8_t>(bus), n, iq, err)) {
            const Err code = lastErr();
            const std::string failed = lastError();
            bool resets_ok = true;
            std::string ignore;
            for (uint8_t prev : sent) {
                if (!homeReset(static_cast<uint8_t>(bus), prev, &ignore)) resets_ok = false;
            }
            if (code == Err::Io && (sent.empty() || !resets_ok))
                impl_->notifyDisconnect(Err::Io, errMessage(Err::Io));
            return impl_->fail(err, code, failed.empty() ? std::string(errMessage(code)) : failed);
        }
        sent.push_back(n);
    }
    return true;
}

bool A3HandApi::homeReset(uint8_t bus, uint8_t node, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    if (!validAddress(bus, node, false)) return impl_->fail(err, Err::InvalidArg);
    if (!impl_->sendRaw(packSelfHoming(bus, node, kCmdHomeReset, 0.f), err)) return false;
    impl_->noteReset(bus, node);
    return true;
}

bool A3HandApi::homeResetSlot(int slot, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    InternalLifecycle nested;
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    uint8_t bus = 0, node = 0;
    if (!slotToBusNode(slot, &bus, &node)) return impl_->fail(err, Err::InvalidSlot);
    return homeReset(bus, node, err);
}

bool A3HandApi::waitOnline(uint32_t mask, int timeout_ms, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    if (mask & ~kSelfOnlineMaskAll) return impl_->fail(err, Err::InvalidArg);
    if (mask == 0) {
        impl_->clearError();
        return true;
    }
    return impl_->waitUntil(
        [this, mask]() { return ((rxMask() & mask) == mask) ? 1 : 0; }, timeout_ms, Err::Timeout, err);
}

bool A3HandApi::waitSlotOnline(int slot, int timeout_ms, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    if (slot < 0 || slot >= kSelfMotorNum) return impl_->fail(err, Err::InvalidSlot);
    return waitOnline(1u << slot, timeout_ms, err);
}

bool A3HandApi::waitHomeDone(int slot, int timeout_ms, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    if (slot < 0 || slot >= kSelfMotorNum) return impl_->fail(err, Err::InvalidSlot);
    return impl_->waitUntil([this, slot]() { return impl_->pollHome(slot); }, timeout_ms, Err::Timeout, err);
}

bool A3HandApi::estop(std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    InternalLifecycle nested;
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    stopStream();
    return enable(kBusAll, kNodeAll, false, kPwrDrvAll, err);
}

bool A3HandApi::setPosPid(int slot, float kp, float ki, float kd, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    if (!std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd) || kp < 0 || ki < 0 || kd < 0)
        return impl_->fail(err, Err::InvalidArg);
    uint8_t bus = 0, node = 0;
    if (!slotToBusNode(slot, &bus, &node)) return impl_->fail(err, Err::InvalidSlot);
    return impl_->sendRaw(packSelfPosPid(bus, node, kp, ki, kd), err);
}

bool A3HandApi::setCurPid(int slot,
                          float id_kp, float id_ki, float id_kd,
                          float iq_kp, float iq_ki, float iq_kd,
                          std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    for (float v : {id_kp, id_ki, id_kd, iq_kp, iq_ki, iq_kd})
        if (!std::isfinite(v) || v < 0) return impl_->fail(err, Err::InvalidArg);
    uint8_t bus = 0, node = 0;
    if (!slotToBusNode(slot, &bus, &node)) return impl_->fail(err, Err::InvalidSlot);
    return impl_->sendRaw(packSelfCurPid(bus, node, id_kp, id_ki, id_kd, iq_kp, iq_ki, iq_kd), err);
}

bool A3HandApi::openSpin(int slot, bool start, float freq_hz, float half_s, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    if (!std::isfinite(freq_hz) || !std::isfinite(half_s) || (start && (freq_hz <= 0 || half_s <= 0)))
        return impl_->fail(err, Err::InvalidArg);
    uint8_t bus = 0, node = 0;
    if (!slotToBusNode(slot, &bus, &node)) return impl_->fail(err, Err::InvalidSlot);
    return impl_->sendRaw(packSelfOpenSpin(bus, node, start ? kCmdSpinStart : kCmdSpinStop, freq_hz, half_s),
                          err);
}

bool A3HandApi::sendTouch(uint8_t src, uint8_t cmd, const uint8_t *data, size_t n, std::string *err) {
    if (lifecycle_internal == 0 && (in_callback || impl_->callback_depth_.load(std::memory_order_acquire) > 0)) return impl_->fail(err, Err::InvalidArg, "commands and waits are not allowed in callbacks");
    std::lock_guard<std::recursive_mutex> lifecycle(impl_->lifecycle_mu_);
    if ((src >= kTouchSlotNum && src != kTouchSrcAll) || (n && !data) || n > 65531)
        return impl_->fail(err, Err::InvalidArg);
    return impl_->sendRaw(packTouchCmd(src, cmd, data, n), err);
}

std::array<SelfTelem, kSelfMotorNum> A3HandApi::selfTelem() const {
    std::lock_guard<std::mutex> lock(impl_->state_mu_);
    auto out = impl_->self_telem_;
    if (impl_->lost_.load() || impl_->hand_t_ == std::chrono::steady_clock::time_point{} ||
        std::chrono::steady_clock::now() - impl_->hand_t_ > std::chrono::milliseconds(kTelemetryStaleMs))
        for (auto &s : out) s.online = false;
    return out;
}

TelemetrySnapshot A3HandApi::telemetrySnapshot() const {
    std::lock_guard<std::mutex> lock(impl_->state_mu_);
    TelemetrySnapshot out;
    out.self = impl_->self_telem_; out.mit = impl_->mit_telem_;
    out.protect = impl_->protect_; out.board = impl_->board_; out.touch = impl_->touch_;
    out.seq = impl_->telem_seq_; out.tick_ms = impl_->tick_ms_; out.rx_mask = impl_->rx_mask_;
    const auto now = std::chrono::steady_clock::now();
    const auto age = [now](auto time) -> int64_t {
        if (time == std::chrono::steady_clock::time_point{}) return -1;
        return std::chrono::duration_cast<std::chrono::milliseconds>(now - time).count();
    };
    out.age_ms = age(impl_->hand_t_);
    const bool live = impl_->run_.load() && !impl_->lost_.load();
    out.fresh = live && out.age_ms >= 0 && out.age_ms <= kTelemetryStaleMs;
    if (!out.fresh) {
        out.rx_mask = 0;
        for (auto &s : out.self) s.online = false;
        for (auto &m : out.mit) { m.online = false; m.valid = false; }
    }
    if (!live || age(impl_->board_t_) < 0 || age(impl_->board_t_) > kTelemetryStaleMs) out.board.valid = false;
    for (size_t i = 0; i < out.touch.size(); ++i) {
        if (live && age(impl_->touch_t_[i]) >= 0 && age(impl_->touch_t_[i]) <= kTelemetryStaleMs) continue;
        out.touch[i].online = false;
        for (auto &part : out.touch[i].parts) part.valid = false;
    }
    for (size_t i = 0; i < out.protect.size(); ++i)
        if (!live || age(impl_->protect_t_[i]) < 0 || age(impl_->protect_t_[i]) > kProtectStaleMs) out.protect[i] = {};
    return out;
}

std::array<ProtectEvent, kSelfMotorNum> A3HandApi::protectTelem() const {
    std::lock_guard<std::mutex> lock(impl_->state_mu_);
    auto now = std::chrono::steady_clock::now();
    auto out = impl_->protect_;
    for (size_t i = 0; i < out.size(); ++i) {
        if (!out[i].active) continue;
        auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - impl_->protect_t_[i]);
        if (age.count() > kProtectStaleMs) out[i] = {};
    }
    return out;
}

std::array<MitTelem, kMitMotorNum> A3HandApi::mitTelem() const {
    return telemetrySnapshot().mit;
}

BoardTelem A3HandApi::boardTelem() const {
    return telemetrySnapshot().board;
}

std::array<TouchSlot, kTouchSlotNum> A3HandApi::touch() const {
    return telemetrySnapshot().touch;
}

uint32_t A3HandApi::rxMask() const {
    std::lock_guard<std::mutex> lock(impl_->state_mu_);
    if (impl_->lost_.load() || impl_->hand_t_ == std::chrono::steady_clock::time_point{} ||
        std::chrono::steady_clock::now() - impl_->hand_t_ > std::chrono::milliseconds(kTelemetryStaleMs)) return 0;
    return impl_->rx_mask_;
}

uint16_t A3HandApi::telemSeq() const {
    std::lock_guard<std::mutex> lock(impl_->state_mu_);
    return impl_->telem_seq_;
}

uint32_t A3HandApi::tickMs() const {
    std::lock_guard<std::mutex> lock(impl_->state_mu_);
    return impl_->tick_ms_;
}

Err A3HandApi::lastErr() const {
    std::lock_guard<std::mutex> lock(impl_->state_mu_);
    return impl_->last_code_;
}

std::string A3HandApi::lastError() const {
    std::lock_guard<std::mutex> lock(impl_->state_mu_);
    return impl_->last_err_;
}

std::array<float, kSelfMotorNum> A3HandApi::targetSelf() const {
    std::lock_guard<std::mutex> lock(impl_->state_mu_);
    return impl_->self_pos_;
}

std::array<MitCmd, kMitMotorNum> A3HandApi::targetMit() const {
    std::lock_guard<std::mutex> lock(impl_->state_mu_);
    return impl_->mit_;
}

std::string A3HandApi::linkName() const {
    std::lock_guard<std::mutex> lock(impl_->io_mu_);
    return impl_->link_name_;
}

void A3HandApi::setTelemCallback(TelemCallback cb) {
    std::lock_guard<std::mutex> lock(impl_->cb_mu_);
    impl_->telem_cb_ = std::move(cb);
}

void A3HandApi::setDisconnectCallback(DisconnectCallback cb) {
    std::lock_guard<std::mutex> lock(impl_->cb_mu_);
    impl_->disconnect_cb_ = std::move(cb);
}

} // namespace a3
