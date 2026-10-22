#include "protocol.hpp"
#include "a3/detail/Protocol.h"
#include <algorithm>
#include <stdexcept>

namespace a32wire {
static uint16_t u16(const uint8_t* p) { return static_cast<uint16_t>(uint16_t(p[0])|(uint16_t(p[1])<<8)); }
static uint32_t u32(const uint8_t* p) { return uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24); }

uint16_t crc(const uint8_t* data,size_t size) { return a3::crc16Modbus(data,size); }

Bytes frame(uint8_t function,const Bytes& payload) {
    if (payload.size()>2040) throw std::invalid_argument("oversized A3 frame");
    return a3::packAa55(function,payload);
}

Bytes host_info() { return a3::packHostInfo(); }

Bytes enable() {
    // The project's public stop operations never emit a motor-disable packet.
    return a3::packSelfPwr(a3::kBusAll,a3::kNodeAll,a3::kCmdEnable,a3::kPwrDrvAll);
}

Bytes home(int slot,bool reset,float iq) {
    uint8_t bus{},node{};
    if (!a3::slotToBusNode(slot,&bus,&node)) throw std::invalid_argument("invalid A3 self slot");
    return a3::packSelfHoming(bus,node,reset?a3::kCmdHomeReset:a3::kCmdHome,reset?0.f:iq);
}

Bytes targets(uint8_t sequence,const std::array<float,18>& self,const std::array<float,2>& mit) {
    std::array<float,a3::kMitMotorNum> motor{};
    const std::array<float,a3::kMitMotorNum> zero{};
    for (size_t i=0;i<motor.size();++i) motor[i]=a3::mitOutToMotor(mit[i]);
    Bytes wire(a3::kHandBatchWireLen);
    // ABI supplies output-axis positions only; velocity/current stay zero.
    a3::packHandBatch(wire.data(),sequence,self.data(),motor.data(),zero.data(),zero.data());
    return wire;
}

bool pop(Bytes& pending,uint8_t& function,Bytes& payload) {
    // Preserve the one-frame interface and early device/length rejection while
    // delegating complete frame validation to the SDK. Retain partial headers
    // and advance one byte after corrupt frames to recover embedded AA55 data.
    const uint8_t magic[2]={0xaa,0x55};
    for (;;) {
        auto start=std::search(pending.begin(),pending.end(),magic,magic+2);
        if (start==pending.end()) {
            const bool suffix=!pending.empty() && pending.back()==0xaa;
            pending.clear(); if (suffix) pending.push_back(0xaa); return false;
        }
        pending.erase(pending.begin(),start);
        if (pending.size()<6) return false;
        const size_t plen=u16(pending.data()+2), total=plen+8;
        if (total>2048 || pending[4]!=a3::kDevTelem) { pending.erase(pending.begin()); continue; }
        if (pending.size()<total) return false;
        const Bytes raw(pending.begin(),pending.begin()+static_cast<std::ptrdiff_t>(total));
        a3::FrameView parsed;
        if (!a3::parseFrame(raw,&parsed)) { pending.erase(pending.begin()); continue; }
        function=parsed.fun;
        payload.assign(parsed.payload,parsed.payload+parsed.plen);
        pending.erase(pending.begin(),pending.begin()+static_cast<std::ptrdiff_t>(total));
        return true;
    }
}

bool parse_firmware(const Bytes& payload,Firmware& value) {
    a3::FwInfo decoded;
    if (!a3::parseHostInfo(payload.data(),payload.size(),&decoded)) return false;
    value={decoded.valid,decoded.proto_major,decoded.proto_minor,decoded.app_major,
        decoded.app_minor,decoded.app_patch,decoded.app_build,static_cast<uint8_t>(decoded.hand_side)};
    return true;
}

bool parse_hand(const Bytes& payload,Hand& value) {
    std::array<a3::SelfTelem,a3::kSelfMotorNum> self{};
    std::array<a3::MitTelem,a3::kMitMotorNum> mit{};
    Hand decoded;
    if (!a3::parseHandTelem(payload.data(),payload.size(),&self,&mit,&decoded.sequence,&decoded.tick)) return false;
    for (size_t i=0;i<self.size();++i) {
        const auto& source=self[i];
        auto& row=decoded.self[i];
        // parseHandTelem already converts linear counts * 0.001374 and rotary
        // counts / 819 to amperes. Forward its values without a second scale.
        // Keep SDK homing classification, including the OpenSpin high-byte check.
        row={source.pos_act,source.pos_ref,source.iq_meas,source.iq_ref,
            source.mcu_temp_centi,static_cast<uint8_t>(source.homing),source.online,source.open_spin};
        if (row.online) decoded.mask|=1u<<i;
    }
    for (size_t i=0;i<mit.size();++i) {
        const auto& source=mit[i];
        auto& row=decoded.mit[i];
        row={source.pos_out_deg,source.vel_out_rpm,source.iq_a,source.vbus_v,
            source.fault,source.state,source.calib,source.mode,source.valid,source.online};
        if (source.valid) {
            // Preserve ABI-v2 precision and defined arithmetic at both uint32
            // extremes. Decode the center in int64 before converting to float;
            // the remaining telemetry interpretation is provided by the SDK.
            const uint8_t* raw=payload.data()+a3::kHandTelemSelfLen+a3::kMitStatusBytes*i;
            row.position=static_cast<float>(static_cast<double>(int64_t(u32(raw+2))-2147483648LL)
                *360.0/static_cast<double>(a3::kMitPosCounts)/static_cast<double>(a3::kMitGearRatio));
        }
    }
    value=decoded;
    return true;
}

bool parse_board(const Bytes& payload,Board& value) {
    a3::BoardTelem decoded;
    if (!a3::parseBoardTelem(payload.data(),payload.size(),&decoded)) return false;
    value={decoded.valid,decoded.ntc_c_centi,decoded.ntc_b_centi,decoded.vin24_mV};
    return true;
}
}
