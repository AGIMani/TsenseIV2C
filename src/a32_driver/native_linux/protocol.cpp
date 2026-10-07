#include "protocol.hpp"
#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace a32wire {
static uint16_t u16(const uint8_t* p) { return static_cast<uint16_t>(uint16_t(p[0])|(uint16_t(p[1])<<8)); }
static uint32_t u32(const uint8_t* p) { return uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24); }
static int16_t i16(const uint8_t* p) { const auto n=u16(p); return static_cast<int16_t>(n<32768?int(n):int(n)-65536); }
static float f32(const uint8_t* p) { const auto bits=u32(p); float value; std::memcpy(&value,&bits,4); return value; }
static void put16(Bytes& out,size_t offset,uint16_t value) { out[offset]=static_cast<uint8_t>(value); out[offset+1]=static_cast<uint8_t>(value>>8); }
static void putFloat(Bytes& out,size_t offset,float value) {
    static_assert(sizeof(float)==4,"A3 wire requires IEEE 754 binary32");
    uint32_t bits; std::memcpy(&bits,&value,4);
    for (size_t i=0;i<4;++i) out[offset+i]=static_cast<uint8_t>(bits>>(8*i));
}
uint16_t crc(const uint8_t* data,size_t size) {
    uint16_t value=0xffff;
    for (size_t i=0;i<size;++i) {
        value^=data[i];
        for (int bit=0;bit<8;++bit) value=static_cast<uint16_t>((value>>1)^((value&1)?0xa001:0));
    }
    return value;
}
Bytes frame(uint8_t function,const Bytes& payload) {
    if (payload.size()>2040) throw std::invalid_argument("oversized A3 frame");
    Bytes out(payload.size()+8,0);
    out[0]=0xaa; out[1]=0x55; put16(out,2,static_cast<uint16_t>(payload.size()));
    out[4]=3; out[5]=function;
    std::copy(payload.begin(),payload.end(),out.begin()+6);
    put16(out,out.size()-2,crc(out.data(),out.size()-2));
    return out;
}
Bytes host_info() { return frame(1,{1,1,0,1,0,0,0,0}); }
Bytes power(bool enabled) { return frame(0x21,{0xff,0xff,static_cast<uint8_t>(enabled?1:2),3}); }
Bytes home(int slot,bool reset,float iq) {
    if (slot<0 || slot>=18) throw std::invalid_argument("invalid A3 self slot");
    Bytes payload(12,0);
    payload[0]=static_cast<uint8_t>(slot<8?0:slot<16?1:2);
    payload[1]=static_cast<uint8_t>(slot<8?slot+1:slot<16?slot-7:slot-15);
    payload[2]=static_cast<uint8_t>(reset?2:1);
    putFloat(payload,4,reset?0.f:iq);
    return frame(0x22,payload);
}
Bytes targets(uint8_t sequence,const std::array<float,18>& self,const std::array<float,2>& mit) {
    Bytes payload(100,0); payload[0]=sequence;
    for (size_t i=0;i<18;++i) putFloat(payload,1+4*i,self[i]);
    for (size_t i=0;i<2;++i) putFloat(payload,73+4*i,mit[i]*50.f);
    // ABI only supplies position. Velocity/current feed-forward and padding stay zero.
    return frame(0x20,payload);
}
bool pop(Bytes& pending,uint8_t& function,Bytes& payload) {
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
        if (total>2048 || pending[4]!=3) { pending.erase(pending.begin()); continue; }
        if (pending.size()<total) return false;
        if (u16(pending.data()+total-2)!=crc(pending.data(),total-2)) { pending.erase(pending.begin()); continue; }
        function=pending[5]; payload.assign(pending.begin()+6,pending.begin()+6+static_cast<std::ptrdiff_t>(plen));
        pending.erase(pending.begin(),pending.begin()+static_cast<std::ptrdiff_t>(total));
        return true;
    }
}
bool parse_firmware(const Bytes& payload,Firmware& value) {
    if (payload.size()<8) return false;
    value={true,payload[0],payload[1],payload[2],payload[3],payload[4],payload[5]}; return true;
}
bool parse_hand(const Bytes& payload,Hand& value) {
    if (payload.size()<444) return false;
    Hand decoded;
    decoded.sequence=u16(payload.data()); decoded.tick=u32(payload.data()+4);
    decoded.mask=u32(payload.data()+8)&0x3ffff;
    for (size_t i=0;i<18;++i) {
        const uint8_t* raw=payload.data()+12+24*i;
        auto& row=decoded.self[i]; row.online=(decoded.mask&(1u<<i))!=0;
        if (raw[0]>=1 && raw[0]<=4) row.homing=255;
        else if (raw[0]==0 && raw[1]==0 && raw[2]==0) row.homing=raw[3]<=4?raw[3]:0;
        else row.homing=254;
        row.open_spin=row.homing==255;
        row.position=f32(raw+4); row.iq=f32(raw+8); row.iq_ref=f32(raw+12); row.target=f32(raw+16);
        row.temperature=i16(raw+22);
    }
    if (payload.size()>=484) for (size_t i=0;i<2;++i) {
        if (!(payload[2]&(1u<<i))) continue;
        const uint8_t* raw=payload.data()+444+20*i;
        auto& row=decoded.mit[i]; row.valid=row.online=true;
        // Subtract the unsigned center in int64 before converting to float, preserving
        // tiny values adjacent to center and avoiding signed overflow at both extremes.
        row.position=static_cast<float>(static_cast<double>(int64_t(u32(raw+2))-2147483648LL)*360.0/2097152.0/50.0);
        row.velocity=static_cast<float>(int(u16(raw+6))-32768)*0.1f/50.f;
        row.iq=static_cast<float>(int(u16(raw+8))-32768)*88.f/32768.f;
        row.voltage=static_cast<float>(u16(raw+10))*0.1f;
        row.fault=u32(raw+12); row.state=raw[16]; row.calib=raw[17]; row.mode=raw[18];
    }
    value=decoded; return true;
}
bool parse_board(const Bytes& payload,Board& value) {
    if (payload.size()<8) return false;
    value={true,i16(payload.data()+2),i16(payload.data()+4),u16(payload.data()+6)}; return true;
}
}
