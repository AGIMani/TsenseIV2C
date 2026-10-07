// Independent implementation of the A3 protocol, not the vendor SDK.
// Wire layouts are documented with offline vendor-oracle evidence in README.md.
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace a32wire {
using Bytes = std::vector<uint8_t>;
struct Self {
    float position{}, target{}, iq{}, iq_ref{};
    int16_t temperature{32767};
    uint8_t homing{};
    bool online{}, open_spin{};
};
struct Mit {
    float position{}, velocity{}, iq{}, voltage{};
    uint32_t fault{};
    uint8_t state{}, calib{}, mode{};
    bool valid{}, online{};
};
struct Firmware {
    bool valid{};
    uint8_t proto_major{}, proto_minor{}, app_major{}, app_minor{}, app_patch{}, app_build{};
};
struct Board { bool valid{}; int16_t ntc_c{32767}, ntc_b{32767}; uint16_t vin{65535}; };
struct Hand {
    std::array<Self,18> self{};
    std::array<Mit,2> mit{};
    uint16_t sequence{};
    uint32_t tick{}, mask{};
};
uint16_t crc(const uint8_t* data, size_t size);
Bytes frame(uint8_t function, const Bytes& payload);
Bytes host_info();
Bytes power(bool enabled);
Bytes home(int slot, bool reset, float iq);
Bytes targets(uint8_t sequence, const std::array<float,18>& self, const std::array<float,2>& mit);
// Consumes bad bytes, oversized headers and complete frames. Incomplete frames remain buffered.
bool pop(Bytes& pending, uint8_t& function, Bytes& payload);
bool parse_firmware(const Bytes& payload, Firmware& value);
bool parse_hand(const Bytes& payload, Hand& value);
bool parse_board(const Bytes& payload, Board& value);
}
