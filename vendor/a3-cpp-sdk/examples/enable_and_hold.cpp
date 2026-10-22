#include "a3/A3HandApi.h"
#include "a3/Protocol.h"
#include "a3/UsbCdc.h"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

namespace {

void printOnline(const a3::A3HandApi &hand) {
    using namespace a3;
    auto selft = hand.selfTelem();
    int n = 0;
    for (int i = 0; i < kSelfMotorNum; ++i) {
        if (!selft[static_cast<size_t>(i)].online) continue;
        ++n;
        std::cout << "  #" << i << "  " << selft[static_cast<size_t>(i)].pos_act << " "
                  << slotUnit(i) << "  " << decodeHomingOrTheta(selft[static_cast<size_t>(i)].homing_or_theta) << "\n";
    }
    if (n == 0) std::cout << "  （暂无在线槽，检查供电与 CAN）\n";
    std::cout << "rx_ok=" << hand.rxOk() << "  rx_hz=" << hand.rxHz()
              << "  tx_hz=" << hand.txHz() << "\n";
}

void usage() {
    std::cout << "a3_example " << a3::sdkVersion() << "\n"
              << "用法:\n"
              << "  a3_example --list\n"
              << "  a3_example --usb COM7\n"
              << "  a3_example --usb /dev/ttyACM0\n"
              << "  a3_example --usb auto --hold [--home] [--hz 500]\n"
              << "  a3_example --eth auto [5001] --hold\n"
              << "  a3_example --eth <dhcp-ip> [5001] --hold\n";
}

} // namespace

int main(int argc, char **argv) {
    using namespace a3;
    std::cout << "a3-cpp-sdk " << sdkVersion() << "\n";

    uint8_t wire[kHandBatchWireLen];
    float self[kSelfMotorNum]{};
    float mz[kMitMotorNum]{};
    packHandBatch(wire, 1, self, mz, mz, mz);
    uint16_t crc = static_cast<uint16_t>(wire[106] | (wire[107] << 8));
    uint16_t calc = crc16Modbus(wire, 106);
    std::cout << "0x20 wire " << kHandBatchWireLen << " B  CRC "
              << (crc == calc ? "OK" : "FAIL") << "\n";
    if (crc != calc) return 1;

    bool do_list = false, do_hold = false, do_home = false;
    std::string usb, eth;
    uint16_t eth_port = 5001;
    int hz = 500;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--list") == 0) do_list = true;
        else if (std::strcmp(argv[i], "--hold") == 0) do_hold = true;
        else if (std::strcmp(argv[i], "--home") == 0) do_home = true;
        else if (std::strcmp(argv[i], "--usb") == 0 && i + 1 < argc) usb = argv[++i];
        else if (std::strcmp(argv[i], "--eth") == 0 && i + 1 < argc) {
            eth = argv[++i];
            if (i + 1 < argc && argv[i + 1][0] != '-')
                eth_port = static_cast<uint16_t>(std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--hz") == 0 && i + 1 < argc)
            hz = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            usage();
            return 0;
        } else {
            std::cerr << "未知参数 " << argv[i] << "\n";
            usage();
            return 1;
        }
    }

    if (argc < 2 || do_list || (usb.empty() && eth.empty())) {
        auto ports = listUsbPorts();
        std::cout << "USB 口:\n";
        for (const auto &p : ports)
            std::cout << "  " << p.name << "  " << p.desc << (p.h7_cdc ? "  [H7]" : "") << "\n";
        if (usb.empty() && eth.empty()) {
            if (argc >= 2 && !do_list) usage();
            return 0;
        }
    }

    std::string err;
    A3HandApi hand;
    hand.setDisconnectCallback([](Err, const std::string &reason) {
        std::cerr << "断开: " << reason << "\n";
    });

    if (!usb.empty()) {
        if (usb == "auto") {
            if (!hand.connectUsbAuto(&err)) {
                std::cerr << err << "\n";
                return 1;
            }
        } else if (!hand.connectUsb(usb, &err)) {
            std::cerr << err << "\n";
            return 1;
        }
    } else if (eth == "auto") {
        if (!hand.connectEthAuto(eth_port, &err)) {
            std::cerr << err << "\n";
            return 1;
        }
    } else if (!hand.connectEth(eth, eth_port, &err)) {
        std::cerr << err << "\n";
        return 1;
    }

    std::cout << "已连接 " << hand.linkName();
    const auto fw = hand.firmware();
    if (fw.valid)
        std::cout << "  H7 APP " << int(fw.app_major) << "." << int(fw.app_minor) << "."
                  << int(fw.app_patch) << "." << int(fw.app_build)
                  << " proto " << int(fw.proto_major) << "." << int(fw.proto_minor);
    std::cout << "，读 2s 遥测...\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));
    printOnline(hand);
    const uint32_t present = hand.rxMask();
    if (present == 0) {
        std::cerr << "无电机在线（不要用 kSelfOnlineMaskAll，除非 18 槽都装了）\n";
        if (do_hold) {
            hand.disconnect();
            return 1;
        }
    }

    if (!do_hold) {
        hand.disconnect();
        return 0;
    }

    if (!hand.enable(0xFF, 0xFF, true, kPwrDrvAll, &err)) {
        std::cerr << "使能失败: " << err << "\n";
        hand.disconnect();
        return 1;
    }
    std::cout << "全手使能\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (do_home) {
        std::cout << "在线槽回零 iq_seek=80 counts（非安培；0x22 仅单台）...\n";
        auto selft = hand.selfTelem();
        for (int slot = 0; slot < kSelfMotorNum; ++slot) {
            if (!selft[static_cast<size_t>(slot)].online) continue;
            if (!hand.homeSlot(slot, IqSeekCounts{80.f}, &err) || !hand.waitHomeDone(slot, 15000, &err)) {
                std::cerr << "  #" << slot << " " << err << "\n";
                hand.estop();
                hand.disconnect();
                return 1;
            }
        }
        printOnline(hand);
    }

    float open_pos[kSelfMotorNum]{};
    float grip_pos[kSelfMotorNum]{};
    for (int i = 0; i < kSelfMotorNum; ++i) {
        if (!slotIsRotary(i)) grip_pos[i] = 5.f;
    }

    hand.setPositions(open_pos);
    if (!hand.startStream(hz, &err)) {
        std::cerr << "startStream: " << err << "\n";
        hand.estop(&err);
        hand.disconnect();
        return 1;
    }
    std::cout << "连续 " << hz << " Hz，张开（全 0）2s...\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));

    hand.setPositions(grip_pos);
    std::cout << "直线槽 5 mm 握持 2s...\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));

    hand.setPositions(open_pos);
    std::cout << "回到 0，保持 2s...\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));
    printOnline(hand);

    if (!hand.estop(&err)) {
        std::cerr << "stop failed: " << err << "\n";
        hand.disconnect();
        return 1;
    }
    std::cout << "已断电并断开\n";
    if (!hand.lastError().empty()) std::cout << "lastError=" << hand.lastError() << "\n";
    hand.disconnect();
    return 0;
}
