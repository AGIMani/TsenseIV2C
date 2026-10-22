#pragma once

#include "a3/ITransport.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace a3 {

/**
 * IPv4 hosts on this machine's subnets with TCP `port` open.
 * H7 has no static address; it prints `eth dhcp ip=` after DHCP. Prefixes wider than /24
 * are scanned as the /24 that contains the local address. Empty when no DHCP server is reachable.
 */
A3_API std::vector<std::string> listEthHosts(uint16_t port = 5001);

class A3_API Tcp5001 : public ITransport {
public:
    Tcp5001(std::string host, uint16_t port = 5001);
    ~Tcp5001() override;
    Tcp5001(const Tcp5001 &) = delete;
    Tcp5001 &operator=(const Tcp5001 &) = delete;

    bool open() override;
    void close() override;
    bool send(const uint8_t *p, size_t n) override;
    size_t recv(uint8_t *p, size_t n, int timeout_ms) override;
    bool isOpen() const override;
    std::string name() const override;

private:
    std::string host_;
    uint16_t port_;
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace a3
