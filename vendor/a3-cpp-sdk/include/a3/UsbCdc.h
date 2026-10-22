#pragma once

#include "a3/A3Export.h"
#include "a3/ITransport.h"
#include "a3/Types.h"

#include <memory>
#include <string>
#include <vector>

namespace a3 {

A3_API std::vector<ComPort> listUsbPorts();

class A3_API UsbCdc : public ITransport {
public:
    explicit UsbCdc(std::string port);
    ~UsbCdc() override;
    UsbCdc(const UsbCdc &) = delete;
    UsbCdc &operator=(const UsbCdc &) = delete;

    bool open() override;
    void close() override;
    bool send(const uint8_t *p, size_t n) override;
    size_t recv(uint8_t *p, size_t n, int timeout_ms) override;
    bool isOpen() const override;
    std::string name() const override { return port_; }

private:
    std::string port_;
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace a3
