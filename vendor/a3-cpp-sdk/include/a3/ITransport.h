#pragma once

#include "a3/A3Export.h"

#include <cstddef>
#include <cstdint>
#include <string>

namespace a3 {

class A3_API ITransport {
public:
    virtual ~ITransport() = default;
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool send(const uint8_t *p, size_t n) = 0;
    /** timeout_ms=0 为非阻塞。返回读到的字节数；超时或失败返回 0，对端关闭后 isOpen() 为 false。 */
    virtual size_t recv(uint8_t *p, size_t n, int timeout_ms) = 0;
    virtual bool isOpen() const = 0;
    virtual std::string name() const = 0;
};

A3_API bool ensureWinsock();

} // namespace a3
