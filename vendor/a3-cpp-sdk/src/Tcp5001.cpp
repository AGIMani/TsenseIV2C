#include "a3/Tcp5001.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "iphlpapi.lib")
using socklen_t = int;
#else
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
using SOCKET = int;
constexpr int INVALID_SOCKET = -1;
constexpr int SOCKET_ERROR = -1;
static int closesocket(SOCKET s) { return ::close(s); }
#endif

namespace a3 {

bool ensureWinsock() {
#ifdef _WIN32
    static bool ok = false;
    static std::mutex mu;
    std::lock_guard<std::mutex> lock(mu);
    if (ok) return true;
    WSADATA wsa{};
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return false;
    ok = true;
#endif
    return true;
}

struct Tcp5001::Impl {
    SOCKET s = INVALID_SOCKET;
    mutable std::mutex mu;
};

Tcp5001::Tcp5001(std::string host, uint16_t port)
    : host_(std::move(host)), port_(port), impl_(std::make_unique<Impl>()) {}

Tcp5001::~Tcp5001() { close(); }

std::string Tcp5001::name() const { return host_ + ":" + std::to_string(port_); }

bool Tcp5001::open() {
    close();
    if (!ensureWinsock()) return false;
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    addrinfo *res = nullptr;
    std::string ps = std::to_string(port_);
    if (getaddrinfo(host_.c_str(), ps.c_str(), &hints, &res) != 0 || !res) return false;
    SOCKET s = INVALID_SOCKET;
    for (addrinfo *ai = res; ai; ai = ai->ai_next) {
        s = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (s == INVALID_SOCKET) continue;
#ifdef _WIN32
        DWORD ms = 8000;
        setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char *>(&ms), sizeof(ms));
        setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char *>(&ms), sizeof(ms));
        if (connect(s, ai->ai_addr, static_cast<socklen_t>(ai->ai_addrlen)) == 0) break;
        closesocket(s);
        s = INVALID_SOCKET;
#else
        // SO_SNDTIMEO does not bound connect() on Linux. Use a non-blocking connect.
        int fl = fcntl(s, F_GETFL, 0);
        if (fl < 0 || fcntl(s, F_SETFL, fl | O_NONBLOCK) != 0) {
            closesocket(s);
            s = INVALID_SOCKET;
            continue;
        }
        const int cr = ::connect(s, ai->ai_addr, static_cast<socklen_t>(ai->ai_addrlen));
        if (cr != 0 && errno != EINPROGRESS) {
            closesocket(s);
            s = INVALID_SOCKET;
            continue;
        }
        if (cr != 0) {
            fd_set wfds;
            FD_ZERO(&wfds);
            FD_SET(s, &wfds);
            timeval tv{};
            tv.tv_sec = 8;
            tv.tv_usec = 0;
            const int sel = ::select(s + 1, nullptr, &wfds, nullptr, &tv);
            int soerr = 0;
            socklen_t slen = sizeof(soerr);
            if (sel <= 0 || !FD_ISSET(s, &wfds) ||
                ::getsockopt(s, SOL_SOCKET, SO_ERROR, &soerr, &slen) != 0 || soerr != 0) {
                closesocket(s);
                s = INVALID_SOCKET;
                continue;
            }
        }
        break;
#endif
    }
    freeaddrinfo(res);
    if (s == INVALID_SOCKET) return false;
    int one = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char *>(&one), sizeof(one));
#ifdef _WIN32
    u_long nb = 1;
    ioctlsocket(s, FIONBIO, &nb);
#else
    int fl = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, fl | O_NONBLOCK);
#endif
    impl_->s = s;
    return true;
}

void Tcp5001::close() {
    std::lock_guard<std::mutex> lock(impl_->mu);
    if (impl_->s != INVALID_SOCKET) {
        closesocket(impl_->s);
        impl_->s = INVALID_SOCKET;
    }
}

bool Tcp5001::send(const uint8_t *p, size_t n) {
    std::lock_guard<std::mutex> lock(impl_->mu);
    if (impl_->s == INVALID_SOCKET) return false;
    size_t off = 0;
    auto t0 = std::chrono::steady_clock::now();
    while (off < n) {
#ifdef _WIN32
        int w = ::send(impl_->s, reinterpret_cast<const char *>(p + off),
                       static_cast<int>(n - off), 0);
        if (w == SOCKET_ERROR) {
            int e = WSAGetLastError();
            if (e == WSAEWOULDBLOCK) {
                if (std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0)
                        .count() > 4)
                    return false;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            return false;
        }
#else
        ssize_t w = ::send(impl_->s, p + off, n - off, MSG_NOSIGNAL);
        if (w < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                if (std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - t0)
                        .count() > 4)
                    return false;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            return false;
        }
#endif
        if (w == 0) return false;
        if (w > 0) off += static_cast<size_t>(w);
    }
    return true;
}

size_t Tcp5001::recv(uint8_t *p, size_t n, int timeout_ms) {
    std::lock_guard<std::mutex> lock(impl_->mu);
    if (impl_->s == INVALID_SOCKET || p == nullptr || n == 0) return 0;
    if (timeout_ms < 0) timeout_ms = 0;
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(impl_->s, &rfds);
    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
#ifdef _WIN32
    int r = select(0, &rfds, nullptr, nullptr, &tv);
#else
    int r = select(impl_->s + 1, &rfds, nullptr, nullptr, &tv);
#endif
    if (r <= 0) return 0;
    auto drop = [&]() {
        closesocket(impl_->s);
        impl_->s = INVALID_SOCKET;
    };
#ifdef _WIN32
    int rd = ::recv(impl_->s, reinterpret_cast<char *>(p), static_cast<int>(n), 0);
    if (rd == SOCKET_ERROR) {
        int e = WSAGetLastError();
        if (e == WSAEWOULDBLOCK) return 0;
        drop();
        return 0;
    }
    if (rd == 0) {
        drop();
        return 0;
    }
    return static_cast<size_t>(rd);
#else
    ssize_t rd = ::recv(impl_->s, p, n, 0);
    if (rd < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        drop();
        return 0;
    }
    if (rd == 0) {
        drop();
        return 0;
    }
    return static_cast<size_t>(rd);
#endif
}

bool Tcp5001::isOpen() const {
    if (!impl_) return false;
    std::lock_guard<std::mutex> lock(impl_->mu);
    return impl_->s != INVALID_SOCKET;
}

namespace {

struct ScanRange {
    uint32_t first;
    uint32_t last;
    uint32_t self;
};

void add_scan_range(std::vector<ScanRange> *ranges, uint32_t ip, int prefix) {
    if ((ip >> 24) == 127u) return;
    if ((ip & 0xFFFF0000u) == 0xA9FE0000u) return;
    if (prefix <= 0 || prefix > 30) return;
    int use = prefix < 24 ? 24 : prefix;
    uint32_t mask = use >= 32 ? 0xFFFFFFFFu : (0xFFFFFFFFu << (32 - use));
    uint32_t net = ip & mask;
    uint32_t bcast = net | ~mask;
    if (bcast <= net + 1u) return;
    uint32_t first = net + 1u;
    uint32_t last = bcast - 1u;
    if (last < first || (last - first) > 512u) return;
    for (const auto &r : *ranges) {
        if (r.first == first && r.last == last) return;
    }
    ranges->push_back(ScanRange{first, last, ip});
}

std::vector<ScanRange> local_scan_ranges() {
    std::vector<ScanRange> ranges;
#ifdef _WIN32
    ULONG sz = 16 * 1024;
    std::vector<unsigned char> buf(sz);
    auto *addrs = reinterpret_cast<IP_ADAPTER_ADDRESSES *>(buf.data());
    ULONG flags = GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER;
    ULONG ret = GetAdaptersAddresses(AF_INET, flags, nullptr, addrs, &sz);
    if (ret == ERROR_BUFFER_OVERFLOW) {
        buf.resize(sz);
        addrs = reinterpret_cast<IP_ADAPTER_ADDRESSES *>(buf.data());
        ret = GetAdaptersAddresses(AF_INET, flags, nullptr, addrs, &sz);
    }
    if (ret != NO_ERROR) return ranges;
    for (auto *ad = addrs; ad != nullptr; ad = ad->Next) {
        if (ad->OperStatus != IfOperStatusUp) continue;
        for (auto *u = ad->FirstUnicastAddress; u != nullptr; u = u->Next) {
            if (u->Address.lpSockaddr == nullptr || u->Address.lpSockaddr->sa_family != AF_INET) continue;
            auto *sa = reinterpret_cast<sockaddr_in *>(u->Address.lpSockaddr);
            add_scan_range(&ranges, ntohl(sa->sin_addr.s_addr), static_cast<int>(u->OnLinkPrefixLength));
        }
    }
#else
    ifaddrs *ifa = nullptr;
    if (getifaddrs(&ifa) != 0) return ranges;
    for (auto *p = ifa; p != nullptr; p = p->ifa_next) {
        if (p->ifa_addr == nullptr || p->ifa_netmask == nullptr) continue;
        if ((p->ifa_flags & IFF_UP) == 0 || (p->ifa_flags & IFF_LOOPBACK) != 0) continue;
        if (p->ifa_addr->sa_family != AF_INET) continue;
        auto *sa = reinterpret_cast<sockaddr_in *>(p->ifa_addr);
        auto *nm = reinterpret_cast<sockaddr_in *>(p->ifa_netmask);
        uint32_t mask = ntohl(nm->sin_addr.s_addr);
        int prefix = 0;
        for (uint32_t bit = 0x80000000u; bit != 0 && (mask & bit) != 0; bit >>= 1) ++prefix;
        add_scan_range(&ranges, ntohl(sa->sin_addr.s_addr), prefix);
    }
    freeifaddrs(ifa);
#endif
    return ranges;
}

bool ipv4_less(const std::string &a, const std::string &b) {
    in_addr ia{};
    in_addr ib{};
    if (inet_pton(AF_INET, a.c_str(), &ia) != 1 || inet_pton(AF_INET, b.c_str(), &ib) != 1)
        return a < b;
    return ntohl(ia.s_addr) < ntohl(ib.s_addr);
}

void set_nonblock(SOCKET s) {
#ifdef _WIN32
    u_long nb = 1;
    ioctlsocket(s, FIONBIO, &nb);
#else
    int flags = fcntl(s, F_GETFL, 0);
    if (flags >= 0) fcntl(s, F_SETFL, flags | O_NONBLOCK);
#endif
}

bool connect_pending(int cr) {
    if (cr == 0) return false;
#ifdef _WIN32
    int werr = WSAGetLastError();
    return werr == WSAEWOULDBLOCK || werr == WSAEINPROGRESS;
#else
    return errno == EINPROGRESS || errno == EWOULDBLOCK;
#endif
}

} // namespace

std::vector<std::string> listEthHosts(uint16_t port) {
    std::vector<std::string> found;
    if (port == 0 || !ensureWinsock()) return found;
    std::vector<uint32_t> ips;
    for (const auto &r : local_scan_ranges()) {
        for (uint32_t h = r.first; h <= r.last; ++h) {
            if (h == r.self) continue;
            ips.push_back(htonl(h));
        }
    }
    struct Probe {
        SOCKET s;
        uint32_t ip;
    };
    std::vector<Probe> pending;
    pending.reserve(ips.size());
    auto add_host = [&found](uint32_t ip) {
        in_addr ia{};
        ia.s_addr = ip;
        char text[INET_ADDRSTRLEN]{};
        if (inet_ntop(AF_INET, &ia, text, sizeof(text)) == nullptr) return;
        found.emplace_back(text);
    };
    for (uint32_t ip : ips) {
        SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (s == INVALID_SOCKET) continue;
        set_nonblock(s);
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_addr.s_addr = ip;
        a.sin_port = htons(port);
        int cr = connect(s, reinterpret_cast<sockaddr *>(&a), sizeof(a));
        if (cr == 0) {
            closesocket(s);
            add_host(ip);
            continue;
        }
        if (!connect_pending(cr)) {
            closesocket(s);
            continue;
        }
        pending.push_back(Probe{s, ip});
    }
    if (!pending.empty()) {
#ifdef _WIN32
        std::vector<WSAPOLLFD> pf(pending.size());
        for (size_t i = 0; i < pending.size(); ++i) {
            pf[i].fd = pending[i].s;
            pf[i].events = POLLWRNORM;
            pf[i].revents = 0;
        }
        WSAPoll(pf.data(), static_cast<ULONG>(pending.size()), 250);
#else
        std::vector<pollfd> pf(pending.size());
        for (size_t i = 0; i < pending.size(); ++i) {
            pf[i].fd = pending[i].s;
            pf[i].events = POLLOUT;
            pf[i].revents = 0;
        }
        poll(pf.data(), pending.size(), 250);
#endif
        for (size_t i = 0; i < pending.size(); ++i) {
#ifdef _WIN32
            short ready = pf[i].revents;
            bool writable = (ready & POLLWRNORM) != 0;
#else
            short ready = pf[i].revents;
            bool writable = (ready & POLLOUT) != 0;
#endif
            int err = 0;
            socklen_t len = sizeof(err);
            if (writable || (ready & (POLLERR | POLLHUP)) != 0)
                getsockopt(pending[i].s, SOL_SOCKET, SO_ERROR, reinterpret_cast<char *>(&err), &len);
            closesocket(pending[i].s);
            if (writable && err == 0) add_host(pending[i].ip);
        }
    }
    std::sort(found.begin(), found.end(), ipv4_less);
    found.erase(std::unique(found.begin(), found.end()), found.end());
    return found;
}

} // namespace a3
