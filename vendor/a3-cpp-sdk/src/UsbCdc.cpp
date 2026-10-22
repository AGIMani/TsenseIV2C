#include "a3/UsbCdc.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <cwchar>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <initguid.h>
#include <devguid.h>
#include <setupapi.h>
#pragma comment(lib, "setupapi.lib")
#pragma comment(lib, "uuid.lib")
#else
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace a3 {

#ifdef _WIN32

struct UsbCdc::Impl {
    HANDLE h = INVALID_HANDLE_VALUE;
    mutable std::mutex mu;
};

static std::string narrow(const wchar_t *w) {
    if (!w) return {};
    int n = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
    if (n <= 1) return {};
    std::string s(static_cast<size_t>(n - 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, w, -1, &s[0], n, nullptr, nullptr);
    return s;
}

static std::string trim_copy(std::string s) {
    while (!s.empty() && (s.back() == '\0' || s.back() == ' ' || s.back() == '\t')) s.pop_back();
    size_t i = 0;
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
    return s.substr(i);
}

static std::string upper_ascii(std::string s) {
    for (char &c : s)
        if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 32);
    return s;
}

static bool is_com_name(const std::string &n) {
    if (n.size() < 4) return false;
    if (n[0] != 'C' && n[0] != 'c') return false;
    if (n[1] != 'O' && n[1] != 'o') return false;
    if (n[2] != 'M' && n[2] != 'm') return false;
    if (!std::isdigit(static_cast<unsigned char>(n[3]))) return false;
    for (size_t i = 3; i < n.size(); ++i) {
        if (!std::isdigit(static_cast<unsigned char>(n[i]))) return false;
    }
    return true;
}

static std::string canon_com(std::string n) {
    n = trim_copy(std::move(n));
    if (!is_com_name(n)) return {};
    return "COM" + n.substr(3);
}

static std::string com_in_text(const std::string &s) {
    const std::string u = upper_ascii(s);
    size_t p = u.find("(COM");
    if (p == std::string::npos) p = u.find("COM");
    else p += 1;
    if (p == std::string::npos) return {};
    size_t i = p + 3;
    if (i >= u.size() || !std::isdigit(static_cast<unsigned char>(u[i]))) return {};
    size_t j = i;
    while (j < u.size() && std::isdigit(static_cast<unsigned char>(u[j]))) ++j;
    return "COM" + u.substr(i, j - i);
}

static std::string read_prop_sz(HDEVINFO info, SP_DEVINFO_DATA *dev, DWORD prop) {
    wchar_t buf[1024]{};
    DWORD type = 0;
    if (!SetupDiGetDeviceRegistryPropertyW(info, dev, prop, &type, reinterpret_cast<PBYTE>(buf),
                                           sizeof(buf), nullptr)) {
        return {};
    }
    if (type == REG_MULTI_SZ) {
        std::string out;
        for (const wchar_t *p = buf; *p; p += std::wcslen(p) + 1) {
            if (!out.empty()) out += ' ';
            out += narrow(p);
            if (p + 1 >= buf + (sizeof(buf) / sizeof(buf[0])) - 1) break;
        }
        return out;
    }
    return narrow(buf);
}

static std::string read_instance_id(HDEVINFO info, SP_DEVINFO_DATA *dev) {
    wchar_t buf[512]{};
    if (!SetupDiGetDeviceInstanceIdW(info, dev, buf, static_cast<DWORD>(sizeof(buf) / sizeof(buf[0])),
                                     nullptr)) {
        return {};
    }
    return narrow(buf);
}

static bool contains_ci(const std::string &hay, const char *needle) {
    const std::string u = upper_ascii(hay);
    const std::string n = upper_ascii(needle);
    return u.find(n) != std::string::npos;
}

static bool is_stlink_text(const std::string &text) {
    return contains_ci(text, "STLINK") || contains_ci(text, "ST-LINK") || contains_ci(text, "ST LINK");
}

static bool hwid_is_h7(const std::string &hwid) {
    const std::string u = upper_ascii(hwid);
    std::string compact;
    compact.reserve(u.size());
    for (char c : u)
        if (c != ' ') compact.push_back(c);
    return compact.find("VID_0483") != std::string::npos && compact.find("PID_5740") != std::string::npos;
}

static bool is_h7_cdc(const std::string &desc, const std::string &hwid) {
    const std::string all = desc + " " + hwid;
    if (is_stlink_text(all)) return false;
    if (hwid_is_h7(hwid) || hwid_is_h7(desc)) return true;
    if (desc.find("USB 串行设备") != std::string::npos) return true;
    const std::string u = upper_ascii(desc);
    if (u.find("USB SERIAL DEVICE") != std::string::npos) return true;
    if (u.find("STM32 VIRTUAL COM") != std::string::npos) return true;
    return false;
}

static int com_num(const std::string &n) {
    if (n.size() < 4) return 0;
    return std::atoi(n.c_str() + 3);
}

static void upsert_port(std::unordered_map<std::string, ComPort> &by,
                        std::string name, std::string desc, std::string hwid = {}) {
    name = canon_com(std::move(name));
    if (name.empty()) return;
    const bool h7 = is_h7_cdc(desc, hwid);
    auto it = by.find(name);
    if (it == by.end()) {
        ComPort p;
        p.name = name;
        p.desc = std::move(desc);
        p.h7_cdc = h7;
        by.emplace(std::move(name), std::move(p));
        return;
    }
    if (it->second.desc.empty() && !desc.empty()) it->second.desc = desc;
    else if (!desc.empty() && desc.size() > it->second.desc.size()) it->second.desc = std::move(desc);
    it->second.h7_cdc = it->second.h7_cdc || h7;
}

static void collect_class_ports(std::unordered_map<std::string, ComPort> &by) {
    HDEVINFO info = SetupDiGetClassDevsW(&GUID_DEVCLASS_PORTS, nullptr, nullptr, DIGCF_PRESENT);
    if (info == INVALID_HANDLE_VALUE) return;
    SP_DEVINFO_DATA dev{};
    dev.cbSize = sizeof(dev);
    for (DWORD i = 0; SetupDiEnumDeviceInfo(info, i, &dev); ++i) {
        wchar_t desc[256]{};
        wchar_t friendly[256]{};
        SetupDiGetDeviceRegistryPropertyW(info, &dev, SPDRP_DEVICEDESC, nullptr,
                                          reinterpret_cast<PBYTE>(desc), sizeof(desc), nullptr);
        SetupDiGetDeviceRegistryPropertyW(info, &dev, SPDRP_FRIENDLYNAME, nullptr,
                                          reinterpret_cast<PBYTE>(friendly), sizeof(friendly), nullptr);
        const std::string d = narrow(friendly[0] ? friendly : desc);
        const std::string hwid = read_prop_sz(info, &dev, SPDRP_HARDWAREID) + " " +
                                 read_prop_sz(info, &dev, SPDRP_COMPATIBLEIDS) + " " +
                                 read_instance_id(info, &dev);
        std::string name;
        HKEY hk = SetupDiOpenDevRegKey(info, &dev, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
        if (hk != INVALID_HANDLE_VALUE) {
            wchar_t portw[64]{};
            DWORD type = 0, sz = sizeof(portw);
            if (RegQueryValueExW(hk, L"PortName", nullptr, &type, reinterpret_cast<LPBYTE>(portw), &sz) ==
                ERROR_SUCCESS) {
                name = narrow(portw);
            }
            RegCloseKey(hk);
        }
        if (!is_com_name(trim_copy(name))) name = com_in_text(d);
        upsert_port(by, std::move(name), d, hwid);
    }
    SetupDiDestroyDeviceInfoList(info);
}

static void collect_interface_ports(std::unordered_map<std::string, ComPort> &by) {
    static const GUID kComPortIface = {
        0x86e0d1e0L, 0x8089, 0x11d0, {0x9c, 0xe4, 0x08, 0x00, 0x3e, 0x30, 0x1f, 0x73}};
    HDEVINFO info = SetupDiGetClassDevsW(&kComPortIface, nullptr, nullptr,
                                         DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (info == INVALID_HANDLE_VALUE) return;
    SP_DEVICE_INTERFACE_DATA ifd{};
    ifd.cbSize = sizeof(ifd);
    for (DWORD i = 0; SetupDiEnumDeviceInterfaces(info, nullptr, &kComPortIface, i, &ifd); ++i) {
        DWORD need = 0;
        SetupDiGetDeviceInterfaceDetailW(info, &ifd, nullptr, 0, &need, nullptr);
        if (need < sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W)) continue;
        std::vector<uint8_t> buf(need);
        auto *detail = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W *>(buf.data());
        detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
        SP_DEVINFO_DATA dev{};
        dev.cbSize = sizeof(dev);
        if (!SetupDiGetDeviceInterfaceDetailW(info, &ifd, detail, need, nullptr, &dev)) continue;
        wchar_t friendly[256]{};
        SetupDiGetDeviceRegistryPropertyW(info, &dev, SPDRP_FRIENDLYNAME, nullptr,
                                          reinterpret_cast<PBYTE>(friendly), sizeof(friendly), nullptr);
        const std::string d = narrow(friendly);
        const std::string hwid = read_prop_sz(info, &dev, SPDRP_HARDWAREID) + " " +
                                 read_prop_sz(info, &dev, SPDRP_COMPATIBLEIDS) + " " +
                                 read_instance_id(info, &dev);
        std::string name;
        HKEY hk = SetupDiOpenDevRegKey(info, &dev, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
        if (hk != INVALID_HANDLE_VALUE) {
            wchar_t portw[64]{};
            DWORD type = 0, sz = sizeof(portw);
            if (RegQueryValueExW(hk, L"PortName", nullptr, &type, reinterpret_cast<LPBYTE>(portw), &sz) ==
                ERROR_SUCCESS) {
                name = narrow(portw);
            }
            RegCloseKey(hk);
        }
        if (!is_com_name(trim_copy(name))) name = com_in_text(d);
        upsert_port(by, std::move(name), d, hwid);
    }
    SetupDiDestroyDeviceInfoList(info);
}

static void collect_dos_devices(std::unordered_map<std::string, ComPort> &by) {
    DWORD cap = 65536;
    std::vector<char> buf(cap);
    DWORD n = QueryDosDeviceA(nullptr, buf.data(), cap);
    if (n == 0 && GetLastError() == ERROR_INSUFFICIENT_BUFFER) {
        cap = 1 << 20;
        buf.assign(cap, 0);
        n = QueryDosDeviceA(nullptr, buf.data(), cap);
    }
    if (n == 0) return;
    for (size_t i = 0; i + 1 < n;) {
        const char *s = buf.data() + i;
        const size_t len = std::strlen(s);
        if (len == 0) break;
        if (is_com_name(s)) upsert_port(by, s, {});
        i += len + 1;
    }
}

static void collect_get_comm_ports(std::unordered_map<std::string, ComPort> &by) {
    using Fn = ULONG(WINAPI *)(PULONG, ULONG, PULONG);
    auto fn = reinterpret_cast<Fn>(GetProcAddress(GetModuleHandleW(L"kernel32.dll"), "GetCommPorts"));
    if (!fn) return;
    ULONG ports[256]{};
    ULONG got = 0;
    const ULONG err = fn(ports, 256, &got);
    if (err != ERROR_SUCCESS && err != ERROR_MORE_DATA && err != ERROR_INSUFFICIENT_BUFFER) return;
    for (ULONG i = 0; i < got && i < 256; ++i) {
        upsert_port(by, "COM" + std::to_string(ports[i]), {});
    }
}

std::vector<ComPort> listUsbPorts() {
    std::unordered_map<std::string, ComPort> by;
    collect_class_ports(by);
    collect_interface_ports(by);
    collect_dos_devices(by);
    collect_get_comm_ports(by);
    std::vector<ComPort> out;
    out.reserve(by.size());
    for (auto &kv : by) out.push_back(std::move(kv.second));
    std::sort(out.begin(), out.end(), [](const ComPort &a, const ComPort &b) {
        if (a.h7_cdc != b.h7_cdc) return a.h7_cdc > b.h7_cdc;
        const int na = com_num(a.name), nb = com_num(b.name);
        if (na != nb) return na < nb;
        return a.name < b.name;
    });
    return out;
}

UsbCdc::UsbCdc(std::string port) : port_(std::move(port)), impl_(std::make_unique<Impl>()) {}
UsbCdc::~UsbCdc() { close(); }

bool UsbCdc::open() {
    close();
    std::string path = port_;
    if (path.rfind("\\\\.\\", 0) != 0) path = std::string("\\\\.\\") + port_;
    HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                           OPEN_EXISTING, 0, nullptr);
    if (h == INVALID_HANDLE_VALUE) return false;
    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb)) {
        CloseHandle(h);
        return false;
    }
    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    SetCommState(h, &dcb);
    COMMTIMEOUTS to{};
    to.ReadIntervalTimeout = MAXDWORD;
    to.ReadTotalTimeoutMultiplier = 0;
    to.ReadTotalTimeoutConstant = 0;
    to.WriteTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant = 1000;
    SetCommTimeouts(h, &to);
    SetupComm(h, 65536, 65536);
    EscapeCommFunction(h, SETDTR);
    EscapeCommFunction(h, CLRRTS);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    impl_->h = h;
    return true;
}

void UsbCdc::close() {
    std::lock_guard<std::mutex> lock(impl_->mu);
    if (impl_->h != INVALID_HANDLE_VALUE) {
        CloseHandle(impl_->h);
        impl_->h = INVALID_HANDLE_VALUE;
    }
}

bool UsbCdc::send(const uint8_t *p, size_t n) {
    std::lock_guard<std::mutex> lock(impl_->mu);
    if (impl_->h == INVALID_HANDLE_VALUE) return false;
    DWORD wr = 0;
    if (!WriteFile(impl_->h, p, static_cast<DWORD>(n), &wr, nullptr)) return false;
    return wr == n;
}

size_t UsbCdc::recv(uint8_t *p, size_t n, int timeout_ms) {
    std::lock_guard<std::mutex> lock(impl_->mu);
    if (impl_->h == INVALID_HANDLE_VALUE) return 0;
    if (timeout_ms > 0) {
        auto t0 = std::chrono::steady_clock::now();
        while (true) {
            COMSTAT st{};
            DWORD err = 0;
            if (!ClearCommError(impl_->h, &err, &st)) {
                CloseHandle(impl_->h);
                impl_->h = INVALID_HANDLE_VALUE;
                return 0;
            }
            if (st.cbInQue > 0) break;
            auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - t0)
                          .count();
            if (dt >= timeout_ms) return 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    DWORD rd = 0;
    if (!ReadFile(impl_->h, p, static_cast<DWORD>(n), &rd, nullptr)) {
        CloseHandle(impl_->h);
        impl_->h = INVALID_HANDLE_VALUE;
        return 0;
    }
    return static_cast<size_t>(rd);
}

bool UsbCdc::isOpen() const {
    std::lock_guard<std::mutex> lock(impl_->mu);
    return impl_->h != INVALID_HANDLE_VALUE;
}

#else

struct UsbCdc::Impl {
    int fd = -1;
    mutable std::mutex mu;
};

static bool tty_index_name(const std::string &n, const char *prefix) {
    const size_t plen = std::strlen(prefix);
    if (n.size() <= plen || n.compare(0, plen, prefix) != 0) return false;
    for (size_t i = plen; i < n.size(); ++i) {
        if (n[i] < '0' || n[i] > '9') return false;
    }
    return true;
}

static bool read_sysfs_line(const std::string &path, std::string *out) {
    std::ifstream in(path);
    if (!in) return false;
    std::string s;
    if (!std::getline(in, s)) return false;
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ' || s.back() == '\t'))
        s.pop_back();
    if (s.empty()) return false;
    *out = std::move(s);
    return true;
}

static unsigned parse_hex(const std::string &s) {
    char *end = nullptr;
    const unsigned long v = std::strtoul(s.c_str(), &end, 16);
    if (end == s.c_str()) return 0;
    return static_cast<unsigned>(v);
}

static std::string upper_ascii_copy(std::string s) {
    for (char &c : s)
        if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 32);
    return s;
}

static bool text_has_stlink(const std::string &text) {
    const std::string u = upper_ascii_copy(text);
    return u.find("STLINK") != std::string::npos || u.find("ST-LINK") != std::string::npos ||
           u.find("ST LINK") != std::string::npos;
}

static std::string hex4(unsigned v) {
    static const char *kDig = "0123456789abcdef";
    std::string s(4, '0');
    for (int i = 3; i >= 0; --i) {
        s[static_cast<size_t>(i)] = kDig[v & 0xFu];
        v >>= 4;
    }
    return s;
}

static std::string resolved_path(const std::string &path) {
    char buf[4096];
    if (!realpath(path.c_str(), buf)) return {};
    return buf;
}

struct UsbIdentity {
    unsigned vid = 0;
    unsigned pid = 0;
    bool have_id = false;
    std::string product;
    std::string manufacturer;
    std::string serial;
};

static UsbIdentity usb_identity(const std::string &tty) {
    UsbIdentity id;
    std::string dir = resolved_path("/sys/class/tty/" + tty + "/device");
    for (int hop = 0; hop < 8 && !dir.empty(); ++hop) {
        std::string vend;
        std::string prod;
        if (read_sysfs_line(dir + "/idVendor", &vend) && read_sysfs_line(dir + "/idProduct", &prod)) {
            id.vid = parse_hex(vend);
            id.pid = parse_hex(prod);
            id.have_id = true;
            read_sysfs_line(dir + "/product", &id.product);
            read_sysfs_line(dir + "/manufacturer", &id.manufacturer);
            read_sysfs_line(dir + "/serial", &id.serial);
            return id;
        }
        const auto slash = dir.find_last_of('/');
        if (slash == std::string::npos || slash == 0) break;
        dir.resize(slash);
    }
    return id;
}

std::vector<ComPort> listUsbPorts() {
    std::vector<ComPort> out;
    DIR *d = opendir("/dev");
    if (!d) return out;
    while (dirent *e = readdir(d)) {
        const std::string n = e->d_name;
        if (!tty_index_name(n, "ttyACM") && !tty_index_name(n, "ttyUSB")) continue;
        const UsbIdentity id = usb_identity(n);
        ComPort p;
        p.name = "/dev/" + n;
        const std::string label = !id.product.empty() ? id.product
                                  : !id.manufacturer.empty() ? id.manufacturer
                                                             : std::string("USB CDC");
        if (id.have_id) {
            p.desc = label + " (" + hex4(id.vid) + ":" + hex4(id.pid) + ")";
            if (!id.serial.empty()) p.desc += " " + id.serial;
            const bool stlink = text_has_stlink(id.product + " " + id.manufacturer);
            p.h7_cdc = id.vid == 0x0483u && id.pid == 0x5740u && !stlink;
        } else {
            p.desc = n;
            p.h7_cdc = false;
        }
        out.push_back(std::move(p));
    }
    closedir(d);
    std::sort(out.begin(), out.end(), [](const ComPort &a, const ComPort &b) {
        if (a.h7_cdc != b.h7_cdc) return a.h7_cdc > b.h7_cdc;
        return a.name < b.name;
    });
    return out;
}

UsbCdc::UsbCdc(std::string port) : port_(std::move(port)), impl_(std::make_unique<Impl>()) {}
UsbCdc::~UsbCdc() { close(); }

bool UsbCdc::open() {
    close();
    int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return false;
    if (!isatty(fd)) {
        ::close(fd);
        return false;
    }
    termios tio{};
    if (tcgetattr(fd, &tio) != 0) {
        ::close(fd);
        return false;
    }
    cfmakeraw(&tio);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tio.c_cflag |= CLOCAL | CREAD;
#ifdef CRTSCTS
    tio.c_cflag &= ~CRTSCTS;
#endif
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        ::close(fd);
        return false;
    }
    ioctl(fd, TIOCEXCL);
    int dtr = TIOCM_DTR;
    ioctl(fd, TIOCMBIS, &dtr);
    int rts = TIOCM_RTS;
    ioctl(fd, TIOCMBIC, &rts);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    tcflush(fd, TCIOFLUSH);
    impl_->fd = fd;
    return true;
}

void UsbCdc::close() {
    std::lock_guard<std::mutex> lock(impl_->mu);
    if (impl_->fd >= 0) {
        ::close(impl_->fd);
        impl_->fd = -1;
    }
}

bool UsbCdc::send(const uint8_t *p, size_t n) {
    std::lock_guard<std::mutex> lock(impl_->mu);
    if (impl_->fd < 0) return false;
    size_t off = 0;
    const auto t0 = std::chrono::steady_clock::now();
    while (off < n) {
        const ssize_t w = ::write(impl_->fd, p + off, n - off);
        if (w < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now() - t0)
                                    .count();
                if (dt > 1000) return false;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            return false;
        }
        if (w == 0) return false;
        off += static_cast<size_t>(w);
    }
    return true;
}

size_t UsbCdc::recv(uint8_t *p, size_t n, int timeout_ms) {
    std::lock_guard<std::mutex> lock(impl_->mu);
    if (impl_->fd < 0 || n == 0) return 0;
    if (timeout_ms > 0) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(impl_->fd, &rfds);
        timeval tv{};
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        const int r = ::select(impl_->fd + 1, &rfds, nullptr, nullptr, &tv);
        if (r == 0) return 0;
        if (r < 0) {
            if (errno == EINTR) return 0;
            ::close(impl_->fd);
            impl_->fd = -1;
            return 0;
        }
    }
    const ssize_t rd = ::read(impl_->fd, p, n);
    if (rd < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
        ::close(impl_->fd);
        impl_->fd = -1;
        return 0;
    }
    return static_cast<size_t>(rd);
}

bool UsbCdc::isOpen() const {
    std::lock_guard<std::mutex> lock(impl_->mu);
    return impl_->fd >= 0;
}

#endif

} // namespace a3
