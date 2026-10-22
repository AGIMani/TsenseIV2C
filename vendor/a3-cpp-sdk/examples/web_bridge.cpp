#include "a3/A3HandApi.h"
#include "a3/ITransport.h"
#include "a3/Protocol.h"
#include "a3/Tcp5001.h"
#include "a3/UsbCdc.h"
#include "CommandJson.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <exception>
#include <fstream>
#include <iostream>
#include <mutex>
#include <limits>
#include <sstream>
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
#pragma comment(lib, "ws2_32.lib")
using socklen_t = int;
static void sock_close(SOCKET s) { closesocket(s); }
#else
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>
using SOCKET = int;
constexpr int INVALID_SOCKET = -1;
static void sock_close(SOCKET s) { ::close(s); }
#endif

using namespace a3;

static A3HandApi g_hand;
static std::mutex g_log_mu;
static std::mutex g_command_mu;
static std::vector<std::string> g_logs;
static std::string g_webroot;
static int g_http_port = 8080;
static bool g_bind_lan = false;

static void log_line(const std::string &s) {
    std::lock_guard<std::mutex> lock(g_log_mu);
    g_logs.push_back(s);
    if (g_logs.size() > 200) g_logs.erase(g_logs.begin(), g_logs.begin() + 50);
    std::cout << s << std::endl;
}

static std::string json_escape(const std::string &s) {
    std::string o;
    o.reserve(s.size() + 8);
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') {
            o.push_back('\\');
            o.push_back(static_cast<char>(c));
        } else if (c == '\n')
            o += "\\n";
        else if (c == '\r')
            o += "\\r";
        else if (c < 0x20) {
            char b[8];
            std::snprintf(b, sizeof(b), "\\u%04x", c);
            o += b;
        } else
            o.push_back(static_cast<char>(c));
    }
    return o;
}

static std::string json_num(double v) {
    if (!std::isfinite(v)) return "null";
    char b[64];
    std::snprintf(b, sizeof(b), "%.6g", v);
    return b;
}

static std::string json_state() {
    const auto snapshot = g_hand.telemetrySnapshot();
    const auto &self = snapshot.self;
    const auto &prot = snapshot.protect;
    const auto &mit = snapshot.mit;
    const auto &board = snapshot.board;
    const auto &touch = snapshot.touch;
    auto tgt = g_hand.targetSelf();
    auto tmit = g_hand.targetMit();
    std::ostringstream os;
    os << "{\"ok\":true,\"connected\":" << (g_hand.connected() ? "true" : "false")
       << ",\"link\":\"" << json_escape(g_hand.linkName()) << "\""
       << ",\"streaming\":" << (g_hand.streaming() ? "true" : "false")
       << ",\"stream_hz\":" << g_hand.streamHz()
       << ",\"tx_hz\":" << json_num(g_hand.txHz())
       << ",\"rx_hz\":" << json_num(g_hand.rxHz())
       << ",\"rx_ok\":" << g_hand.rxOk()
       << ",\"seq\":" << snapshot.seq
       << ",\"tick_ms\":" << snapshot.tick_ms
       << ",\"fresh\":" << (snapshot.fresh ? "true" : "false")
       << ",\"age_ms\":" << snapshot.age_ms;
    os << ",\"board\":{\"valid\":" << (board.valid ? "true" : "false")
       << ",\"ntc_c\":\"" << json_escape(decodeTempCenti(board.ntc_c_centi)) << "\""
       << ",\"ntc_b\":\"" << json_escape(decodeTempCenti(board.ntc_b_centi)) << "\""
       << ",\"vin24\":\"" << json_escape(decodeVinMv(board.vin24_mV)) << "\"}";
    os << ",\"self\":[";
    for (int i = 0; i < kSelfMotorNum; ++i) {
        if (i) os << ",";
        const auto &s = self[static_cast<size_t>(i)];
        const auto &p = prot[static_cast<size_t>(i)];
        os << "{\"slot\":" << i
           << ",\"bus\":\"" << slotBusName(i) << "\""
           << ",\"id\":" << slotLocalId(i)
           << ",\"kind\":\"" << (slotIsRotary(i) ? "旋转" : "直线") << "\""
           << ",\"unit\":\"" << slotUnit(i) << "\""
           << ",\"target\":" << json_num(tgt[static_cast<size_t>(i)])
           << ",\"pos\":" << json_num(s.pos_act)
           << ",\"pref\":" << json_num(s.pos_ref)
           << ",\"iq\":" << json_num(s.iq_meas)
           << ",\"iqref\":" << json_num(s.iq_ref)
           << ",\"grav\":\"" << json_escape(s.open_spin ? decodeHomingOrTheta(s.homing_or_theta) : decodeGravIq(s.grav_iq_mA)) << "\""
           << ",\"temp\":\"" << json_escape(s.open_spin ? "—" : decodeTempCenti(s.mcu_temp_centi)) << "\""
           << ",\"home\":\"" << json_escape(decodeHomingOrTheta(s.homing_or_theta)) << "\""
           << ",\"online\":" << (s.online ? "true" : "false")
           << ",\"prot\":\"" << json_escape(p.active ? fmtProtect(p.over_temp, p.over_current) : "—")
           << "\"}";
    }
    os << "],\"mit\":[";
    for (int i = 0; i < kMitMotorNum; ++i) {
        if (i) os << ",";
        const auto &m = mit[static_cast<size_t>(i)];
        os << "{\"idx\":" << i
           << ",\"target_pos\":" << json_num(tmit[static_cast<size_t>(i)].pos_out_deg)
           << ",\"target_vel\":" << json_num(tmit[static_cast<size_t>(i)].vel_out_rpm)
           << ",\"target_iq\":" << json_num(tmit[static_cast<size_t>(i)].iq_a)
           << ",\"pos\":" << json_num(m.pos_out_deg)
           << ",\"vel\":" << json_num(m.vel_out_rpm)
           << ",\"iq\":" << json_num(m.iq_a)
           << ",\"vbus\":" << json_num(m.vbus_v)
           << ",\"valid\":" << (m.valid ? "true" : "false")
           << ",\"online\":" << (m.online ? "true" : "false")
           << ",\"state\":\"" << json_escape(fmtMitState(m.state)) << "\""
           << ",\"calib\":\"" << json_escape(fmtMitCalib(m.calib)) << "\""
           << ",\"mode\":\"" << json_escape(fmtMitMode(m.mode)) << "\""
           << ",\"type\":\"" << json_escape(fmtMitType(m.mtype)) << "\""
           << ",\"faults\":\"" << json_escape(fmtMitFaults(m.fault)) << "\"}";
    }
    os << "],\"touch\":[";
    static const int kParts[6][3] = {{0, 1, 2}, {0, 1, 2}, {0, 1, 2}, {0, 1, 2}, {0, 2, -1}, {0, -1, -1}};
    static const char *kNames[] = {"手指1 · CAN1", "手指2 · CAN1", "手指3 · CAN2",
                                   "手指4 · CAN2", "拇指 · CAN3", "手掌 · UART"};
    for (int src = 0; src < kTouchSlotNum; ++src) {
        if (src) os << ",";
        const auto &t = touch[static_cast<size_t>(src)];
        os << "{\"src\":" << src << ",\"name\":\"" << kNames[src] << "\""
           << ",\"online\":" << (t.online ? "true" : "false")
           << ",\"seq\":" << t.seq << ",\"cmd\":" << static_cast<int>(t.cmd)
           << ",\"ack\":\"" << json_escape(fmtTouchAck(t.ack)) << "\",\"parts\":[";
        bool first = true;
        for (int pi = 0; pi < 3; ++pi) {
            int idx = kParts[src][pi];
            if (idx < 0) continue;
            if (!first) os << ",";
            first = false;
            const auto &p = t.parts[static_cast<size_t>(idx)];
            os << "{\"idx\":" << idx << ",\"valid\":" << (p.valid ? "true" : "false")
               << ",\"rows\":" << p.rows << ",\"cols\":" << p.cols
               << ",\"ok\":" << p.ok << ",\"mean\":" << json_num(p.mean)
               << ",\"mx\":" << json_num(p.mx) << ",\"vals\":[";
            for (size_t vi = 0; vi < p.vals.size(); ++vi) {
                if (vi) os << ",";
                os << json_num(p.vals[vi]);
            }
            os << "]}";
        }
        os << "]}";
    }
    os << "],\"logs\":[";
    {
        std::lock_guard<std::mutex> lock(g_log_mu);
        size_t n = g_logs.size();
        size_t start = n > 40 ? n - 40 : 0;
        for (size_t i = start; i < n; ++i) {
            if (i > start) os << ",";
            os << "\"" << json_escape(g_logs[i]) << "\"";
        }
    }
    os << "]}";
    return os.str();
}

static std::string json_ports() {
    auto ports = listUsbPorts();
    std::ostringstream os;
    os << "{\"ok\":true,\"ports\":[";
    for (size_t i = 0; i < ports.size(); ++i) {
        if (i) os << ",";
        os << "{\"name\":\"" << json_escape(ports[i].name) << "\""
           << ",\"desc\":\"" << json_escape(ports[i].desc) << "\""
           << ",\"h7_cdc\":" << (ports[i].h7_cdc ? "true" : "false")
           << ",\"st_cdc\":" << (ports[i].h7_cdc ? "true" : "false") << "}";
    }
    os << "]}";
    return os.str();
}

static std::string json_eth_hosts(uint16_t port) {
    auto hosts = listEthHosts(port);
    log_line("ETH 扫描 :" + std::to_string(port) + " 找到 " + std::to_string(hosts.size()));
    std::ostringstream os;
    os << "{\"ok\":true,\"hosts\":[";
    for (size_t i = 0; i < hosts.size(); ++i) {
        if (i) os << ",";
        os << "\"" << json_escape(hosts[i]) << "\"";
    }
    os << "]}";
    return os.str();
}

static bool json_find_str(const std::string &j, const char *key, std::string *out) {
    CommandJson args; return CommandJson::parse(j, &args) && args.str(key, out);
}
static bool json_find_num(const std::string &j, const char *key, double *out) {
    CommandJson args; return CommandJson::parse(j, &args) && args.num(key, out);
}
static bool json_find_arr(const std::string &j, const char *key, std::vector<double> *out) {
    CommandJson args; return CommandJson::parse(j, &args) && args.arr(key, out);
}
static std::string json_ok(const std::string &msg = {}) {
    if (msg.empty()) return "{\"ok\":true}";
    return std::string("{\"ok\":true,\"msg\":\"") + json_escape(msg) + "\"}";
}

static std::string json_err(const std::string &e) {
    return std::string("{\"ok\":false,\"error\":\"") + json_escape(e) + "\"}";
}

static std::string handle_cmd(const std::string &body) {
    std::lock_guard<std::mutex> command(g_command_mu);
    CommandJson args;
    if (!CommandJson::parse(body, &args)) return json_err("invalid command JSON");
    std::string type;
    if (!json_find_str(body, "type", &type)) return json_err("缺少 type");
    const auto number = [&args](const char *key, double lo, double hi, bool integer = false, bool boolean = false) {
        auto v = args.get(key);
        if (!v || (v->kind != CommandJson::Value::Number && !(boolean && v->kind == CommandJson::Value::Boolean))) return false;
        return v->number >= lo && v->number <= hi && (!integer || std::floor(v->number) == v->number);
    };
    const auto text = [&args](const char *key) {
        auto v = args.get(key); return v && v->kind == CommandJson::Value::String && !v->text.empty();
    };
    const auto array = [&args](const char *key, size_t length, bool bytes = false) {
        auto v = args.get(key); if (!v || v->kind != CommandJson::Value::Array || v->array.size() != length) return false;
        for (double x : v->array)
            if (std::abs(x) > std::numeric_limits<float>::max() || (bytes && (x < 0 || x > 255 || std::floor(x) != x))) return false;
        return true;
    };
    std::string allowed = " type ";
    bool valid = true;
    if (type == "connect") {
        allowed += "link host port "; std::string link;
        valid = args.str("link", &link) && (link == "usb" || link == "eth");
        if (valid && link == "usb") valid = text("port") && !args.get("host");
        if (valid && link == "eth") valid = text("host") && (!args.get("port") || number("port", 1, 65535, true));
    } else if (type == "set_positions") {
        allowed += "self mit_pos mit_vel mit_iq "; valid = array("self", kSelfMotorNum);
        for (auto key : {"mit_pos", "mit_vel", "mit_iq"}) if (args.get(key)) valid = valid && array(key, kMitMotorNum);
        if (auto vel = args.get("mit_vel"))
            for (double value : vel->array) valid = valid && std::abs(value) <= std::numeric_limits<float>::max() / kMitGearRatio;
    } else if (type == "stream") {
        allowed += "on hz "; valid = number("on", 0, 1, true, true);
        double on = 0; args.num("on", &on);
        valid = valid && ((!on && !args.get("hz")) || number("hz", 1, 1000, true));
    } else if (type == "enable" || type == "home") {
        allowed += type == "enable" ? "slot scope on drv " : "slot scope iq_seek ";
        std::string scope; valid = number("slot", 0, kSelfMotorNum-1, true) && args.str("scope", &scope);
        valid = valid && (scope == "slot" || scope == "bus_lin" || scope == "bus_rot" ||
            (type == "enable" && (scope == "hand" || scope == "bus")));
        if (type == "enable") valid = valid && number("on", 0, 1, true, true) && number("drv", 1, 3, true);
        else valid = valid && number("iq_seek", std::numeric_limits<float>::min(), std::numeric_limits<float>::max());
    } else if (type == "pid_pos" || type == "pid_cur") {
        valid = number("slot", 0, kSelfMotorNum-1, true); allowed += "slot ";
        if (type == "pid_pos") {
            allowed += "kp ki kd ";
            for (auto key : {"kp", "ki", "kd"}) valid = valid && number(key, 0, std::numeric_limits<float>::max());
        } else {
            allowed += "id_kp id_ki id_kd iq_kp iq_ki iq_kd ";
            for (auto key : {"id_kp", "id_ki", "id_kd", "iq_kp", "iq_ki", "iq_kd"}) valid = valid && number(key, 0, std::numeric_limits<float>::max());
        }
    } else if (type == "open_spin") {
        allowed += "slot start freq_hz half_s ";
        valid = number("slot", 0, kSelfMotorNum-1, true) && number("start", 0, 1, true, true);
        double start=0; args.num("start", &start);
        for (auto key : {"freq_hz", "half_s"}) if (start || args.get(key)) valid = valid && number(key, std::numeric_limits<float>::min(), std::numeric_limits<float>::max());
    } else if (type == "touch") {
        allowed += "src cmd data "; auto data = args.get("data");
        valid = (number("src", 0, kTouchSlotNum-1, true) || number("src", 255, 255, true)) &&
            number("cmd", 0, 255, true) && data && data->kind == CommandJson::Value::Array &&
            data->array.size() <= 65531 && array("data", data->array.size(), true);
    } else if (type != "zero" && type != "send_once" && type != "disconnect") valid = false;
    for (const auto &field : args.fields)
        if (allowed.find(" " + field.first + " ") == std::string::npos) valid = false;
    if (!valid) return json_err("missing, unknown or invalid command argument");
    std::string err;
    if (type == "connect") {
        std::string link = "usb";
        json_find_str(body, "link", &link);
        if (link == "eth") {
            std::string host;
            if (!json_find_str(body, "host", &host) || host.empty()) return json_err("缺少 host");
            double port = 5001;
            json_find_num(body, "port", &port);
            if (!g_hand.connectEth(host, static_cast<uint16_t>(port), &err)) return json_err(err);
            log_line("已连接 ETH " + host + ":" + std::to_string(static_cast<int>(port)));
            return json_ok("已连接");
        }
        std::string port;
        if (!json_find_str(body, "port", &port)) return json_err("缺少 port");
        if (!g_hand.connectUsb(port, &err)) return json_err(err);
        log_line("已连接 USB " + port);
        return json_ok("已连接");
    }
    if (type == "disconnect") {
        g_hand.disconnect();
        log_line("已断开");
        return json_ok();
    }
    if (type == "set_positions") {
        std::vector<double> self;
        json_find_arr(body, "self", &self);
        float arr[kSelfMotorNum]{};
        auto cur = g_hand.targetSelf();
        for (int i = 0; i < kSelfMotorNum; ++i)
            arr[i] = (static_cast<size_t>(i) < self.size()) ? static_cast<float>(self[static_cast<size_t>(i)])
                                                            : cur[static_cast<size_t>(i)];
        if (!g_hand.setPositions(arr)) return json_err(g_hand.lastError());
        std::vector<double> mpos, mvel, miq;
        json_find_arr(body, "mit_pos", &mpos);
        json_find_arr(body, "mit_vel", &mvel);
        json_find_arr(body, "mit_iq", &miq);
        auto tm = g_hand.targetMit();
        for (int i = 0; i < kMitMotorNum; ++i) {
            float p = (static_cast<size_t>(i) < mpos.size()) ? static_cast<float>(mpos[static_cast<size_t>(i)])
                                                             : tm[static_cast<size_t>(i)].pos_out_deg;
            float v = (static_cast<size_t>(i) < mvel.size()) ? static_cast<float>(mvel[static_cast<size_t>(i)])
                                                             : tm[static_cast<size_t>(i)].vel_out_rpm;
            float q = (static_cast<size_t>(i) < miq.size()) ? static_cast<float>(miq[static_cast<size_t>(i)])
                                                            : tm[static_cast<size_t>(i)].iq_a;
            if (!g_hand.setMit(i, p, v, q)) return json_err(g_hand.lastError());
        }
        return json_ok();
    }
    if (type == "send_once") {
        if (!g_hand.sendOnce(&err)) return json_err(err);
        return json_ok("已发一帧");
    }
    if (type == "stream") {
        double on = 0, hz = 500;
        json_find_num(body, "on", &on);
        json_find_num(body, "hz", &hz);
        if (on) {
            if (!g_hand.startStream(static_cast<int>(hz), &err)) return json_err(err);
            log_line("开始连续 " + std::to_string(static_cast<int>(hz)) + " Hz");
        } else {
            g_hand.stopStream();
            log_line("停止连续");
        }
        return json_ok();
    }
    if (type == "zero") {
        float z[kSelfMotorNum]{};
        g_hand.setPositions(z);
        g_hand.setMit(0, 0, 0, 0);
        g_hand.setMit(1, 0, 0, 0);
        return json_ok("已置零");
    }
    if (type == "enable" || type == "home") {
        double slot = 0, on = 1, drv = 3, iq = 80;
        std::string scope = "slot";
        json_find_num(body, "slot", &slot);
        json_find_num(body, "on", &on);
        json_find_num(body, "drv", &drv);
        json_find_num(body, "iq_seek", &iq);
        json_find_str(body, "scope", &scope);
        uint8_t bus = 0, node = 0;
        if (!slotToBusNode(static_cast<int>(slot), &bus, &node)) return json_err("invalid slot");
        auto drv_u = static_cast<uint8_t>(drv);
        bool ok = true;
        if (type == "enable") {
            bool en = on != 0;
            if (scope == "hand") ok = g_hand.enable(kBusAll, kNodeAll, en, drv_u, &err);
            else if (scope == "bus") ok = g_hand.enable(bus, kNodeAll, en, drv_u, &err);
            else if (scope == "bus_lin") ok = g_hand.enableBusKind(bus, false, en, drv_u, &err);
            else if (scope == "bus_rot") ok = g_hand.enableBusKind(bus, true, en, drv_u, &err);
            else ok = g_hand.enableSlot(static_cast<int>(slot), en, drv_u, &err);
            if (ok) log_line(std::string(en ? "使能 " : "断电 ") + scope);
        } else {
            if (scope == "bus_lin") ok = g_hand.homeBusKind(bus, false, IqSeekCounts{static_cast<float>(iq)}, &err);
            else if (scope == "bus_rot") ok = g_hand.homeBusKind(bus, true, IqSeekCounts{static_cast<float>(iq)}, &err);
            else ok = g_hand.homeSlot(static_cast<int>(slot), IqSeekCounts{static_cast<float>(iq)}, &err);
            if (ok) log_line("回零 " + scope + " iq_seek=" + std::to_string(iq));
        }
        return ok ? json_ok() : json_err(err);
    }
    if (type == "pid_pos") {
        double slot = 0, kp = 0, ki = 0, kd = 0;
        json_find_num(body, "slot", &slot);
        json_find_num(body, "kp", &kp);
        json_find_num(body, "ki", &ki);
        json_find_num(body, "kd", &kd);
        if (!g_hand.setPosPid(static_cast<int>(slot), static_cast<float>(kp), static_cast<float>(ki),
                              static_cast<float>(kd), &err))
            return json_err(err);
        log_line("位置 PID 槽 " + std::to_string(static_cast<int>(slot)));
        return json_ok();
    }
    if (type == "pid_cur") {
        double slot = 0, a = 0, b = 0, c = 0, d = 0, e = 0, f = 0;
        json_find_num(body, "slot", &slot);
        json_find_num(body, "id_kp", &a);
        json_find_num(body, "id_ki", &b);
        json_find_num(body, "id_kd", &c);
        json_find_num(body, "iq_kp", &d);
        json_find_num(body, "iq_ki", &e);
        json_find_num(body, "iq_kd", &f);
        if (!g_hand.setCurPid(static_cast<int>(slot), static_cast<float>(a), static_cast<float>(b),
                              static_cast<float>(c), static_cast<float>(d), static_cast<float>(e),
                              static_cast<float>(f), &err))
            return json_err(err);
        log_line("电流 PID 槽 " + std::to_string(static_cast<int>(slot)));
        return json_ok();
    }
    if (type == "open_spin") {
        double slot = 0, start = 1, freq = 2, half = 1;
        json_find_num(body, "slot", &slot);
        json_find_num(body, "start", &start);
        json_find_num(body, "freq_hz", &freq);
        json_find_num(body, "half_s", &half);
        if (!g_hand.openSpin(static_cast<int>(slot), start != 0, static_cast<float>(freq),
                             static_cast<float>(half), &err))
            return json_err(err);
        log_line(start ? "开环开始" : "开环停止");
        return json_ok();
    }
    if (type == "touch") {
        double src = 0, cmd = 2;
        json_find_num(body, "src", &src);
        json_find_num(body, "cmd", &cmd);
        std::vector<double> data;
        json_find_arr(body, "data", &data);
        std::vector<uint8_t> raw;
        for (double v : data) raw.push_back(static_cast<uint8_t>(static_cast<int>(v) & 0xFF));
        if (!g_hand.sendTouch(static_cast<uint8_t>(src), static_cast<uint8_t>(cmd),
                              raw.empty() ? nullptr : raw.data(), raw.size(), &err))
            return json_err(err);
        log_line("触觉 cmd=0x" + std::to_string(static_cast<int>(cmd)));
        return json_ok();
    }
    return json_err("未知命令 " + type);
}

static std::string mime_of(const std::string &path) {
    if (path.size() >= 5 && path.substr(path.size() - 5) == ".html") return "text/html; charset=utf-8";
    if (path.size() >= 3 && path.substr(path.size() - 3) == ".js") return "application/javascript; charset=utf-8";
    if (path.size() >= 4 && path.substr(path.size() - 4) == ".css") return "text/css; charset=utf-8";
    if (path.size() >= 4 && path.substr(path.size() - 4) == ".svg") return "image/svg+xml";
    if (path.size() >= 4 && path.substr(path.size() - 4) == ".png") return "image/png";
    if (path.size() >= 4 && path.substr(path.size() - 4) == ".jpg") return "image/jpeg";
    if (path.size() >= 5 && path.substr(path.size() - 5) == ".jpeg") return "image/jpeg";
    if (path.size() >= 4 && path.substr(path.size() - 4) == ".ico") return "image/x-icon";
    return "application/octet-stream";
}

static bool read_file(const std::string &path, std::string *out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    std::ostringstream ss;
    ss << in.rdbuf();
    *out = ss.str();
    return true;
}

static std::string exe_dir() {
#ifdef _WIN32
    char buf[MAX_PATH]{};
    GetModuleFileNameA(nullptr, buf, MAX_PATH);
    std::string p = buf;
    auto s = p.find_last_of("\\/");
    return s == std::string::npos ? "." : p.substr(0, s);
#else
    char buf[4096]{};
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return ".";
    std::string p(buf, static_cast<size_t>(n));
    auto s = p.find_last_of('/');
    return s == std::string::npos ? "." : p.substr(0, s);
#endif
}

static std::string http_response(int code, const char *status, const std::string &ctype,
                                 const std::string &body) {
    std::ostringstream os;
    os << "HTTP/1.1 " << code << " " << status << "\r\n"
       << "Content-Type: " << ctype << "\r\n"
       << "Content-Length: " << body.size() << "\r\n"
       << "Cache-Control: no-store\r\n"
       << "Connection: close\r\n\r\n"
       << body;
    return os.str();
}

static void handle_client(SOCKET c) {
    std::string req;
    char buf[4096];
    while (req.find("\r\n\r\n") == std::string::npos) {
#ifdef _WIN32
        int n = recv(c, buf, sizeof(buf), 0);
#else
        ssize_t n = recv(c, buf, sizeof(buf), 0);
#endif
        if (n <= 0) {
            sock_close(c);
            return;
        }
        req.append(buf, static_cast<size_t>(n));
        if (req.size() > 2 * 1024 * 1024) {
            sock_close(c);
            return;
        }
    }
    auto hdr_end = req.find("\r\n\r\n");
    std::string headers = req.substr(0, hdr_end);
    std::string body = req.substr(hdr_end + 4);
    constexpr size_t kMaxHttpBody = 256 * 1024;
    size_t clen = 0;
    auto cl = headers.find("Content-Length:");
    if (cl == std::string::npos) cl = headers.find("content-length:");
    if (cl != std::string::npos) clen = static_cast<size_t>(std::strtoul(headers.c_str() + cl + 15, nullptr, 10));
    bool body_too_big = clen > kMaxHttpBody || body.size() > kMaxHttpBody;
    if (!body_too_big) {
        while (body.size() < clen) {
#ifdef _WIN32
            int n = recv(c, buf, sizeof(buf), 0);
#else
            ssize_t n = recv(c, buf, sizeof(buf), 0);
#endif
            if (n <= 0) break;
            body.append(buf, static_cast<size_t>(n));
            if (body.size() > kMaxHttpBody) {
                body_too_big = true;
                break;
            }
        }
    }
    std::istringstream hs(headers);
    std::string method, path, ver;
    hs >> method >> path >> ver;
    std::string query;
    auto q = path.find('?');
    if (q != std::string::npos) {
        query = path.substr(q + 1);
        path = path.substr(0, q);
    }
    std::string resp;
    if (body_too_big) {
        resp = http_response(413, "Payload Too Large", "text/plain", "too large");
    } else if (method == "OPTIONS") {
        resp = http_response(204, "No Content", "text/plain", "");
    } else if (method == "GET" && path == "/api/state") {
        resp = http_response(200, "OK", "application/json; charset=utf-8", json_state());
    } else if (method == "GET" && path == "/api/ports") {
        resp = http_response(200, "OK", "application/json; charset=utf-8", json_ports());
    } else if (method == "GET" && path == "/api/eth-hosts") {
        uint16_t port = 5001;
        auto p = query.find("port=");
        if (p != std::string::npos) {
            int v = std::atoi(query.c_str() + p + 5);
            if (v > 0 && v <= 65535) port = static_cast<uint16_t>(v);
        }
        try {
            resp = http_response(200, "OK", "application/json; charset=utf-8", json_eth_hosts(port));
        } catch (const std::exception &) {
            resp = http_response(500, "Error", "application/json; charset=utf-8",
                                 "{\"ok\":false,\"error\":\"eth scan failed\"}");
        }
    } else if (method == "POST" && path == "/api/cmd") {
        resp = http_response(200, "OK", "application/json; charset=utf-8", handle_cmd(body));
    } else if (method == "GET") {
        std::string rel = (path == "/" ? "/index.html" : path);
        if (rel.find("..") != std::string::npos) {
            resp = http_response(403, "Forbidden", "text/plain", "forbidden");
        } else {
            std::string file = g_webroot + rel;
#ifdef _WIN32
            std::replace(file.begin(), file.end(), '/', '\\');
#endif
            std::string content;
            if (read_file(file, &content))
                resp = http_response(200, "OK", mime_of(rel), content);
            else
                resp = http_response(404, "Not Found", "text/plain; charset=utf-8", "not found");
        }
    } else {
        resp = http_response(405, "Method Not Allowed", "text/plain", "no");
    }
    size_t off = 0;
    while (off < resp.size()) {
#ifdef _WIN32
        const int n = ::send(c, resp.data() + off, static_cast<int>(resp.size() - off), 0);
        if (n <= 0) break;
        off += static_cast<size_t>(n);
#else
        const ssize_t n = ::send(c, resp.data() + off, resp.size() - off, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (n == 0) break;
        off += static_cast<size_t>(n);
#endif
    }
    sock_close(c);
}

int main(int argc, char **argv) {
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "--port" || a == "--http") && i + 1 < argc) g_http_port = std::atoi(argv[++i]);
        else if (a == "--webroot" && i + 1 < argc) g_webroot = argv[++i];
        else if (a == "--lan") g_bind_lan = true;
        else if (a == "-h" || a == "--help") {
            std::cout << "a3_web_bridge [--port 8080] [--webroot path] [--lan]\n"
                      << "  default bind 127.0.0.1 (no auth). --lan binds all interfaces.\n";
            return 0;
        }
    }
    if (g_webroot.empty()) {
        std::string dir = exe_dir();
        const char *cands[] = {"/webui/static", "/../webui/static", "/../../webui/static"};
        for (const char *c : cands) {
            std::string p = dir + c;
            std::ifstream t(p + "/index.html");
            if (t) {
                g_webroot = p;
                break;
            }
        }
        if (g_webroot.empty()) g_webroot = dir + "/webui/static";
    }
#ifndef _WIN32
    ::signal(SIGPIPE, SIG_IGN);
#endif
    if (!ensureWinsock()) {
        std::cerr << "WSAStartup 失败\n";
        return 1;
    }
    g_hand.setDisconnectCallback([](Err, const std::string &reason) { log_line("断开: " + reason); });
    SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s == INVALID_SOCKET) {
        std::cerr << "socket 失败\n";
        return 1;
    }
    int one = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&one), sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(g_bind_lan ? INADDR_ANY : INADDR_LOOPBACK);
    addr.sin_port = htons(static_cast<uint16_t>(g_http_port));
    if (bind(s, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
        std::cerr << "bind :" << g_http_port << " 失败\n";
        return 1;
    }
    listen(s, 16);
    std::cout << "A3 HTML 上位机  http://127.0.0.1:" << g_http_port << "/\n";
    if (g_bind_lan)
        std::cout << "警告: --lan 监听所有网卡，无鉴权，可远程使能电机\n";
    std::cout << "静态目录  " << g_webroot << "\n";
    for (;;) {
        sockaddr_in cli{};
        socklen_t n = sizeof(cli);
        SOCKET c = accept(s, reinterpret_cast<sockaddr *>(&cli), &n);
        if (c == INVALID_SOCKET) continue;
        try {
            std::thread(handle_client, c).detach();
        } catch (...) {
            sock_close(c);
        }
    }
}
