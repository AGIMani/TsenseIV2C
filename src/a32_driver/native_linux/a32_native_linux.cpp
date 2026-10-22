// Linux version-2 C ABI adapter using the bundled A3 SDK protocol source.
// Native POSIX transport; no automatic enable, disable or motion.
#include "protocol.hpp"
#include "a3/Stroke.h"
#include "a3/Types.h"
#include "a3/Version.h"
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#define API extern "C" __attribute__((visibility("default")))
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
static int64_t nowNs() { return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count(); }
static int64_t nowMs() { return nowNs()/1000000; }
static int64_t unixMs() { return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count(); }
static std::string ioError(const char* what) { return std::string(what)+": "+std::strerror(errno); }
struct Failure : std::runtime_error {
    int code;
    Failure(int value, const std::string& message) : std::runtime_error(message), code(value) {}
};
static std::string quote(const std::string& text) {
    std::ostringstream out; out << '"';
    for (char raw : text) {
        const auto ch=static_cast<unsigned char>(raw);
        if (ch=='"' || ch=='\\') out << '\\' << ch;
        else if (ch<32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(ch) << std::dec;
        else out << ch;
    }
    out << '"'; return out.str();
}
static void number(std::ostream& out, double value) { if (std::isfinite(value)) out << value; else out << "null"; }
static int copyText(const std::string& text, char* out, int capacity) {
    if (text.size()>=static_cast<size_t>(INT_MAX)) return -1;
    const int required=static_cast<int>(text.size())+1;
    if (out && capacity>=required) std::memcpy(out,text.c_str(),static_cast<size_t>(required));
    return required;
}
struct ControlState { int32_t connected, telemetryAgeMs, homingMask, mitFaultMask; };
struct SendTiming { int64_t sequence, startedNs, finishedNs; double intervalMs, durationMs; };

struct Context {
    // ABI calls serialize against each other; RX only owns stateMutex. The descriptor
    // stays open until RX is joined, including after unexpected link loss.
    std::mutex apiMutex, stateMutex;
    std::atomic<bool> stop{true};
    std::thread receiver;
    int fd{-1};
    bool tcp{}, serialExclusive{}, connected{}, transportLive{}, targetsSet{};
    std::string link, error, failedOperation, disconnectReason;
    int errorCode{}, disconnectCode{};
    uint64_t disconnectCount{}, rxOk{}, sendAttempts{}, sendSuccesses{}, sendFailures{};
    uint64_t intervalsBelow1{}, intervalsAbove4{};
    int64_t receivedAt{}, disconnectedAt{}, disconnectedUnix{}, lastSendUnix{};
    int64_t lastSendStart{}, lastSendEnd{}, previousRx{};
    double lastDuration{}, maxDuration{}, lastInterval{-1}, rxHz{}, txHz{};
    uint8_t commandSequence{};
    std::array<float,18> targetSelf{};
    std::array<float,2> targetMit{};
    a32wire::Firmware firmware;
    a32wire::Hand hand;
    a32wire::Board board;
    ~Context() { cleanup(); }

    void cleanup() noexcept {
        stop.store(true);
        if (receiver.joinable()) receiver.join();
        if (fd>=0) {
            if (serialExclusive) ::ioctl(fd,TIOCNXCL);
            ::close(fd); fd=-1;
        }
        serialExclusive=false;
        std::lock_guard<std::mutex> lock(stateMutex);
        connected=false; transportLive=false; receivedAt=0; targetsSet=false;
        hand={}; board={}; firmware={}; rxHz=txHz=0;
    }
    void lost(int code, const std::string& reason) {
        stop.store(true);
        std::lock_guard<std::mutex> lock(stateMutex);
        if (transportLive) {
            transportLive=false; connected=false;
            ++disconnectCount; disconnectCode=errorCode=code;
            disconnectedAt=nowMs(); disconnectedUnix=unixMs();
            disconnectReason=error=reason;
            for (auto& item: hand.self) item.online=false;
            for (auto& item: hand.mit) item.online=false;
        }
    }
    void receive() noexcept {
        try {
            a32wire::Bytes pending;
            pending.reserve(4096);
            while (!stop.load()) {
                pollfd wait{fd,POLLIN,0};
                int result=::poll(&wait,1,100);
                if (result<0) { if (errno==EINTR) continue; throw Failure(7,ioError("receive poll")); }
                if (!result) continue;
                // Drain bytes before honoring POLLHUP, since the last valid packet
                // and EOF may arrive in the same poll result.
                if (wait.revents&POLLIN) {
                    uint8_t chunk[2048];
                    const ssize_t count=::read(fd,chunk,sizeof(chunk));
                    if (count<0) {
                        if (errno==EINTR || errno==EAGAIN || errno==EWOULDBLOCK) continue;
                        throw Failure(7,ioError("receive"));
                    }
                    if (!count) throw Failure(2,"device closed the connection");
                    pending.insert(pending.end(),chunk,chunk+count);
                    uint8_t fun{}; a32wire::Bytes payload;
                    while (a32wire::pop(pending,fun,payload)) {
                        std::lock_guard<std::mutex> lock(stateMutex);
                        if (fun==1) {
                            a32wire::Firmware decoded;
                            if (a32wire::parse_firmware(payload,decoded)) firmware=decoded;
                        } else if (fun==2) {
                            a32wire::Hand decoded;
                            if (a32wire::parse_hand(payload,decoded)) {
                                hand=decoded; receivedAt=nowMs(); ++rxOk;
                                const auto stamp=nowNs();
                                if (previousRx && stamp>previousRx) rxHz=1e9/static_cast<double>(stamp-previousRx);
                                previousRx=stamp;
                            }
                        } else if (fun==3) {
                            a32wire::Board decoded;
                            if (a32wire::parse_board(payload,decoded)) board=decoded;
                        }
                    }
                    if (pending.size()>8192) pending.erase(pending.begin(),pending.end()-1);
                } else if (wait.revents&(POLLERR|POLLHUP|POLLNVAL)) {
                    throw Failure(2,"device disconnected during receive");
                }
            }
        } catch (const Failure& failure) { if (!stop.load()) lost(failure.code,failure.what()); }
          catch (const std::exception& failure) { if (!stop.load()) lost(7,failure.what()); }
          catch (...) { if (!stop.load()) lost(7,"unknown receive failure"); }
    }
    void writeAll(const a32wire::Bytes& bytes, bool connecting=false) {
        { std::lock_guard<std::mutex> lock(stateMutex);
          if (!transportLive || (!connecting && !connected)) throw Failure(1,"not connected"); }
        const auto deadline=Clock::now()+250ms;
        size_t offset=0;
        try {
            while (offset<bytes.size()) {
                if (stop.load()) throw Failure(2,"connection lost during send");
                const auto left=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-Clock::now()).count();
                if (left<=0) throw Failure(3,"send timed out (250 ms)");
                pollfd wait{fd,POLLOUT,0};
                int result=::poll(&wait,1,static_cast<int>(left));
                if (result<0) { if (errno==EINTR) continue; throw Failure(7,ioError("send poll")); }
                if (!result) continue;
                if (wait.revents&(POLLERR|POLLHUP|POLLNVAL)) throw Failure(2,"device disconnected during send");
                if (!(wait.revents&POLLOUT)) continue;
                const auto count=tcp ? ::send(fd,bytes.data()+offset,bytes.size()-offset,MSG_NOSIGNAL)
                                     : ::write(fd,bytes.data()+offset,bytes.size()-offset);
                if (count<0) {
                    if (errno==EINTR || errno==EAGAIN || errno==EWOULDBLOCK) continue;
                    throw Failure(7,ioError("send"));
                }
                if (!count) throw Failure(2,"device closed during send");
                offset+=static_cast<size_t>(count);
            }
        } catch (const Failure& failure) { lost(failure.code,failure.what()); throw; }
    }
    void startConnection(const std::string& name, bool isTcp) {
        { std::lock_guard<std::mutex> lock(stateMutex);
          link=name; tcp=isTcp; transportLive=true; connected=false; stop.store(false);
          firmware={}; hand={}; board={}; receivedAt=0; previousRx=0; rxOk=0;
          targetsSet=false; error.clear(); errorCode=0; failedOperation.clear(); }
        try {
            receiver=std::thread([this] { receive(); });
            // Version information is optional metadata. Some deployed firmware
            // does not answer this query; neither its absence nor its version
            // gates transport connection. Telemetry and commands retain their
            // existing frame validation and connection-loss handling.
            writeAll(a32wire::host_info(),true);
            std::lock_guard<std::mutex> lock(stateMutex);
            if (!transportLive) throw Failure(errorCode,error.empty()?"connection lost during connect":error);
            connected=true;
        } catch (...) { cleanup(); throw; }
    }
    void setTargets(const float* self, const float* mit) {
        if (!self || !mit) throw Failure(6,"targets must not be null");
        // Validate the entire transaction before changing any cached target.
        for (int i=0;i<18;++i) if (!std::isfinite(self[i])) throw Failure(6,"self targets must be finite");
        for (int i=0;i<2;++i) if (!std::isfinite(mit[i])) throw Failure(6,"MIT targets must be finite");
        std::lock_guard<std::mutex> lock(stateMutex);
        for (int i=0;i<18;++i) targetSelf[static_cast<size_t>(i)]=a3::clampSlotPos(i,self[i]);
        for (int i=0;i<2;++i) targetMit[static_cast<size_t>(i)]=a3::clampMitPos(mit[i]);
        targetsSet=true;
    }
    void stopSending() {
        // Local cancellation only: keep telemetry connected and require a fresh
        // explicit target before another position send. Never change motor power.
        std::lock_guard<std::mutex> lock(stateMutex);
        targetsSet=false;
        targetSelf={};
        targetMit={};
    }
    void sendOnce(SendTiming* timing) {
        a32wire::Bytes bytes;
        { std::lock_guard<std::mutex> lock(stateMutex);
          if (!targetsSet) throw Failure(6,"set explicit targets before sending; connect does not initialize motion");
          bytes=a32wire::targets(commandSequence++,targetSelf,targetMit); }
        const auto started=nowNs();
        { std::lock_guard<std::mutex> lock(stateMutex);
          ++sendAttempts; lastSendUnix=unixMs();
          lastInterval=lastSendStart?static_cast<double>(started-lastSendStart)/1e6:-1;
          if (lastInterval>=0 && lastInterval<1) ++intervalsBelow1;
          if (lastInterval>4) ++intervalsAbove4;
          if (lastInterval>0) txHz=1000/lastInterval;
          lastSendStart=started; }
        auto finish=[this,timing,started](bool success) {
            std::lock_guard<std::mutex> lock(stateMutex);
            if (success) ++sendSuccesses; else ++sendFailures;
            lastSendEnd=nowNs(); lastDuration=static_cast<double>(lastSendEnd-started)/1e6;
            maxDuration=std::max(maxDuration,lastDuration);
            if (timing) *timing={static_cast<int64_t>(sendAttempts),started,lastSendEnd,lastInterval,lastDuration};
        };
        try { writeAll(bytes); finish(true); } catch (...) { finish(false); throw; }
    }
};

template<class F> static int invoke(Context* c, const char* operation, F action) noexcept {
    if (!c) return 0;
    try {
        std::lock_guard<std::mutex> api(c->apiMutex);
        action(); return 1;
    } catch (const Failure& failure) {
        std::lock_guard<std::mutex> lock(c->stateMutex);
        c->failedOperation=operation; c->error=failure.what(); c->errorCode=failure.code;
    } catch (const std::exception& failure) {
        std::lock_guard<std::mutex> lock(c->stateMutex);
        c->failedOperation=operation; c->error=failure.what(); c->errorCode=-1;
    } catch (...) {
        std::lock_guard<std::mutex> lock(c->stateMutex);
        c->failedOperation=operation; c->error="unknown native driver exception"; c->errorCode=-1;
    }
    return 0;
}
static std::vector<std::string> autoPorts() {
    std::vector<std::string> found;
    std::error_code ec;
    for (const auto& item: std::filesystem::directory_iterator("/sys/class/tty",ec)) {
        const auto name=item.path().filename().string();
        if (name.rfind("ttyACM",0)!=0) continue;
        auto path=std::filesystem::canonical(item.path()/"device",ec);
        if (ec) { ec.clear(); continue; }
        while (path!="/" && !path.empty()) {
            std::string vid,pid;
            std::ifstream(path/"idVendor")>>vid;
            std::ifstream(path/"idProduct")>>pid;
            if (vid=="0483" && pid=="5740") { found.push_back("/dev/"+name); break; }
            path=path.parent_path();
        }
    }
    std::sort(found.begin(),found.end()); return found;
}
static void connectSerial(Context* c, const std::string& path) {
    c->cleanup();
    c->fd=::open(path.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
    if (c->fd<0) throw Failure(7,ioError("open serial port"));
    try {
        if (::ioctl(c->fd,TIOCEXCL)<0) throw Failure(7,ioError("claim serial port exclusively"));
        c->serialExclusive=true;
        termios options{};
        if (::tcgetattr(c->fd,&options)<0) throw Failure(7,ioError("read serial settings"));
        ::cfmakeraw(&options);
        options.c_cflag|=CLOCAL|CREAD;
        options.c_cflag&=static_cast<tcflag_t>(~(CSTOPB|PARENB|CRTSCTS|HUPCL));
        options.c_cc[VMIN]=0; options.c_cc[VTIME]=0;
        if (::cfsetispeed(&options,B115200)<0 || ::cfsetospeed(&options,B115200)<0 ||
            ::tcsetattr(c->fd,TCSANOW,&options)<0) throw Failure(7,ioError("configure serial port"));
        // Do not pulse DTR/RTS or send enable/position commands during connect.
        c->startConnection("USB:"+path,false);
    } catch (...) { c->cleanup(); throw; }
}
static void connectTcp(Context* c, const char* host, unsigned short port) {
    if (!host || !*host || !port) throw Failure(6,"TCP host and nonzero port required");
    c->cleanup();
    addrinfo hints{}; hints.ai_family=AF_UNSPEC; hints.ai_socktype=SOCK_STREAM;
    addrinfo* addresses=nullptr;
    const std::string service=std::to_string(port);
    const int lookup=::getaddrinfo(host,service.c_str(),&hints,&addresses);
    if (lookup) throw Failure(7,std::string("resolve TCP host: ")+::gai_strerror(lookup));
    const auto deadline=Clock::now()+1500ms;
    for (auto* item=addresses;item;item=item->ai_next) {
        const auto remaining=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-Clock::now()).count();
        if (remaining<=0) break;
        const int candidate=::socket(item->ai_family,item->ai_socktype|SOCK_NONBLOCK|SOCK_CLOEXEC,item->ai_protocol);
        if (candidate<0) continue;
        bool ok=::connect(candidate,item->ai_addr,item->ai_addrlen)==0;
        if (!ok && errno==EINPROGRESS) {
            pollfd wait{candidate,POLLOUT,0};
            int result;
            do { result=::poll(&wait,1,static_cast<int>(std::max<int64_t>(0,std::chrono::duration_cast<std::chrono::milliseconds>(deadline-Clock::now()).count()))); }
            while (result<0 && errno==EINTR && Clock::now()<deadline);
            int socketError{}; socklen_t len=sizeof(socketError);
            ok=result>0 && ::getsockopt(candidate,SOL_SOCKET,SO_ERROR,&socketError,&len)==0 && socketError==0;
        }
        if (ok) { c->fd=candidate; break; }
        ::close(candidate);
    }
    ::freeaddrinfo(addresses);
    if (c->fd<0) throw Failure(7,"TCP connect failed or timed out (1500 ms)");
    const int yes=1;
    ::setsockopt(c->fd,IPPROTO_TCP,TCP_NODELAY,&yes,sizeof(yes));
    c->startConnection(std::string("TCP:")+host+":"+service,true);
}

API int a32_abi_version() noexcept { return 2; }
// Stateless conversion for offline consumers and parity checks; no transport
// or motor state is read or changed. Invalid input leaves the output unchanged.
API int a32_angles_to_actuators(const float* angles_deg,float* actuators) noexcept {
    return a3::anglesDegToActuator(angles_deg,actuators)?1:0;
}
API Context* a32_create() noexcept { try { return new Context(); } catch (...) { return nullptr; } }
API void a32_destroy(Context* c) noexcept { try { delete c; } catch (...) {} }
API int a32_last_error(Context* c, char* out, int capacity) noexcept {
    try { if (!c) return copyText("Invalid native handle",out,capacity);
        std::lock_guard<std::mutex> lock(c->stateMutex); return copyText(c->error,out,capacity);
    } catch (...) { return -1; }
}
API int a32_connect_usb(Context* c, const char* port) noexcept {
    return invoke(c,"connect_usb",[&] {
        if (port && *port && std::string(port)!="auto") { connectSerial(c,port); return; }
        const auto found=autoPorts();
        if (found.empty()) throw Failure(8,"no H7 USB CDC port (0483:5740); connect the device and check Linux USB permissions");
        if (found.size()!=1) throw Failure(6,"multiple H7 USB CDC devices; specify an explicit /dev/ttyACM path");
        connectSerial(c,found.front());
    });
}
API int a32_connect_eth(Context* c,const char* host,unsigned short port) noexcept { return invoke(c,"connect_eth",[&] { connectTcp(c,host,port); }); }
API int a32_disconnect(Context* c) noexcept { return invoke(c,"disconnect",[&] { c->cleanup(); }); }
API int a32_set_targets(Context* c,const float* self,const float* mit) noexcept { return invoke(c,"set_targets",[&] { c->setTargets(self,mit); }); }
API int a32_send_once(Context* c) noexcept { return invoke(c,"send_once",[&] { c->sendOnce(nullptr); }); }
API int a32_send_targets(Context* c,const float* self,const float* mit,SendTiming* timing) noexcept {
    return invoke(c,"send_targets",[&] {
        if (!timing) throw Failure(6,"timing output required");
        c->setTargets(self,mit);
        int64_t earliest;
        { std::lock_guard<std::mutex> lock(c->stateMutex); earliest=c->lastSendStart+1000000; }
        // Preserve the additive ABI's minimum 1 ms spacing at the actual send boundary.
        if (nowNs()<earliest) std::this_thread::sleep_until(Clock::time_point(std::chrono::nanoseconds(earliest)));
        c->sendOnce(timing);
    });
}
API int a32_enable_all(Context* c,int enabled) noexcept {
    return invoke(c,"enable_all",[&] {
        if (enabled) c->writeAll(a32wire::enable());
        else c->stopSending();
    });
}
API int a32_home_slot(Context* c,int slot,float iq) noexcept {
    return invoke(c,"home_slot",[&] {
        if (slot<0 || slot>=18) throw Failure(5,"invalid self motor slot (0..17)");
        if (!std::isfinite(iq) || iq<=0) throw Failure(6,"homing current must be positive finite motor counts");
        c->writeAll(a32wire::home(slot,false,iq));
    });
}
API int a32_home_reset_slot(Context* c,int slot) noexcept {
    return invoke(c,"home_reset_slot",[&] {
        if (slot<0 || slot>=18) throw Failure(5,"invalid self motor slot (0..17)");
        c->writeAll(a32wire::home(slot,true,0));
    });
}
API int a32_stop_sending(Context* c) noexcept { return invoke(c,"stop_sending",[&] { c->stopSending(); }); }
// Preserve the existing C ABI entry point, now with local cancellation semantics.
API int a32_estop(Context* c) noexcept { return invoke(c,"estop",[&] { c->stopSending(); }); }
API int a32_control_state(Context* c,ControlState* out) noexcept {
    return invoke(c,"control_state",[&] {
        if (!out) throw Failure(6,"control state output required");
        std::lock_guard<std::mutex> lock(c->stateMutex);
        const auto age=c->receivedAt?nowMs()-c->receivedAt:-1;
        *out={c->connected?1:0,static_cast<int32_t>(std::min<int64_t>(age,INT32_MAX)),0,0};
        for (size_t i=0;i<18;++i) if (c->hand.self[i].online && (c->hand.self[i].homing==1 || c->hand.self[i].homing==4)) out->homingMask|=1<<i;
        for (size_t i=0;i<2;++i) if (c->hand.mit[i].online && (c->hand.mit[i].fault || c->hand.mit[i].state==5)) out->mitFaultMask|=1<<i;
    });
}
API int a32_state(Context* c,char* out,int capacity) noexcept {
    if (!c) return -1;
    try {
        std::lock_guard<std::mutex> lock(c->stateMutex);
        std::ostringstream s; s << std::setprecision(9) << std::boolalpha;
        s << "{\"connected\":" << c->connected << ",\"streaming\":false,\"sdk_version\":" << quote(a3::sdkVersion())
          << ",\"native_implementation\":\"a3-sdk-protocol-linux-abi2/1.0\",\"link\":" << quote(c->link)
          << ",\"telemetry_format\":\"a32-native-sdk-state/v2\",\"telemetry_units\":{\"self_current\":\"A\",\"external_current\":\"A\"}"
          << ",\"rx_mask\":" << c->hand.mask << ",\"rx_ok\":" << c->rxOk << ",\"rx_hz\":"; number(s,c->rxHz);
        s << ",\"tx_hz\":"; number(s,c->txHz);
        s << ",\"telemetry_age_ms\":" << (c->receivedAt?nowMs()-c->receivedAt:-1) << ",\"sequence\":" << c->hand.sequence << ",\"tick_ms\":" << c->hand.tick;
        const auto& f=c->firmware;
        s << ",\"firmware\":{\"valid\":" << f.valid << ",\"proto_major\":" << int(f.proto_major) << ",\"proto_minor\":" << int(f.proto_minor)
          << ",\"app_major\":" << int(f.app_major) << ",\"app_minor\":" << int(f.app_minor) << ",\"app_patch\":" << int(f.app_patch) << ",\"app_build\":" << int(f.app_build)
          << ",\"hand_side\":" << int(f.hand_side) << "},\"self\":[";
        for (size_t i=0;i<18;++i) {
            if (i) s << ',';
            const auto& row=c->hand.self[i];
            s << "{\"slot\":" << i << ",\"online\":" << row.online << ",\"homing\":" << int(row.homing) << ",\"position\":"; number(s,row.position);
            s << ",\"target\":"; number(s,row.target); s << ",\"iq\":"; number(s,row.iq); s << ",\"iq_ref\":"; number(s,row.iq_ref);
            s << ",\"current_unit\":\"A\"";
            s << ",\"temperature_centi\":" << row.temperature << ",\"open_spin\":" << row.open_spin << '}';
        }
        s << "],\"mit\":[";
        for (size_t i=0;i<2;++i) {
            if (i) s << ',';
            const auto& row=c->hand.mit[i];
            s << "{\"index\":" << i << ",\"online\":" << row.online << ",\"valid\":" << row.valid << ",\"position\":"; number(s,row.position);
            s << ",\"velocity_rpm\":"; number(s,row.velocity); s << ",\"iq\":"; number(s,row.iq); s << ",\"voltage\":"; number(s,row.voltage);
            s << ",\"current_unit\":\"A\"";
            s << ",\"fault\":" << row.fault << ",\"state\":" << int(row.state) << ",\"calib\":" << int(row.calib) << ",\"mode\":" << int(row.mode) << '}';
        }
        s << "],\"board\":{\"valid\":" << c->board.valid << ",\"ntc_c_centi\":" << c->board.ntc_c << ",\"ntc_b_centi\":" << c->board.ntc_b << ",\"vin24_mv\":" << c->board.vin << "},\"targets\":{\"self\":[";
        for (size_t i=0;i<18;++i) { if (i) s << ','; number(s,c->targetSelf[i]); }
        s << "],\"mit_pos\":[";
        for (size_t i=0;i<2;++i) { if (i) s << ','; number(s,c->targetMit[i]); }
        s << "]},\"last_error\":" << quote(c->error) << ",\"last_error_code\":" << c->errorCode << ",\"diagnostics\":{\"last_failed_operation\":" << quote(c->failedOperation)
          << ",\"disconnect_count\":" << c->disconnectCount << ",\"last_disconnect_error_code\":" << c->disconnectCode
          << ",\"last_disconnect_at_unix_ms\":" << c->disconnectedUnix << ",\"last_disconnect_age_ms\":" << (c->disconnectedAt?nowMs()-c->disconnectedAt:-1)
          << ",\"last_disconnect_reason\":" << quote(c->disconnectReason) << ",\"send_attempts\":" << c->sendAttempts
          << ",\"send_successes\":" << c->sendSuccesses << ",\"send_failures\":" << c->sendFailures << ",\"last_send_at_unix_ms\":" << c->lastSendUnix
          << ",\"last_send_duration_ms\":"; number(s,c->lastDuration); s << ",\"max_send_duration_ms\":"; number(s,c->maxDuration);
        s << ",\"last_send_interval_ms\":"; number(s,c->lastInterval);
        s << ",\"send_intervals_below_1ms\":" << c->intervalsBelow1 << ",\"send_intervals_above_4ms\":" << c->intervalsAbove4 << "}}";
        return copyText(s.str(),out,capacity);
    } catch (...) { return -1; }
}
