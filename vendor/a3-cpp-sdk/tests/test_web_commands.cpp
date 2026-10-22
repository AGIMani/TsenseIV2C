#define main bridge_main
#include "../examples/web_bridge.cpp"
#undef main
#include <memory>
#include <atomic>

class CaptureTransport : public ITransport {
public:
    std::atomic<bool> live{false};
    int sends = 0;
    bool open() override { live = true; return true; }
    void close() override { live = false; }
    bool send(const uint8_t *, size_t) override { ++sends; return live.load(); }
    size_t recv(uint8_t *, size_t, int) override { return 0; }
    bool isOpen() const override { return live.load(); }
    std::string name() const override { return "web-test"; }
};
int main() {
    int fails = 0;
    auto rejected = [&fails](const std::string &command) {
        if (handle_cmd(command).find("\"ok\":false") == std::string::npos) {
            std::cerr << "unexpected acceptance: " << command << "\n"; ++fails;
        }
    };
    rejected(R"({"type":"stream","on":1,"hz":500})");
    auto tr = std::make_unique<CaptureTransport>(); auto *raw = tr.get();
    g_hand.attach(std::move(tr));
    rejected(R"({"type":"pid_pos"})");
    rejected(R"({"type":"enable"})");
    rejected(R"({"type":"enable","slot":99,"scope":"bus","on":1,"drv":3})");
    rejected(R"({"type":"pid_pos","slot":0,"kp":NaN,"ki":0,"kd":0})");
    rejected(R"({"type":"pid_pos","slot":0,"kp":1e999,"ki":0,"kd":0})");
    rejected(R"({"type":"zero","type":"enable"})");
    rejected(R"({"type":"zero"} trailing)");
    rejected(R"({"type":"zero",})");
    rejected(R"({"type":"zero","unused":1})");
    rejected(R"({"type":"touch","src":0,"cmd":2,"data":[1,]})");
    if (raw->sends != 0) ++fails;
    if (handle_cmd(R"({"type":"pid_pos","slot":0,"kp":1,"ki":0,"kd":0})").find("\"ok\":true") == std::string::npos || raw->sends != 1) ++fails;
    g_hand.disconnect();
    return fails ? 1 : 0;
}
