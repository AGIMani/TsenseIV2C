// Offline lifecycle regression: all initialization/connection calls are wrapped.
// No SDK initialization, host discovery, USB access, or calibration is performed.
#include <array>
#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <vector>
#define private public
#include "ManusGlove.hpp"
#undef private
#include <cassert>
#include <cstring>
#include <stdexcept>
#include <thread>

namespace {
constexpr auto ok = SDKReturnCode::SDKReturnCode_Success;
constexpr auto failed = SDKReturnCode::SDKReturnCode_SdkNotAvailable;
std::atomic<int> initialized{0}, shutdowns{0}, searches{0}, connections{0};
std::atomic<bool> failInitialize{false}, failRegistration{false};
std::atomic<bool> holdConnection{false}, enteredConnection{false};
bool worldCoordinates = false;
DisconnectedFromCoreCallback_t onDisconnected = nullptr;

void reset() {
    initialized = shutdowns = searches = connections = 0;
    failInitialize = failRegistration = holdConnection = enteredConnection = false;
    worldCoordinates = false;
    onDisconnected = nullptr;
}

void ready(manus_glove::ManusGlove& glove) {
    glove.m_LandscapeReceived = true;
    glove.m_LeftGloveId = 42;
    glove.m_RawSkeletonFrameCount = 1;
    glove.m_ErgonomicsFrameCount = 1;
}
}

extern "C" SDKReturnCode __wrap_CoreSdk_InitializeIntegrated() {
    ++initialized;
    return failInitialize ? failed : ok;
}
extern "C" SDKReturnCode __wrap_CoreSdk_InitializeCore() {
    return __wrap_CoreSdk_InitializeIntegrated();
}
extern "C" SDKReturnCode __wrap_CoreSdk_ShutDown() { ++shutdowns; return ok; }
extern "C" SDKReturnCode __wrap_CoreSdk_RegisterCallbackForOnLog(LoggingCallback_t) {
    return failRegistration ? failed : ok;
}
extern "C" SDKReturnCode __wrap_CoreSdk_RegisterCallbackForSystemStream(SystemStreamCallback_t) { return ok; }
extern "C" SDKReturnCode __wrap_CoreSdk_RegisterCallbackForOnDisconnect(DisconnectedFromCoreCallback_t callback) {
    onDisconnected = callback;
    return ok;
}
extern "C" SDKReturnCode __wrap_CoreSdk_RegisterCallbackForRawSkeletonStream(RawSkeletonStreamCallback_t) { return ok; }
extern "C" SDKReturnCode __wrap_CoreSdk_RegisterCallbackForLandscapeStream(LandscapeStreamCallback_t) { return ok; }
extern "C" SDKReturnCode __wrap_CoreSdk_RegisterCallbackForErgonomicsStream(ErgonomicsStreamCallback_t) { return ok; }
extern "C" SDKReturnCode __wrap_CoreSdk_InitializeCoordinateSystemWithVUH(CoordinateSystemVUH coordinates, bool world) {
    assert(coordinates.handedness == Side::Side_Right);
    assert(coordinates.up == AxisPolarity::AxisPolarity_PositiveZ);
    assert(coordinates.view == AxisView::AxisView_XFromViewer);
    assert(coordinates.unitScale == 1.0f);
    worldCoordinates = world;
    return ok;
}
extern "C" SDKReturnCode __wrap_CoreSdk_ConnectToHost(ManusHost host) {
    ManusHost empty{};
    ManusHost_Init(&empty);
    assert(std::memcmp(&host, &empty, sizeof(host)) == 0);
    ++connections;
    enteredConnection = true;
    while (holdConnection.load()) std::this_thread::yield();
    return ok;
}
extern "C" SDKReturnCode __wrap_CoreSdk_LookForHosts(uint32_t, bool) { ++searches; return ok; }
extern "C" SDKReturnCode __wrap_CoreSdk_GetNumberOfAvailableHostsFound(uint32_t* count) { *count = 0; return ok; }
extern "C" SDKReturnCode __wrap_CoreSdk_GetAvailableHostsFound(ManusHost*, uint32_t) { assert(false); return failed; }
extern "C" SDKReturnCode __wrap_CoreSdk_SetRawSkeletonHandMotion(HandMotion) { return ok; }

int main() {
    using manus_glove::ManusGlove;
    reset();
    {
        ManusGlove glove;
        bool rejected = false;
        try { ManusGlove second; } catch (const std::runtime_error&) { rejected = true; }
        assert(rejected && ManusGlove::s_Instance.load() == &glove);
        ready(glove);
        assert(glove.Connect("integrated", true, 1));
        assert(glove.IsConnected() && glove.IsWorldCoordinates() && worldCoordinates);
        assert(initialized == 1 && connections == 1 && searches == 0);
        glove.m_LeftSkeleton.valid = true;
        assert(onDisconnected != nullptr);
        onDisconnected(nullptr);
        assert(!glove.IsConnected() && !glove.GetRawSkeleton("left").valid);
        assert(glove.GetGloveId("left") == 0);
        assert(glove.GetDebugEvents().back().typeName == "disconnected");
        SkeletonStreamInfo lateFrame{};
        lateFrame.skeletonsCount = 1;
        ManusGlove::OnRawSkeletonStreamCallback(&lateFrame);
        assert(!glove.GetRawSkeleton("left").valid);
        glove.Disconnect();
        glove.Disconnect();
        assert(!glove.IsConnected() && shutdowns == 1);
        assert(glove.GetGloveId("left") == 0 && !glove.GetRawSkeleton("left").valid);
    }
    assert(ManusGlove::s_Instance.load() == nullptr);
    reset();
    {
        ManusGlove glove;
        glove.RequestDisconnect();
        assert(!glove.Connect("integrated", true, 1));
        assert(initialized == 0 && shutdowns == 0);
    }
    for (bool failInit : {true, false}) {
        reset();
        ManusGlove glove;
        failInitialize = failInit;
        failRegistration = !failInit;
        assert(!glove.Connect("integrated", true, 1));
        assert(!glove.IsConnected() && initialized == 1 && shutdowns == 1);
        glove.Disconnect();
        assert(shutdowns == 1);
    }
    reset();
    {
        ManusGlove glove;
        std::thread connector([&] { assert(!glove.Connect("remote", true, 10)); });
        while (searches.load() == 0) std::this_thread::yield();
        auto start = std::chrono::steady_clock::now();
        glove.RequestDisconnect();
        connector.join();
        assert(std::chrono::steady_clock::now() - start < std::chrono::seconds(1));
        assert(shutdowns == 1 && !glove.IsConnected());
    }
    reset();
    {
        ManusGlove glove;
        holdConnection = true;
        std::thread connector([&] { assert(!glove.Connect("integrated", true, 1)); });
        while (!enteredConnection.load()) std::this_thread::yield();
        glove.RequestDisconnect(); // this must not wait for the blocked SDK call
        std::thread disconnector([&] { glove.Disconnect(); });
        // Disconnect is serialized behind Connect; SDK shutdown must not overlap.
        assert(shutdowns == 0);
        holdConnection = false;
        connector.join();
        disconnector.join();
        assert(shutdowns == 1 && !glove.IsConnected());
    }
}
