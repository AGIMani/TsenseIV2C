// Offline SDK callback test: --wrap replaces both SDK skeleton reads.
// Does not call Connect(), InitializeSdk() or any hardware-facing method.
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
#include <thread>
#include <time.h>

static uint64_t generation = 0;
extern "C" SDKReturnCode __wrap_CoreSdk_GetRawSkeletonInfo(uint32_t, RawSkeletonInfo* info) {
    info->gloveId = 42;
    info->nodesCount = 21;
    info->publishTime.time = generation;
    return SDKReturnCode::SDKReturnCode_Success;
}
extern "C" SDKReturnCode __wrap_CoreSdk_GetRawSkeletonData(uint32_t, SkeletonNode* nodes, uint32_t count) {
    for (uint32_t i = 0; i < count; ++i) {
        nodes[i] = SkeletonNode{};
        nodes[i].id = count - i + 100; // deliberately unrelated to array order
        nodes[i].transform.position.x = static_cast<float>(generation);
    }
    return SDKReturnCode::SDKReturnCode_Success;
}
int main() {
    using manus_glove::ManusGlove;
    ManusGlove glove;
    glove.m_RightGloveId = 42;
    SkeletonStreamInfo info{};
    info.skeletonsCount = 1;
    assert(!glove.GetRawSkeleton("right").valid);
    timespec before{}, after{};
    clock_gettime(CLOCK_MONOTONIC, &before);
    ManusGlove::OnRawSkeletonStreamCallback(&info);
    clock_gettime(CLOCK_MONOTONIC, &after);
    auto first = glove.GetRawSkeleton("right");
    assert(first.valid && first.sequence == 1);
    assert(first.nodeIds.size() == 21 && first.nodeIds.front() == 121 && first.nodeIds.back() == 101);
    assert(first.hostArrivalNs >= static_cast<uint64_t>(before.tv_sec) * 1000000000ULL + before.tv_nsec);
    assert(first.hostArrivalNs <= static_cast<uint64_t>(after.tv_sec) * 1000000000ULL + after.tv_nsec);
    auto repeated = glove.GetRawSkeleton("right");
    assert(repeated.sequence == first.sequence && repeated.hostArrivalNs == first.hostArrivalNs);
    ManusGlove::OnRawSkeletonStreamCallback(&info); // static pose, genuinely new callback
    assert(glove.GetRawSkeleton("right").sequence == 2);
    assert(glove.DrainSkeletonEvents().size() == 2);
    assert(glove.SkeletonEventsDropped() == 0);

    // Publish and pull concurrently; each array must match its source metadata.
    std::thread producer([&] {
        for (generation = 1; generation < 1000; ++generation)
            ManusGlove::OnRawSkeletonStreamCallback(&info);
    });
    for (int i = 0; i < 1000; ++i) {
        auto snap = glove.GetRawSkeleton("right");
        assert(snap.data[0] == static_cast<float>(snap.sourceTimestampRaw));
    }
    producer.join();
    assert(glove.DrainSkeletonEvents().size() == 256);
    assert(glove.SkeletonEventsDropped() == 999 - 256);
    // Diagnostic contention does not block publication and cannot be hidden.
    std::atomic<bool> locked{false}, release{false};
    std::thread consumer([&] {
        std::lock_guard<std::mutex> held(glove.m_SkeletonEventsMutex);
        locked.store(true);
        while (!release.load()) std::this_thread::yield();
    });
    while (!locked.load()) std::this_thread::yield();
    auto dropped = glove.SkeletonEventsDropped();
    ManusGlove::OnRawSkeletonStreamCallback(&info);
    assert(glove.SkeletonEventsDropped() == dropped + 1);
    release.store(true);
    consumer.join();
    assert(glove.DrainSkeletonEvents().empty());
    glove.SetRecordingEnabled(false);
    dropped = glove.SkeletonEventsDropped();
    auto sequence = glove.GetRawSkeleton("right").sequence;
    ManusGlove::OnRawSkeletonStreamCallback(&info);
    assert(glove.GetRawSkeleton("right").sequence == sequence + 1);
    assert(glove.DrainSkeletonEvents().empty());
    assert(glove.SkeletonEventsDropped() == dropped);
    glove.SetRecordingEnabled(true);
    ManusGlove::OnRawSkeletonStreamCallback(&info);
    assert(glove.DrainSkeletonEvents().size() == 1);
}
