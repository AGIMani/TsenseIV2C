#ifndef _MANUS_GLOVE_HPP_
#define _MANUS_GLOVE_HPP_

// Thin, callback-free-on-the-Python-side wrapper around the Manus Core SDK.
// All SDK callback complexity (threads, pull-on-callback) is handled in C++.
// Python only ever pulls the latest cached frame.
//
// Coordinate mode (global vs local) follows "plan A": it is fixed at connect()
// time via CoreSdk_InitializeCoordinateSystemWithVUH(..., p_UseWorldCoordinates).
// To switch or reconnect after cancellation, disconnect and create a fresh instance.

#include "ManusSDK.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace manus_glove
{

/// @brief A snapshot of one hand's raw skeleton, flattened row-major to [rows x cols].
/// cols is always 10: position(3) + rotation quaternion(4, w,x,y,z) + scale(3).
struct SkeletonSnapshot
{
    std::vector<float> data; ///< rows*cols floats, row-major
    std::vector<uint32_t> nodeIds; ///< SkeletonNode.id for each data row; never infer from row index
    int rows = 0;            ///< node count (typically 26)
    int cols = 10;
    uint32_t gloveId = 0;
    bool valid = false; ///< false if no data for this hand yet
    uint64_t sequence = 0; ///< per-side SDK callback publication, never a Python read
    uint64_t hostArrivalNs = 0; ///< Linux CLOCK_MONOTONIC at callback entry
    uint64_t sourceTimestampRaw = 0; ///< opaque ManusTimestamp, NOT nanoseconds
};

struct SkeletonEvent
{
    std::string side;
    SkeletonSnapshot snapshot;
};

/// @brief Static per-node hierarchy info (fetched once after connect).
/// Enum values are passed through as ints (see ManusSDKTypes.h: ChainType, Side, FingerJointType).
struct NodeInfoOut
{
    uint32_t nodeId = 0;
    uint32_t parentId = 0;
    int chainType = 0;       ///< ChainType (FingerThumb/Index/.../Hand ...)
    int side = 0;            ///< Side (1=Left, 2=Right)
    int fingerJointType = 0; ///< FingerJointType (Metacarpal/Proximal/.../Tip)
};

struct GloveStartupStatus
{
    uint32_t id = 0;
    uint32_t dongleId = 0;
    uint32_t batteryPercentage = 0;
    int32_t transmissionStrength = 0;
    int pairedState = 0;
    bool excluded = false;
    bool valid = false;
};

struct DebugEvent
{
    uint64_t sequence = 0;
    uint64_t timestampMs = 0;
    std::string source;
    int type = 0;
    std::string typeName;
    int severity = -1;
    std::string severityName;
    std::string message;
    uint32_t infoUInt = 0;
};

class ManusGlove
{
public:
    ManusGlove();
    ~ManusGlove();

    // Non-copyable: a single static instance backs the C callbacks.
    ManusGlove(const ManusGlove&) = delete;
    ManusGlove& operator=(const ManusGlove&) = delete;

    /// @brief Initialize the SDK, register callbacks, set coordinate system, connect.
    /// @param p_Mode "integrated" | "local" | "remote"
    /// @param p_WorldCoordinates true = global/world coords, false = local (relative to parent).
    /// @param p_TimeoutSeconds total seconds to keep retrying the host connection.
    /// @return true on success.
    bool Connect(const std::string& p_Mode, bool p_WorldCoordinates, int p_TimeoutSeconds = 10);

    /// @brief Shut the SDK down. Safe to call multiple times; cancellation is sticky.
    void Disconnect();
    /// Non-blocking cooperative cancellation; vendor initialization/shutdown may still block.
    void RequestDisconnect() { m_StopRequested.store(true); }

    bool IsConnected() const { return m_Connected; }

    /// @brief Whether the current connection delivers world (global) coordinates.
    bool IsWorldCoordinates() const { return m_WorldCoordinates; }

    /// @brief Latest raw skeleton for one hand. p_Side: "left" or "right".
    SkeletonSnapshot GetRawSkeleton(const std::string& p_Side);
    std::vector<SkeletonEvent> DrainSkeletonEvents();
    void SetRecordingEnabled(bool enabled);
    uint64_t SkeletonEventsDropped() const { return m_SkeletonEventsDropped.load(); }

    /// @brief Latest merged ergonomics for both hands: 40 floats.
    /// Layout matches ErgonomicsDataType: [0..19] left, [20..39] right.
    std::array<float, ErgonomicsDataType_MAX_SIZE> GetErgonomics();

    /// @brief Static node hierarchy for one hand. Empty until skeleton data has arrived.
    std::vector<NodeInfoOut> GetNodeInfo(const std::string& p_Side);

    /// @brief Resolved glove id for a side, or 0 if not present yet.
    uint32_t GetGloveId(const std::string& p_Side);

    /// @brief Vibrate selected finger motors of the glove for a side.
    /// Powers are ordered Thumb, Index, Middle, Ring, Pinky and should be 0..1.
    /// A zero power vector stops any active vibration.
    int VibrateFingers(const std::string& p_Side, const std::array<float, 5>& p_Powers);

    /// @brief Write a .mcal calibration (bytes) to the glove of the given side.
    /// @return SetGloveCalibrationReturnCode (>=0). Returns -1 if not connected /
    ///         no glove for that side, -2 on SDK transport error.
    int SetCalibration(const std::string& p_Side, const std::vector<uint8_t>& p_Bytes);

    /// @brief Recent SDK log and system-message events. Clears the ring buffer by default.
    std::vector<DebugEvent> GetDebugEvents(bool p_Clear = true);

private:
    static std::atomic<ManusGlove*> s_Instance;

    // ---- SDK callbacks (run on SDK threads) ----
    static void OnLogCallback(LogSeverity p_Severity, const char* const p_Log, uint32_t p_Length);
    static void OnDisconnectedCallback(const ManusHost* const p_Host);
    static void OnSystemStreamCallback(const SystemMessage* const p_SystemMessage);
    static void OnRawSkeletonStreamCallback(const SkeletonStreamInfo* const p_Info);
    static void OnLandscapeStreamCallback(const Landscape* const p_Landscape);
    static void OnErgonomicsStreamCallback(const ErgonomicsStream* const p_Ergo);

    bool InitializeSdk(const std::string& p_Mode, bool p_WorldCoordinates);
    bool ConnectToFirstHost(bool p_LoopbackOnly, int p_TimeoutSeconds);
    void WaitForStartupDiagnostics(int p_TimeoutSeconds);
    void PrintStartupDiagnostics();
    void ClearCachedData();
    void PushDebugEvent(DebugEvent p_Event);
    uint64_t ElapsedMs() const;
    uint32_t GloveIdForSide(int p_Side); // p_Side: Side_Left / Side_Right

    static constexpr size_t kMaxDebugEvents = 300;

    std::mutex m_LifecycleMutex;
    std::atomic<bool> m_Connected{false};
    std::atomic<bool> m_WorldCoordinates{false};
    std::atomic<bool> m_StopRequested{false};
    bool m_Initialized = false; // guarded by m_LifecycleMutex
    bool m_Integrated = false;
    std::chrono::steady_clock::time_point m_StartTime;

    std::mutex m_DebugMutex;
    std::deque<DebugEvent> m_DebugEvents;
    uint64_t m_DebugSequence = 0;

    // Glove id <-> side, learned from the landscape stream.
    std::mutex m_LandscapeMutex;
    uint32_t m_LeftGloveId = 0;
    uint32_t m_RightGloveId = 0;
    uint32_t m_LandscapeGloveCount = 0;
    uint32_t m_LandscapeDongleCount = 0;
    bool m_LandscapeReceived = false;
    GloveStartupStatus m_LeftStartupStatus;
    GloveStartupStatus m_RightStartupStatus;

    // Raw skeleton, double-buffered.
    std::mutex m_SkeletonMutex;
    SkeletonSnapshot m_LeftSkeleton;
    SkeletonSnapshot m_RightSkeleton;
    uint64_t m_RawSkeletonFrameCount = 0;
    static constexpr size_t kMaxSkeletonEvents = 256;
    std::mutex m_SkeletonEventsMutex;
    std::deque<SkeletonEvent> m_SkeletonEvents;
    std::atomic<uint64_t> m_SkeletonEventsDropped{0};
    std::atomic<bool> m_RecordingEnabled{true};

    // Ergonomics, per-side 40-arrays (each glove fills its own half).
    std::mutex m_ErgoMutex;
    std::array<float, ErgonomicsDataType_MAX_SIZE> m_LeftErgo{};
    std::array<float, ErgonomicsDataType_MAX_SIZE> m_RightErgo{};
    bool m_LeftErgoValid = false;
    bool m_RightErgoValid = false;
    uint64_t m_ErgonomicsFrameCount = 0;
};

} // namespace manus_glove

#endif // _MANUS_GLOVE_HPP_
