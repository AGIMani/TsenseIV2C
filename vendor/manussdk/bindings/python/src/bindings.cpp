#include "ManusGlove.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <cstring>
#include <algorithm>
#include <stdexcept>
#include <cmath>

namespace py = pybind11;
using namespace manus_glove;

namespace
{

// Convert a SkeletonSnapshot into a numpy array of shape (rows, 10).
// Returns an empty (0, 10) array when no data is available yet.
py::array_t<float> SnapshotToArray(const SkeletonSnapshot& p_Snap)
{
    const ssize_t t_Rows = p_Snap.valid ? p_Snap.rows : 0;
    const ssize_t t_Cols = 10;
    py::array_t<float> t_Arr({t_Rows, t_Cols});
    if (t_Rows > 0)
        std::memcpy(t_Arr.mutable_data(), p_Snap.data.data(),
                    static_cast<size_t>(t_Rows) * t_Cols * sizeof(float));
    return t_Arr;
}

py::dict SnapshotToDict(const SkeletonSnapshot& s)
{
    py::dict d;
    d["skeleton"] = SnapshotToArray(s);
    d["node_ids"] = s.nodeIds;
    d["sequence"] = s.sequence;
    d["host_arrival_ns"] = s.hostArrivalNs;
    d["host_clock_domain"] = "host_monotonic";
    d["source_timestamp_raw"] = s.sourceTimestampRaw;
    d["source_clock_domain"] = "manus_sdk_compressed_timestamp";
    d["source_timestamp_ns"] = py::none();
    d["tracking_valid"] = py::none(); // RawSkeletonInfo exposes no tracking quality flag.
    d["valid"] = s.valid;
    d["glove_id"] = s.gloveId;
    return d;
}

} // namespace

PYBIND11_MODULE(manus_glove, m)
{
    m.doc() = "pybind11 wrapper around the Manus Core SDK (raw skeleton + ergonomics + calibration)";

    py::class_<NodeInfoOut>(m, "NodeInfo")
        .def_readonly("node_id", &NodeInfoOut::nodeId)
        .def_readonly("parent_id", &NodeInfoOut::parentId)
        .def_readonly("chain_type", &NodeInfoOut::chainType)
        .def_readonly("side", &NodeInfoOut::side)
        .def_readonly("finger_joint_type", &NodeInfoOut::fingerJointType)
        .def("__repr__", [](const NodeInfoOut& n) {
            return "<NodeInfo id=" + std::to_string(n.nodeId) +
                   " parent=" + std::to_string(n.parentId) +
                   " chain=" + std::to_string(n.chainType) +
                   " side=" + std::to_string(n.side) +
                   " joint=" + std::to_string(n.fingerJointType) + ">";
        });

    py::class_<ManusGlove>(m, "ManusGlove")
        .def(py::init<>())
        .def("connect", &ManusGlove::Connect,
             py::arg("mode") = "integrated",
             py::arg("world_coordinates") = false,
             py::arg("timeout_seconds") = 10,
             py::call_guard<py::gil_scoped_release>(),
             "Initialize + connect. mode: 'integrated'|'local'|'remote'. "
             "world_coordinates: True=global, False=local (fixed for this connection).")
        .def("disconnect", &ManusGlove::Disconnect,
             py::call_guard<py::gil_scoped_release>())
        .def("request_disconnect", &ManusGlove::RequestDisconnect,
             "Request cooperative cancellation without waiting for the SDK thread.")
        .def("is_connected", &ManusGlove::IsConnected)
        .def("is_world_coordinates", &ManusGlove::IsWorldCoordinates)
        .def("get_glove_id", &ManusGlove::GetGloveId, py::arg("side"))
        .def("vibrate_fingers",
             [](ManusGlove& self, const std::string& side, const std::vector<float>& powers) {
                 if (powers.size() != 5)
                     throw std::invalid_argument("vibration powers must contain 5 values");
                 std::array<float, 5> values{};
                 std::copy(powers.begin(), powers.end(), values.begin());
                 return self.VibrateFingers(side, values);
             },
             py::arg("side"), py::arg("powers"),
             "Vibrate the five finger motors for one glove side. Powers are in [0, 1].")
        // Raw skeleton: per hand -> numpy (rows, 10) = pos(3)+quat wxyz(4)+scale(3)
        .def("get_raw_skeleton",
             [](ManusGlove& self, const std::string& side) {
                 return SnapshotToArray(self.GetRawSkeleton(side));
             },
             py::arg("side"),
             "Latest raw skeleton for one hand ('left'|'right') as (N,10) float array.")
        .def("get_raw_skeleton_snapshot", [](ManusGlove& self, const std::string& side) {
            return SnapshotToDict(self.GetRawSkeleton(side));
        }, py::arg("side"), "Atomic skeleton and callback provenance; reading does not advance sequence.")
        .def("get_recording_diagnostics", [](ManusGlove& self) {
            py::dict d;
            d["events_dropped"] = self.SkeletonEventsDropped();
            return d;
        })
        .def("set_recording_enabled", &ManusGlove::SetRecordingEnabled, py::arg("enabled"))
        .def("drain_recording_events", [](ManusGlove& self) {
            py::list out;
            for (const auto& event : self.DrainSkeletonEvents()) {
                py::dict d;
                d["stream"] = "manus." + event.side;
                d["event"] = "raw_skeleton";
                d["clock_domain"] = "host_monotonic";
                d["timestamp_ns"] = event.snapshot.hostArrivalNs;
                d["sequence"] = event.snapshot.sequence;
                auto payload = SnapshotToDict(event.snapshot);
                // Keep a failed source inspectable in strict JSON sidecars.
                py::list rows;
                size_t nonfinite = 0;
                for (int row = 0; row < event.snapshot.rows; ++row) {
                    py::list values;
                    for (int col = 0; col < event.snapshot.cols; ++col) {
                        const float value = event.snapshot.data[row * event.snapshot.cols + col];
                        if (std::isfinite(value)) values.append(value);
                        else { values.append(py::none()); ++nonfinite; }
                    }
                    rows.append(values);
                }
                payload["skeleton"] = rows;
                payload["nonfinite_values"] = nonfinite;
                d["payload"] = payload;
                out.append(d);
            }
            return out;
        })
        // Both hands at once -> dict {'left': (N,10), 'right': (N,10)}
        .def("get_raw_skeleton_both",
             [](ManusGlove& self) {
                 py::dict d;
                 d["left"] = SnapshotToArray(self.GetRawSkeleton("left"));
                 d["right"] = SnapshotToArray(self.GetRawSkeleton("right"));
                 return d;
             },
             "Latest raw skeleton for both hands as {'left':(N,10), 'right':(N,10)}.")
        // Ergonomics -> numpy (40,)
        .def("get_ergonomics",
             [](ManusGlove& self) {
                 auto ergo = self.GetErgonomics();
                 py::array_t<float> arr(static_cast<ssize_t>(ergo.size()));
                 std::memcpy(arr.mutable_data(), ergo.data(), ergo.size() * sizeof(float));
                 return arr;
             },
             "Merged ergonomics for both hands as a (40,) float array "
             "([0:20]=left, [20:40]=right).")
        .def("get_node_info", &ManusGlove::GetNodeInfo, py::arg("side"),
             "Static node hierarchy for one hand (list of NodeInfo).")
        .def("drain_debug_events",
             [](ManusGlove& self) {
                 py::list out;
                 for (const DebugEvent& e : self.GetDebugEvents(true))
                 {
                     py::dict d;
                     d["sequence"] = e.sequence;
                     d["timestamp_ms"] = e.timestampMs;
                     d["source"] = e.source;
                     d["type"] = e.type;
                     d["type_name"] = e.typeName;
                     d["severity"] = e.severity;
                     d["severity_name"] = e.severityName;
                     d["message"] = e.message;
                     d["info_uint"] = e.infoUInt;
                     out.append(std::move(d));
                 }
                 return out;
             },
             "Return and clear recent filtered SDK log/system diagnostic events.")
        // Calibration: Python passes raw .mcal bytes for the given side.
        .def("set_calibration",
             [](ManusGlove& self, const std::string& side, py::bytes data) {
                 std::string s = data; // copy bytes
                 std::vector<uint8_t> buf(s.begin(), s.end());
                 return self.SetCalibration(side, buf);
             },
             py::arg("side"), py::arg("mcal_bytes"),
             "Write .mcal calibration bytes to the glove of 'side'. Returns "
             "SetGloveCalibrationReturnCode (1=Success), or <0 on error.");
}
