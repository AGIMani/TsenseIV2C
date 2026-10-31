# A3.2 kinematics and SDK 1.1.15 commands

`A32StrokeMapper` now implements the SDK 1.1.15 calibrated closed-loop conversion for
both A3.2 models and both hands. Its input remains 20 named URDF angles in
radians, with 14 named millimetre outputs and 6 degree outputs. The pure Python
`sdk_stroke.py` uses the upstream `Stroke.cpp` coefficients recorded in
`sdk_stroke_data.py`; tests compare it with the compiled upstream function.
Feedback uses the inverse of the same monotonic curves. Actuators of the same
size share one curve: large MCP 19.63 mm, medium PIP/thumb MCP 16.95 mm, small
DIP/thumb PIP/pinky PIP 12.44 mm. Feedback includes exact 0/90 degree endpoints
and the SDK's float32 boundary tolerances; command angles still use its guards.

Application flexion matches the active URDF at 0..1.57 rad, including index/ring MCP. Swing
remains signed and is converted directly to degrees. The old `lut`/`exact`
method arguments remain accepted metadata; both use the new formula, identified
by `conversion="sdk-1.1.15"`. The offline mapper never loads a native library.

The original `A32Converter`, `a32_fast.py`, reference solver, geometry constants
and source URDF retain their original degree-input/SI-output behavior. A3.0
still uses that original geometry; it is not migrated to the A3.2 SDK formula.
The original solver's source provenance is recorded in `source_snapshot.json`.

See [SDK integration](../../docs/A32_NATIVE_SDK.md) for stroke caps, hardware
mapping, feedback signs and build instructions.
