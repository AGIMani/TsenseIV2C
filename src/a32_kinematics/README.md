# A3.2 independent kinematics

Copied from the user's `D:/Qtworks/gravity` on 2026-09-18. The reference solver,
fast solver, source URDF and inferred mechanism configuration are bundled here.
The only change to the supplied fast solver is its package-relative import.
On 2026-10-05, the index/mid/ring/pinky DIP mechanism stroke upper bounds in
`model/geometry_inferred.mat` were reduced to 0.008 m (8 mm). Other mechanism
parameters and zero positions are unchanged. Effective conversion angle limits
are derived from these stroke bounds.
`source_snapshot.json` records the original and current bundled file hashes.

`A32Converter` retains the original degree-input / SI-output interface.
`A32StrokeMapper` adapts A3.2 left/right URDF dictionaries (radians) to 14 named
linear inputs (mm) and 6 named rotary inputs (degrees). It contains no V2
imports, USB slot definitions or hardware sign assumptions.

The project mapper defaults to `method="lut"` (interpolation error bound
<= 1e-9 m); `method="exact"` remains available. The supplied low-level
converter's defaults are preserved for compatibility.

See [integration and benchmarks](../../docs/angle_to_stroke.md).
