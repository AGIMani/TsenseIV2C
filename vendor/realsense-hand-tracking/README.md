# Extracted depth geometry and fitting references

Source: https://github.com/realsenseai/hand_tracking_samples

Pinned commit: `75d5579aa258d97157d8a29b1c4da32f3d0a7608`.
Copyright 2017 Intel Corporation, Apache-2.0. Original LICENSE and COPYING are
preserved. `source.json` records the source files, line ranges and file hashes.

`depth-and-fit.reference.cpp` contains verbatim selected source fragments:

- `DCamera::deprojectz`: metric depth and camera intrinsics to XYZ.
- `PointCloud`: depth scale conversion before deprojection.
- `CloudConstraint(s)`: surface observations constrain a geometric model.
- `PhysModel::FitPointCloud`: combine cloud and connected-bone constraints.

The fragments document provenance and are not a standalone compilation target.
The executable adaptation is `online_control/online_motion_track/rgbd_fusion.py`.
Its pinhole deprojection directly adapts the original formula; live RealSense
capture instead supplies the SDK's calibrated, distortion-aware deprojection.
Its bounded 21-joint solver is newly implemented for this application's
MediaPipe skeleton and takes the original depth-plus-model fitting approach.
It does not reproduce the old rigid-body solver, CNN, or fixed right-hand model.
No original CNN weights or unrelated third-party rendering libraries are used.
