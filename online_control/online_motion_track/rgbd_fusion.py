"""Conservative RGB-D correction of MediaPipe's 21-joint hand skeleton.

``deproject_pixel`` directly adapts ``DCamera::deprojectz`` from Intel's
realsenseai/hand_tracking_samples, include/misc_image.h (Apache-2.0), commit
75d5579aa258d97157d8a29b1c4da32f3d0a7608. See vendor/realsense-hand-tracking
for the original excerpt and license. The remaining code is a NEW 21-joint
fitter inspired by include/physmodel.h::PhysModel::FitPointCloud: combine
depth observations with connected-bone constraints and a pose prior. It does
not port that project's CNN, rigid-body engine, or anatomical hand model.

Inputs must describe the same, unmirrored color frame. World landmarks are
MediaPipe camera-oriented metric coordinates (+x right, +y down, +z away).
Depth is already aligned to color, in meters. Output is wrist-relative,
in the same orientation, in meters. Skin-depth observations constrain a
small interval in front of the estimated bone center, not its exact Z.
Occluded/unobserved joints retain a pose prior and are explicitly reported.
"""

from __future__ import annotations

from typing import Callable, Mapping

import numpy as np
from scipy.optimize import least_squares


BONES = tuple(
    (a, b)
    for finger in ((0, 1, 2, 3, 4), (0, 5, 6, 7, 8), (0, 9, 10, 11, 12),
                   (0, 13, 14, 15, 16), (0, 17, 18, 19, 20))
    for a, b in zip(finger, finger[1:])
)
PALM = (0, 5, 9, 13, 17)
# Approximate visible-surface-to-center distances, not calibrated anatomy.
SURFACE_OFFSETS_M = np.array(
    [0.008] + [0.008, 0.007, 0.005, 0.004] * 5, dtype=np.float64
)
MIN_DEPTH_M, MAX_DEPTH_M = 0.12, 2.0
MAX_CORRECTION_M = 0.045


class FusionUnavailable(ValueError):
    """No defensible depth-constrained skeleton is available for this hand."""

    def __init__(self, reason: str):
        self.reason = reason
        super().__init__(reason)


def deproject_pixel(uv, depth_m: float, intrinsics: Mapping) -> np.ndarray:
    """Adapted deprojectz: [(u-ppx)/fx, (v-ppy)/fy, 1] * metric Z.

    This pinhole fallback is for rectified/undistorted images. A RealSense
    caller supplies its SDK deproject function for calibrated distortion.
    """
    return np.array([(uv[0] - intrinsics["ppx"]) / intrinsics["fx"],
                     (uv[1] - intrinsics["ppy"]) / intrinsics["fy"], 1.0]) * depth_m


def _landmarks(value, name):
    try:
        result = np.asarray(value, dtype=np.float64)
    except (ValueError, TypeError) as exc:
        raise FusionUnavailable(f"{name} must contain 21 finite XYZ landmarks") from exc
    if result.shape != (21, 3) or not np.isfinite(result).all():
        raise FusionUnavailable(f"{name} must contain 21 finite XYZ landmarks")
    return result


def _patch_candidates(depth, pixel, radius):
    """Separate depth layers instead of averaging a hand edge with background."""
    x, y = np.rint(pixel).astype(int)
    height, width = depth.shape
    if not (0 <= x < width and 0 <= y < height):
        return [], "out_of_frame"
    patch = depth[max(0, y - radius):min(height, y + radius + 1),
                  max(0, x - radius):min(width, x + radius + 1)]
    values = np.sort(patch[np.isfinite(patch) & (patch >= MIN_DEPTH_M) & (patch <= MAX_DEPTH_M)])
    if len(values) < 4:
        return [], "depth_hole"
    groups = np.split(values, np.flatnonzero(np.diff(values) > 0.012) + 1)
    candidates = []
    for group in groups:
        if len(group) < max(4, int(patch.size * 0.18)):
            continue
        median = float(np.median(group))
        mad = float(np.median(np.abs(group - median)))
        if mad > 0.009:
            continue
        support = len(group) / patch.size
        center = float(depth[y, x])
        center_match = np.isfinite(center) and abs(center - median) < 0.012
        confidence = support * np.exp(-mad / 0.005) * (1.0 if center_match else 0.6)
        candidates.append((median, float(confidence)))
    return sorted(candidates, key=lambda item: -item[1]), "depth_discontinuity"


def _root_depth(candidates, relative):
    estimates = [(i, samples[0][0] + SURFACE_OFFSETS_M[i] - relative[i, 2])
                 for i, samples in enumerate(candidates) if samples]
    palm_estimates = [z for i, z in estimates if i in PALM]
    if len(palm_estimates) < 2 or len(estimates) < 5:
        raise FusionUnavailable("insufficient depth: need five joints including two palm landmarks")
    # A wrong layer at one palm landmark must not relocate the whole hand.
    root = float(np.median(palm_estimates))
    agreeing = [z for i, z in estimates if i in PALM and abs(z - root) < 0.055]
    if len(agreeing) < 2:
        raise FusionUnavailable("palm depth layers are inconsistent")
    return float(np.median(agreeing))


def _calibrate_scale(rays, prior, root):
    """Use projected palm size and measured range; never use a single fingertip."""
    projected = rays * (root + prior[:, 2, None])
    pairs = ((0, 5), (0, 9), (0, 13), (0, 17), (5, 13), (5, 17), (9, 17))
    ratios = []
    for a, b in pairs:
        expected = np.linalg.norm(prior[b, :2] - prior[a, :2])
        if expected > 0.008:
            ratios.append(np.linalg.norm(projected[b, :2] - projected[a, :2]) / expected)
    if len(ratios) < 3:
        raise FusionUnavailable("palm projection is too degenerate to calibrate")
    scale = float(np.median(ratios))
    if not 0.60 <= scale <= 1.70:
        raise FusionUnavailable("depth and MediaPipe palm scale disagree")
    return scale


def fuse_hand(normalized, world, depth_m, intrinsics, *, deproject: Callable | None = None):
    """Return corrected wrist-relative landmarks and JSON-safe diagnostics.

    ``deproject(uv, z_m)`` may provide the RealSense SDK's distortion-aware
    deprojection. Failure raises FusionUnavailable; callers must report an
    unavailable hand rather than silently emitting the MediaPipe prior.
    """
    normalized = _landmarks(normalized, "normalized")
    prior = _landmarks(world, "world").copy()
    prior -= prior[0].copy()
    try:
        depth = np.asarray(depth_m, dtype=np.float64)
        camera = {key: float(intrinsics[key]) for key in ("fx", "fy", "ppx", "ppy", "width", "height")}
    except (KeyError, TypeError, ValueError) as exc:
        raise FusionUnavailable("invalid depth image or color intrinsics") from exc
    if (not all(np.isfinite(value) for value in camera.values())
            or camera["fx"] <= 0 or camera["fy"] <= 0
            or camera["width"] <= 0 or camera["height"] <= 0
            or camera["width"] != int(camera["width"]) or camera["height"] != int(camera["height"])
            or depth.shape != (int(camera["height"]), int(camera["width"]))):
        raise FusionUnavailable("depth dimensions must match finite positive color intrinsics")
    bone_indices = np.array(BONES, dtype=int)
    original_lengths = np.linalg.norm(prior[bone_indices[:, 1]] - prior[bone_indices[:, 0]], axis=1)
    if np.any(original_lengths < 0.003) or np.any(original_lengths > 0.15):
        raise FusionUnavailable("MediaPipe skeleton contains degenerate or nonmetric bones")
    pixels = normalized[:, :2] * [camera["width"], camera["height"]]
    callback = deproject or (lambda uv, z: deproject_pixel(uv, z, camera))
    try:
        rays = np.asarray([callback(pixel.tolist(), 1.0) for pixel in pixels], dtype=np.float64)
    except Exception as exc:
        raise FusionUnavailable("camera deprojection failed") from exc
    if rays.shape != (21, 3) or not np.isfinite(rays).all() or np.any(abs(rays[:, 2] - 1) > 1e-4):
        raise FusionUnavailable("camera deprojection returned invalid rays")
    radius = int(np.clip(round(camera["width"] / 220), 2, 5))
    sampled = [_patch_candidates(depth, pixel, radius) for pixel in pixels]
    candidates = [sample[0] for sample in sampled]
    root = _root_depth(candidates, prior)
    scale = _calibrate_scale(rays, prior, root)
    prior *= scale
    root = _root_depth(candidates, prior)
    initial_z = root + prior[:, 2]
    if np.any(initial_z <= MIN_DEPTH_M) or np.any(initial_z >= MAX_DEPTH_M):
        raise FusionUnavailable("hand range is outside the supported 0.12–2.0 meters")
    # Image handedness and the world prior must agree. No left/right reflection
    # is performed here; mirroring belongs only in the display layer.
    projected = rays * initial_z[:, None]
    palm_cross = np.cross(prior[5] - prior[0], prior[17] - prior[0])[2]
    image_cross = np.cross(projected[5] - projected[0], projected[17] - projected[0])[2]
    if abs(palm_cross) > 1e-5 and abs(image_cross) > 1e-5 and palm_cross * image_cross < 0:
        raise FusionUnavailable("image and world landmarks have inconsistent handedness")
    targets = np.zeros(21)
    confidence = np.zeros(21)
    reasons = [entry[1] for entry in sampled]
    measured = np.full(21, np.nan)
    for i, samples in enumerate(candidates):
        if not samples:
            continue
        expected_surface = initial_z[i] - SURFACE_OFFSETS_M[i]
        surface, weight = min(samples, key=lambda item: abs(item[0] - expected_surface))
        if abs(surface - expected_surface) > MAX_CORRECTION_M:
            reasons[i] = "occluded_or_inconsistent_depth"
            continue
        # If a nearer landmark overlaps this ray, visible depth belongs to it.
        overlaps = np.linalg.norm(pixels - pixels[i], axis=1) <= 2 * radius
        nearer = overlaps & (initial_z < initial_z[i] - 0.012)
        if nearer.any() and surface < expected_surface - 0.010:
            reasons[i] = "occluded_by_nearer_joint"
            continue
        measured[i], targets[i], confidence[i] = surface, surface + SURFACE_OFFSETS_M[i], weight
        reasons[i] = "measured"
    valid = confidence > 0
    if np.count_nonzero(valid) < 5 or np.count_nonzero(valid[list(PALM)]) < 2:
        raise FusionUnavailable("insufficient consistent hand depth after occlusion rejection")
    indices = np.flatnonzero(valid)
    lengths = original_lengths * scale
    weights = np.sqrt(confidence[indices])
    # Twenty-one scalar depths are solved on the calibrated image rays. X, Y
    # and Z therefore stay in one camera frame; no mixing of MP and camera XY.
    # Surface intervals permit 3 mm uncertainty around the approximate radius.
    def residual(z):
        points = rays * z[:, None]
        vectors = points[bone_indices[:, 1]] - points[bone_indices[:, 0]]
        bone_error = (np.linalg.norm(vectors, axis=1) - lengths) / 0.004
        relative_error = ((z - z[0]) - prior[:, 2]) / 0.040
        observation = z[indices] - targets[indices]
        interval_error = np.sign(observation) * np.maximum(abs(observation) - 0.003, 0) * weights / 0.004
        weak_center = observation * weights / 0.015
        return np.concatenate((relative_error, bone_error, interval_error, weak_center, [(z[0] - root) / 0.008]))

    def jacobian(z):
        jac = np.zeros((21 + len(BONES) + 2 * len(indices) + 1, 21))
        jac[:21] = np.eye(21) / 0.040
        jac[:21, 0] -= 1 / 0.040
        points = rays * z[:, None]
        for row, (a, b) in enumerate(BONES, 21):
            vector = points[b] - points[a]
            direction = vector / max(np.linalg.norm(vector), 1e-9)
            jac[row, a] = -np.dot(direction, rays[a]) / 0.004
            jac[row, b] = np.dot(direction, rays[b]) / 0.004
        start = 21 + len(BONES)
        jac[start + np.arange(len(indices)), indices] = (abs(z[indices] - targets[indices]) > 0.003) * weights / 0.004
        jac[start + len(indices) + np.arange(len(indices)), indices] = weights / 0.015
        jac[-1, 0] = 1 / 0.008
        return jac

    lower = np.maximum(initial_z - MAX_CORRECTION_M, MIN_DEPTH_M)
    upper = np.minimum(initial_z + MAX_CORRECTION_M, MAX_DEPTH_M)
    try:
        fit = least_squares(residual, initial_z, jac=jacobian, bounds=(lower, upper),
                            loss="soft_l1", f_scale=1.0, max_nfev=45,
                            ftol=1e-5, xtol=1e-5, gtol=1e-5)
    except (ValueError, FloatingPointError) as exc:
        raise FusionUnavailable("depth-constrained optimization failed") from exc
    if not fit.success or not np.isfinite(fit.x).all():
        raise FusionUnavailable("depth-constrained optimization did not converge")
    points = rays * fit.x[:, None]
    fitted_lengths = np.linalg.norm(points[bone_indices[:, 1]] - points[bone_indices[:, 0]], axis=1)
    length_ratio = fitted_lengths / lengths
    if np.any(length_ratio < 0.65) or np.any(length_ratio > 1.35):
        raise FusionUnavailable("depth fit violates skeleton bone-length limits")
    surface_residual = fit.x - (measured + SURFACE_OFFSETS_M)
    if np.median(abs(surface_residual[valid])) > 0.012:
        raise FusionUnavailable("depth fit disagrees with visible hand surface")
    relative = points - points[0]
    diagnostics = {
        "method": "realsense_depth_constrained_21",
        "measured_count": int(valid.sum()),
        "inferred_count": int((~valid).sum()),
        "scale": scale,
        "root_depth_m": float(fit.x[0]),
        "bone_rms_m": float(np.sqrt(np.mean((fitted_lengths - lengths) ** 2))),
        "surface_rms_m": float(np.sqrt(np.mean(surface_residual[valid] ** 2))),
        "relative_z_correction_rms_m": float(np.sqrt(np.mean((relative[:, 2] - prior[:, 2]) ** 2))),
        "solver_evaluations": int(fit.nfev),
        "surface_offset_model": "approximate_4_to_8_mm_with_3_mm_tolerance",
        "coordinate_system": "camera_x_right_y_down_z_away_wrist_relative_meters",
        "points": [{"index": i, "depth_valid": bool(valid[i]), "inferred": not bool(valid[i]),
                    "reason": reasons[i], "confidence": float(confidence[i]),
                    "surface_depth_m": float(measured[i]) if valid[i] else None,
                    "surface_residual_m": float(surface_residual[i]) if valid[i] else None,
                    "z_correction_m": float(relative[i, 2] - prior[i, 2])}
                   for i in range(21)],
    }
    return {"world": relative.tolist(), "normalized": normalized.tolist(), "diagnostics": diagnostics}
