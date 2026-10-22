# TsenseI_retargeting

Standalone MediaPipe Hands (21 landmarks) → dexterous-hand joint retarget.

Solver code lives in this folder. **Robot assets** (URDF / meshes / keypoints /
MJCF) for this monorepo live in one place only:

`assets/hands/TsenseI_hand`

Do not duplicate them under `TsenseI_retargeting/robots/`.

## Install

```bash
pip install -r requirements.txt
# optional faster Jacobians:
# pip install pin
```

Put the **parent** of `TsenseI_retargeting/` on `PYTHONPATH` (this repo already
does that from `retarget_runtime`).

## Quick start

```python
import numpy as np
from TsenseI_retargeting import TopoRetargeter, TopoRetargetConfig

# MediaPipe Hands landmarks in meters, shape (21, 3), MediaPipe order.
landmarks = np.zeros((21, 3), dtype=float)
# ... fill from your MediaPipe pipeline ...

# Default robot_root = <repo>/assets/hands/TsenseI_hand when that folder exists.
retargeter = TopoRetargeter()
result = retargeter.retarget(landmarks)
q = result.joint_values  # dict[str, float] joint name -> radians
```

Demo smoke:

```bash
python examples/retarget_once.py
```

## Landmark order (MediaPipe Hands)

| IDs | Name |
| --- | --- |
| 0 | Wrist |
| 1–4 | Thumb CMC → Tip |
| 5–8 | Index MCP → Tip |
| 9–12 | Middle MCP → Tip |
| 13–16 | Ring MCP → Tip |
| 17–20 | Pinky MCP → Tip |

Scale / palm width uses `‖IndexMCP − PinkyMCP‖` (ids 5 and 17).

Units must be **meters**. Pixel or normalized image coordinates will not work
without your own metric reconstruction.

## Layout

```text
TsenseI_retargeting/          # solver only
  landmarks.py
  mediapipe21_schema.yaml
  toporetarget.py
  fk/
  examples/retarget_once.py
  requirements.txt
  README.md

assets/hands/TsenseI_hand/ # single robot package (shared with app)
  eval_config.yaml
  robot_keypoints_21.yaml
  urdf/
  meshes/
  mjcf/
```

## Swap robots

Provide a robot package directory with:

1. `eval_config.yaml` — `urdf_path`, `keypoint_config`, `active_joints`
2. `robot_keypoints_21.yaml` — exactly 21 keypoints, ids `0..20`,
   `source_skeleton: mediapipe_hands_21`
3. URDF (+ meshes)

Then:

```python
from TsenseI_retargeting import TopoRetargeter, TopoRetargetConfig

retargeter = TopoRetargeter(TopoRetargetConfig(robot_root="/path/to/my_hand"))
```

## Migration checklist (other projects)

Copy the solver folder **and** a robot package (e.g. `assets/hands/TsenseI_hand`):

| Piece | Path |
| --- | --- |
| Public API | `TsenseI_retargeting/__init__.py` |
| Solver | `toporetarget.py`, `landmarks.py` |
| FK stack | `fk/` |
| Schema | `mediapipe21_schema.yaml` |
| Robot assets | `assets/hands/TsenseI_hand/` (or your own) |
| Deps | `requirements.txt` |

Pass `TopoRetargetConfig(robot_root=...)` if the hand package is not at the
default monorepo path.

## Notes

- Five independent fingertip position targets (MediaPipe 4/8/12/16/20) are
  enabled by default. Each coefficient comes from the selected robot keypoint
  YAML's `weight` (2.0 in the current assets); the schema file is descriptive.
  Source points are wrist-centered, scaled to robot palm width, and expressed
  in their palm frame. Robot points use the fixed neutral-pose palm frame.
  The added loss is `0.5 * lambda_fingertip * sum(weight_i * ||tip_i-target_i||² / palm_width²)`.
  During pinch, each partner's position coefficient is multiplied by `1-alpha`;
  the thumb uses `1-max(alpha)` across its four contacts. This continuously
  releases conflicting absolute targets as contact takes priority, restoring
  the full coefficient on release. Uninvolved fingers keep their position weight.
  Residuals use `sqrt(weight_i)`, so 2.0 doubles the squared-error coefficient.
  `lambda_fingertip=1.0` is the default overall multiplier; set it to 0 to disable
  this term. Per-tip zero weights also disable the respective contribution;
  negative/nonfinite weights are rejected. Initialization-only mode omits it.
  Analytic Jacobians, finite differences, line search and loss reporting use
  the same objective. `loss["fingertip_position_weighted"]` reports its contribution;
  diagnostics report the configured weights and whether this term is active.
  This soft objective trades off with topology, bone direction and pinch terms;
  actual camera accuracy and calibrated robot reachability still limit tracking.
- Thumb contact with each of the four fingers (MediaPipe 4–8/12/16/20) is enabled
  by default: `pinch_full_ratio=0.15`,
  `pinch_start_ratio=0.50`, `pinch_weight=65.0`. The current input's 5–17 palm width
  sets both thresholds; each pair independently applies smoothstep to its own
  distance and blends its normalized FK contact residual. Multiple close tips
  can be active together; releasing a pair removes its contact residual.
  Set `pinch_enabled=False` to disable contact without disabling fingertip tracking. Contact is also
  inactive when `use_topo_optimization=False` or the weight is zero.
- `result.diagnostics["pinch"]["pairs"]` reports each finger's gap, ratio and
  activation under `index`, `middle`, `ring`, and `pinky`. The scalar gap/ratio/
  activation fields in `diagnostics["pinch"]` retain the original index meaning;
  palm width and the start threshold are shared by all pairs. Each model's
  `retarget.pinch_index_full_ratio` independently sets the thumb/index full
  activation threshold; `pinch_full_ratio` sets it for the other three fingers.
  Both must satisfy `0 <= full < pinch_start_ratio`. Omitted/null index full
  ratio inherits `pinch_full_ratio` for older configurations. Per-pair diagnostics
  report `full_ratio` and `full_threshold_m`; the top-level threshold follows index.
  Each registered model has
  its own `retarget.pinch_index_weight` for thumb/index contact (4-8), initialized
  from its existing contact weight; `pinch_weight` controls the other three pairs.
  Raising `pinch_index_weight` strengthens only the index contact objective.
  Zero disables only that pair; omitted/null inherits `pinch_weight` for older
  configurations. Per-pair diagnostics include the effective `weight`;
  the top-level weight follows index. This is independent of the fingertip
  position-tracking YAML weight of 2.0.
  `include_artifacts=True` additionally reports the solved gap and contact loss.
  Contact uses calibrated keypoints and remains a soft objective, so a requested
  zero gap is not guaranteed by every model/pose. Existing joint limits, iteration
  budgets and downstream collision/trajectory handling still apply.
- Algorithm is native MediaPipe-21 topology (not Quest-26).
- Pinocchio is optional; without it the solver uses finite-difference Jacobians.
- Live app (`retarget_runtime`) always uses `HAND_MODEL_DIR` →
  `assets/hands/TsenseI_hand`.
