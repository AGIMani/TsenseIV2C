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

- Thumb contact with each of the four fingers (MediaPipe 4–8/12/16/20) is enabled
  by default: `pinch_full_ratio=0.08`,
  `pinch_start_ratio=0.50`, `pinch_weight=65.0`. The current input's 5–17 palm width
  sets both thresholds; each pair independently applies smoothstep to its own
  distance and blends its normalized FK contact residual. Multiple close tips
  can be active together; releasing a pair removes its contact residual.
  Set `pinch_enabled=False` to restore the original objective. Contact is also
  inactive when `use_topo_optimization=False` or the weight is zero.
- `result.diagnostics["pinch"]["pairs"]` reports each finger's gap, ratio and
  activation under `index`, `middle`, `ring`, and `pinky`. The scalar gap/ratio/
  activation fields in `diagnostics["pinch"]` retain the original index meaning;
  palm width, thresholds and weight are shared by all pairs.
  `include_artifacts=True` additionally reports the solved gap and contact loss.
  Contact uses calibrated keypoints and remains a soft objective, so a requested
  zero gap is not guaranteed by every model/pose. Existing joint limits, iteration
  budgets and downstream collision/trajectory handling still apply.
- Algorithm is native MediaPipe-21 topology (not Quest-26).
- Pinocchio is optional; without it the solver uses finite-difference Jacobians.
- Live app (`retarget_runtime`) always uses `HAND_MODEL_DIR` →
  `assets/hands/TsenseI_hand`.
