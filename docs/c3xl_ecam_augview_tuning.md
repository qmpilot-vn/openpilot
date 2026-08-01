# C3XL ecam aug-view tuning (UI-only)

Overlay FL + wide-euler bias apply to the **raylib aug view only**.
Modeld warp always uses stock `DEVICE_CAMERAS` (`ecam=567`, `fcam=2648`).

## Defaults (aug UI)

| Setting | Value |
|---|---|
| ecam FL (JSON) | `650` |
| fcam FL (JSON) | `2700` |
| wide euler bias `[roll, pitch, yaw]` deg | `[0, 1, 0]` |

Device file: `/data/qmpilot/camera_focal_length.json`

```json
{
  "ecam": 650.0,
  "fcam": 2700.0,
  "wide_euler_bias_deg": [0.0, 1.0, 0.0]
}
```

Fallback paths (same schema):

- `~/.qmpilot_camera_focal_length.json`
- `/tmp/qmpilot_camera_focal_length.json`

## Code split

| Path | Uses JSON FL / bias? |
|---|---|
| `selfdrive/ui/onroad/augmented_road_view.py` | **Yes** (overlay K + wide euler bias) |
| `sunnypilot/modeld_v2/modeld.py` | **No** — stock intrinsics only (`ecam_fl=0`) |
| `common/transformations/camera.py` | Stock ecam **567** for modeld |

## Files

- `common/transformations/camera.py` — keep stock ecam **567** (do not bake 650 into DEVICE_CAMERAS)
- `sunnypilot/modeld_v2/modeld.py` — do **not** call `get_focal_lengths()` / `set_focal_lengths()`
- `sunnypilot/modeld_v2/camera_fl_params.py` — JSON helpers for UI
- `selfdrive/ui/onroad/augmented_road_view.py` — applies FL + bias for overlay only

## Behavior

| Consumer | ecam FL | wide euler bias |
|---|---|---|
| Modeld warp | stock 567 | none (model’s own euler only) |
| Aug view overlay | JSON 650 | JSON +1° pitch |

- Aug view reads JSON every frame.
- `ecam` FL only affects the **wide** stream (experimental mode + speed &lt; ~10 m/s). Road cam uses `fcam`.
- Wide extrinsic + bias are UI overlay only; they do not change modeld warp driving.

## Restart

After changing `camera.py` or `modeld.py`, restart modeld + UI. JSON-only tweaks are live for the UI.

## Tune later (optional)

```bash
PYTHONPATH=/data/openpilot python3 -c "
from openpilot.sunnypilot.modeld_v2.camera_fl_params import set_focal_lengths, set_wide_euler_bias_deg
set_focal_lengths(650.0, 2700.0)
set_wide_euler_bias_deg(0.0, 1.0, 0.0)  # roll, pitch, yaw degrees
"
```

- **yaw** → path left/right
- **pitch** → path up/down
- **roll** → path tilt

## Test

```bash
python3 -m pytest sunnypilot/modeld_v2/tests/test_camera_fl_params.py -q -n0
```

On-road: experimental mode on, low speed (wide cam), confirm path sits on lane markings. Driving behavior should match stock warp (not JSON FL).
