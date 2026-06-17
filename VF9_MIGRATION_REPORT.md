# VinFast VF9 Migration Report

**Date:** 2026-06-17  
**Base:** sunnypilot/openpilot `dev` (v2026.06.09-4541)  
**Source fork:** qmplot (`vf8-alpha-c4`)  
**Target branch:** `vf9-alpha-c4`  
**Target remote:** `https://github.com/ronnguyen185/qmpilot.git`

---

## Summary

Migrated the VinFast port and driving stack from **qmplot** into **openpilot**, configured for **VF9** (not VF8). The device uses a single internal Panda with VinFast safety firmware aligned to openpilot’s hash-based health protocol.

---

## Platform configuration

| Setting | Value |
|---------|--------|
| `CarPlatformBundle.platform` | `VINFAST_VF9` |
| Default fingerprint (`card.py`) | `VINFAST_VF9` when bundle unset |
| `AlphaLongitudinalEnabled` | Auto-enabled for VF8/VF9 |
| `Mads` | Forced off for VinFast |
| Panda serial (device) | `1d0017000751333334373931` |
| Firmware | `DEV-157b7a23-DEBUG` (health v3194673618) |

---

## Migrated components

### VinFast car port (`opendbc/car/vinfast/`)

- `interface.py`, `carstate.py`, `carcontroller.py`, `vinfastcan.py`, `values.py`, `fingerprints.py`, `radar_interface.py`, tests
- DBCs: `vinfast_vf8_{chassis,info,body,mrr_scam}.dbc`
- Registered `VINFAST` in `opendbc/car/values.py`
- Torque override: `VINFAST_VF8` / `VINFAST_VF9` in `torque_data/override.toml` (angle control, `MAX_LAT_ACCEL_MEASURED=2.0`)

### Panda / safety

- `vinfast_stub.h` + local `vinfast.h` (gitignored) via `safety.h` `__has_include`
- `SAFETY_VINFAST` (35) in `declarations.h`, `car.capnp`
- Prebuilt firmware in `panda/board/obj/`, `panda/release/`
- VinFast panda examples + debug scripts (`manual_set_panda_vinfast.py`, etc.)

### Controls / selfdrive

- `radard.py` — VinFast radar–vision fusion (`liveTracks`)
- `latcontrol_angle.py` — VF8/VF9 angle PID tuning
- `controlsd.py` — VF9 curvature bias
- `cruise.py` — tag speed sync, pcmCruise buttons
- `longitudinal_planner.py` / `long_mpc.py` — stop gap `2.8 m`
- `car_specific.py` — pcmEnable/pcmDisable on stock ACC edges

### Sunnypilot hooks

- `controlsd_ext.py` — lateral inhibit when ACC available but not active
- `mads.py` — preserve pcmEnable for VinFast OP-long

### `card.py`

- Default `VINFAST_VF9`, `num_pandas` from `pandaStates`
- MADS disabled for VinFast
- `FirmwareQueryDone` after fingerprint

### Capnp (`car.capnp`)

- `Actuators.reengageStatus` — VinFast re-engage logging
- `RadarPoint.motionStatus` / `motionOrientation` / `laneAssignment` — radar fusion
- `CarState.brake` — restored from deprecated

### Fingerprint / FW query

- 13-message shared VF8/VF9 CAN fingerprint (fixed platform required for VF9)
- `fw_versions.py` — division-by-zero fix for empty `FW_VERSIONS` brands
- `car_helpers.py` — skip VIN/FW query when `fixed_fingerprint` set (avoids ISOTP errors on VinFast bus)
- `num_pandas` passed through fingerprint → `get_car()`

### Pandad (qmplot multi-panda stack)

- Binary + sources: USB+SPI, `bus_offset` (Panda[0]=0–3, Panda[1]=4–7)
- Per-panda safety from `safetyConfigs[]` (VinFast: vinfast + silent)
- `SINGLE_PANDA=1` env → first panda only
- Params: `PandaSignatures`, `PandaSomResetTriggered`, `PandadLogprintWarning` in `params_keys.h`; safe fallbacks in `pandad.py` until params rebuild

---

## Issues fixed during migration

| Issue | Fix |
|-------|-----|
| `ZeroDivisionError` in `get_fw_versions_ordered` | qmplot sort-key guard for empty brand ECU lists |
| `KeyError: VINFAST_VF9` in `get_torque_params` | Torque override entries |
| `reengageStatus` capnp missing | `car.capnp` field from qmplot |
| `UnknownKeyName: PandaSignatures` | `params_keys.h` + safe helpers in `pandad.py` |
| ISOTP `invalid frame type: 6` on FW query | Skip VIN/FW when fixed fingerprint set |

---

## Fingerprint notes

- VF8 and VF9 share identical CAN fingerprint; auto CAN fingerprint cannot distinguish them.
- Use `CarPlatformBundle` + `card.py` default (`VINFAST_VF9`) → `FingerprintSource.fixed`.
- `FW_VERSIONS` empty — no firmware-based identification yet.

---

## Dual vs single Panda

| Mode | Setup |
|------|--------|
| **Dual** | Internal + USB Red Panda; info CAN on logical bus 4 |
| **Single** | `SINGLE_PANDA=1` or unplug USB second panda |

---

## Not migrated / optional

- Full qmplot sunnypilot UI vehicle selector entry for VinFast
- `car_list.json` VinFast platform metadata
- VF9-specific unique CAN fingerprint (needs route capture)
- `vinfast.h` in git (local-only; use stub in CI)

---

## Git / remote

- **Previous remote:** `sunnypilot/openpilot`
- **New remote:** `ronnguyen185/qmpilot`
- **Branch:** `vf9-alpha-c4` (from openpilot `dev` + working tree changes)

---

## Verify after deploy

1. Restart manager / reboot
2. `card` starts without crash; `CP.brand == "vinfast"`, `CP.carFingerprint == "VINFAST_VF9"`
3. Panda safety mode 35 (vinfast)
4. `pandad` connects (no `PandaSignatures` loop)
5. No ISOTP spam when platform is fixed
6. Lateral angle + OP-long + radar fusion active on road

---

## File change overview (working tree)

```
31 modified tracked files, ~2200 insertions / ~800 deletions
+ untracked: opendbc/car/vinfast/, DBCs, panda examples, pandad C++ additions
```
