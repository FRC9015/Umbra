# Configuration, calibration, and robot integration

Umbra reads `/home/dietpi/Umbra/config.json` and `/home/dietpi/Umbra/field.json` by **absolute path** at startup. These paths are hard-coded in `main.cpp`; a different login or installation directory requires a source change. Make backups before editing production files. JSON changes to team number, camera list, paths, calibration, or mounts require an application restart. The web interface can apply supported V4L2 controls live and save them to `config.json`.

## Configuration shape

The checked-in `config.json` defines `team_number` and a `cameras` array. This sanitized **illustrative** example shows the supported shape; numbers and camera path below are **not calibrated values** and must be replaced before robot use:

```json
{
  "team_number": 9999,
  "cameras": [
    {
      "name": "front",
      "path": "/dev/v4l/by-path/REPLACE_WITH_CAPTURE_NODE",
      "calibration": {
        "fx": 900.0,
        "fy": 900.0,
        "cx": 640.0,
        "cy": 400.0,
        "dist": [0.0, 0.0, 0.0, 0.0, 0.0]
      },
      "offset": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0
      },
      "tuning": {
        "auto_exposure": false,
        "exposure": 25,
        "gain": 0,
        "brightness": 0
      }
    }
  ]
}
```

Add an object for each camera. Camera `name` becomes the web stream URL and NetworkTables subtable; use unique, URL-friendly names. Camera `path` is a V4L2 capture device. Prefer a stable `/dev/v4l/by-path/…-video-index0` entry after checking the device and physical port. Do not copy Team 9015's existing path strings to another Pi.

| Key | Units / meaning | Used now? |
| --- | --- | --- |
| `team_number` | FRC team used by `SetServerTeam` | Yes, at startup |
| `calibration.fx/fy` | Focal lengths in pixels | Yes |
| `calibration.cx/cy` | Principal point in pixels | Yes |
| `calibration.dist` | OpenCV lens distortion coefficients | Yes; accepted lengths are 4, 5, 8, 12, or 14 |
| `offset.x/y/z` | Camera origin in robot frame, meters | Yes |
| `offset.roll/pitch/yaw` | Camera mount rotation, degrees | Yes |
| `tuning.auto_exposure/exposure/gain/brightness` | Initial V4L2 camera controls | Yes, if `tuning` exists |
| `tuning.decimate/threads/margin/latency_ms` | Fields present in current JSON | **No**; source hard-codes detector settings and does not read these keys |
| `offset.exposure/gain/brightness` | Legacy fields in current JSON | **No**; controls come from `tuning` |

The source applies `auto_exposure=3` for automatic mode and `auto_exposure=1` with `exposure_time_absolute` for manual mode, plus gain and brightness via `v4l2-ctl`. Whether a specific USB camera accepts each control must be checked with `v4l2-ctl --list-ctrls`. If `tuning` is absent, defaults are manual exposure 25, gain 0, brightness 0. Current detector settings are 4 threads, `quad_decimate=2.0`, and acceptance margin strictly above 50. Capture requests MJPEG 1280×800 at 100 fps. These are source constants, not configurable JSON knobs today.

## Calibration and camera mounts

Calibrate **each physical camera** at the actual resolution and focus used by Umbra. Record the OpenCV camera matrix

```text
[ fx   0  cx ]
[  0  fy  cy ]
[  0   0   1 ]
```

and an OpenCV-compatible `dist` vector in the order returned by the calibration process (for the common five-term model: `k1,k2,p1,p2,k3`). Do not reuse another camera's coefficients, approximate them from field measurements, or set them to zero unless calibration supports that. Intrinsics are in **pixels**, not meters. The production solver passes the same distortion to SQPnP and LM refinement. See [Testing](TESTING.md) for synthetic distortion cases.

`offset` describes the **robot-to-camera rigid transform**: camera position in the robot frame and roll/pitch/yaw of the camera mount, with rotation built as `Rz(yaw) * Ry(pitch) * Rx(roll)`. Values are **degrees** in JSON; output yaw is radians. The program converts OpenCV's optical camera axes to the WPILib-style NWU convention and removes this measured mount transform to obtain field-relative robot pose. In the robot frame, +X is forward, +Y is left, +Z is up. Field pose X/Y use meters. Measure the optical origin and camera attitude for each mounted camera, then test against known robot placements; neither the example zeros nor Team 9015's physical offsets are transferable.

The tag corner model uses a source constant `tagSizeMeters = 0.1651`. Confirm that this is the correct printed tag size for your field. A mismatched size biases range and pose.

## Field layout

`field.json` contains a `tags` array. For each tag, the loader reads `ID`, `pose.translation.x/y/z` in meters, and `pose.rotation.quaternion.W/X/Y/Z`. The checked-in file is one specific field layout, not a universal FRC layout. Use the field for the event and verify IDs, dimensions, tag coordinates, quaternion convention, and units. The loader silently returns when the file cannot be opened; then no tag is in `fieldMap` and the app cannot publish valid multi-tag poses. Invalid JSON can terminate startup. Back up the Pi's field file before seasonal changes and validate against official field data.

## NetworkTables observation format

Umbra starts an NT4 client named `UmbraPi`, sets the server team from `team_number`, and publishes a double array at `/Umbra/<camera-name>/observations` for each camera. Successful solves publish:

| Index | Value | Units / caveat |
| --- | --- | --- |
| 0 | Capture timestamp | seconds from `nt::Now()` plus NT server offset if available |
| 1 | Robot field X | meters |
| 2 | Robot field Y | meters |
| 3–5 | 0, 0, 0 | reserved/unused in current format |
| 6 | Robot yaw | radians |
| 7 | Accepted tag count | numeric count, at least two |
| 8 | `distance` | meters; currently `norm(tvec)`, **not verified tag range** |

A measurement is rejected when there are fewer than two accepted known tags, the pose solve fails, or more than 500 ms elapse after frame retrieval before publication. No clearing value is emitted when a solve fails, so a client can still see an old array. Use the NT value's arrival time and payload timestamp, check connection/time synchronization, reject stale or nonfinite values, and apply plausibility/outlier gating before calling a WPILib pose-estimator vision method. Robot-side fusion code is not in this repository. The known `range.tag_distance` test failure means index 8 should not be interpreted as nearest-tag distance.

## Web controls and diagnostics

The HTTP server listens on **all interfaces, port 5801**. On a trusted network, browse `http://PI_IP:5801/`; `/stream/<name>` serves a preview with detected-tag outlines. `/set` changes camera controls in memory and invokes `v4l2-ctl`. `/save` writes current tuning fields into the absolute production `config.json`. The source has **no authentication or TLS**, so isolate the service on a trusted robot network and do not expose it to the public Internet. Save only after backing up configuration and confirming the changed control behaves as intended.

Set `UMBRA_DIAGNOSTICS=1` in the process environment to enable approximately five-second, per-camera status, timing, and pose logs. See [Operations](OPERATIONS.md#diagnostics-and-logs). Logs and web preview aid diagnosis; validate pose correctness on known field points and the roboRIO.
