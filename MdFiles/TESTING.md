# Umbra local testing and measurement

The native tests use the host C++ compiler, OpenCV, and nlohmann_json. On the
current Arch host these are OpenCV 5.0 and nlohmann_json 3.12. The Orange Pi
build continues to use the Debian 13 ARM64 sysroot (OpenCV 4.10 and JSON
3.11.3). Ninja and CMake 3.20 or newer are required. Install missing native
packages with the system package manager only after reviewing the change.

Run from a Fish or Bash terminal in this repository:

```sh
bash scripts/test-local.sh
```

The script configures `build/native-tests`, builds it, and runs every CTest
case. It exits nonzero while the known baseline defects remain. To run only
cases that currently pass, use `bash scripts/test-local.sh -L baseline`.
Passing that subset is not a substitute for running the entire suite.

Equivalent individual commands:

```sh
cmake -S . -B build/native-tests -G Ninja -DUMBRA_BUILD_APP=OFF -DUMBRA_BUILD_TESTS=ON -DOpenCV_DIR=/usr/lib/cmake/opencv5 -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build/native-tests --parallel 4
ctest --test-dir build/native-tests --output-on-failure
ctest --test-dir build/native-tests --output-on-failure -R '^basis\.identity$'
ctest --test-dir build/native-tests -V -R '^benchmark\.synthetic_pose$'
```

Set `OpenCV_DIR` to your native installation's CMake package directory if it
differs. `scripts/test-local.sh` uses the current Arch package location when
available. Rerun the script after every source or camera configuration edit.
Configuration tests read `config.json` at execution time; they never write it.

Build the ARM64 application **without transferring it**:

```sh
podman run --rm -v "$PWD/..:/work" -w /work/Umbra umbra-cross cmake -S . -B build/arm64 -G Ninja -DCMAKE_TOOLCHAIN_FILE=/work/Umbra/cmake/aarch64-dietpi.cmake -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=/work/umbra-sysroot/usr/lib/aarch64-linux-gnu/cmake/opencv4 -Dnlohmann_json_DIR=/work/umbra-sysroot/usr/share/cmake/nlohmann_json
podman run --rm -v "$PWD/..:/work" -w /work/Umbra umbra-cross cmake --build build/arm64 --parallel 4
```

The commands assume the checkout is `/work/Umbra` and the sibling sysroot is
`/work/umbra-sysroot`, as in `AGENTS.md`. Fish accepts `$PWD` and these external
commands. Do not use `scripts/deploy-test.sh` for build-only verification: it
also transfers an executable to the Pi.

## What the tests mean

`pose_math.cpp` is compiled into both `Umbra` and `umbra_pose_tests`. The
application calls its field/tag corner transformation, distortion handling,
PnP solve/refinement, coordinate conversion, and yaw extraction. Tests call
these same functions. Stage 1 moved the existing calculations without
intentionally changing them.

The fixed cases use predetermined robot poses and camera mounts. The synthetic
image generator independently defines WPILib NWU (X forward, Y left, Z up),
OpenCV optical axes (X right, Y down, Z forward), tag corners, rigid transforms,
and the pinhole/rational distortion equations. It projects physically visible
tags into 1280×800 images. The expected robot and optical poses are the input
poses to this independent generator; they are never calculated by calling the
production inverse-transform code. These conventions follow the official
[WPILib coordinate documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html)
and [OpenCV PnP documentation](https://docs.opencv.org/4.10.0/d5/d1f/calib3d_solvePnP.html).

The `config.starboard` and `config.port` cases read the current camera mounts
and calibration from `config.json`, validate finite values and rigid rotation,
generate new synthetic observations, and compare against a separately chosen
robot pose. Changing a mount does not require editing the tests. The fixed
cases remain independent of that configuration. These tests cannot establish
that the configured mounts or intrinsics match the physical hardware.

`basis.*` compares the complete field-from-robot matrix against an independent
known transform. `yaw.*` and `fixed.*` exercise final published x, y, and yaw.
`fixed.updated_starboard` and `fixed.updated_port` preserve the two updated
mount measurements supplied for this stage as fixed fixtures. They do not
change `config.json` or claim physical verification.
`optical.*` checks the actual PnP/refinement stage separately from the known
coordinate-conversion defect. `tag.single` confirms the current policy rejects
one tag; it does not claim that OpenCV cannot solve one tag. `tag.planar`
provides a clean coplanar example, not a proof that ambiguity is handled.
`invalid.*` and `field.reference` expose limited validation behavior; they do
not cover all startup error paths.

The `known_defect` CTest label identifies intentionally retained failing
checks. No test has `WILL_FAIL` or an expected value adjusted to the current
incorrect output. Once a later approved fix passes, update the label and
document the change. CTest's normal exit status is a failure until then.

`benchmark.synthetic_pose` reports a warmed host PnP median, p95, and mean.
It has no performance pass threshold because a host timing is neither Orange
Pi throughput nor camera-to-robot latency. Run it alone in a quiet environment
for comparable numbers. A regular parallel CTest run can distort timing.

On the Stage 1 host run, 300 measured solves after 20 warmups gave median
**0.0241 ms**, p95 **0.0279 ms**, and mean **0.0246 ms**. This is a synthetic
optical PnP solve only. It does not include image retrieval, decoding,
AprilTag detection, streaming, NetworkTables, or robot processing.

## Runtime diagnostics

The production binary contains a disabled-by-default local timing summary.
Set `UMBRA_DIAGNOSTICS=1` in the process environment to enable it. Every five
seconds each camera worker prints its processed-frame count and FPS, last
processed frame sequence, publication and rejected-observation counts, last
status, last successful x/y/yaw, time since last publication, and
average/maximum/sample count in milliseconds for:

- blocking capture read and intervals between successful retrievals;
- BGR-to-gray conversion, AprilTag detection, preview preparation, image blur,
  conditional subpixel corner refinement, and field/tag correspondence work;
- PnP plus robot pose recovery, NetworkTables `Set()` call, intervals between
  calls, retrieval-to-publication completion, and full processing iteration.

Metrics with no samples in a window are omitted. Preview timing measures the
camera worker's frame copy and overlay, not HTTP JPEG encoding or viewer
delivery. Corner-refinement timing is present only when the size threshold is
met. Publication call timing is not remote arrival timing. The frame sequence
is a count of successfully retrieved frames in this worker, not a camera
sequence number. The output does not measure sensor exposure time, source
frame age, network arrival, or robot estimator consumption. Those need
timestamp-capable capture and receiver-side/hardware instrumentation in later
approved stages. Camera open/read failures also print a status transition.
The local publication gate rejects a pose if more than 500 ms elapsed since
frame retrieval; this cannot detect age already accumulated in the camera or
decoder.

## NetworkTables observation contract

Each camera publishes to `/Umbra/<camera>/observations`. A successful update
is `[timestamp_s, x_m, y_m, 0, 0, 0, yaw_rad, accepted_tag_count,
distance_m]`. Position and yaw are robot pose in field NWU; yaw is
counterclockwise positive in `[-pi, pi]`. The zeros remain reserved
placeholders. Distance remains the norm of the optical PnP translation
(camera to field origin), pending robot-side confirmation. No array is sent
for fewer than two accepted tags, a failed solve, invalid geometry, a
processing error, or a frame over 500 ms old after retrieval.

The timestamp is sampled just after `cap.read()` and is `nt::Now()` plus
`GetServerTimeOffset()` when available, expressed in seconds. Before NT
synchronization, the fallback is the local NT clock. The robot subscriber
must validate nine finite elements, tag count greater than one, and new
timestamp/order per camera; use receipt time to expire an old retained topic
value, and compare source age against the robot clock only after the clock
offset is known. NT publisher options remain at their library defaults:
100 ms period with `sendAll=false`, so frame-by-frame delivery is not
guaranteed. Measure arrival rates and capture-to-receipt latency on hardware.

## Baseline and future work

Run `ctest --test-dir build/native-tests --output-on-failure` to reproduce the
current correctness baseline and see expected versus actual values. Before
the B2 distortion fix, the full suite had **43 cases: 13 passed, 30 failed,
0 skipped**. After B2, **16 passed and 27 failed**; the three distortion cases
passed without changing their expectations. After B1/B3 and invalid-input
handling, the complete native suite has **42 passed, 1 failed**. Only the
unchanged distance-semantics case remains a known defect.
Examples from the original Stage 1 baseline:

| Case | Expected | Actual |
| --- | --- | --- |
| `basis.identity` | field-from-robot matrix (0,0) = 1 | approximately 0 |
| `yaw.positive90` | yaw +90° | +215°; wrapped error +125° |
| `optical.radial` | optical translation component = 3.200 m | 3.18643 m |
| `optical.edge` | optical translation component = 3.200 m | 3.12673 m |
| `config.starboard` | robot x = 4.100 m, yaw = +25° | x = 4.18276 m, yaw = +205.053° |
| `config.port` | robot x = 4.100 m, yaw = +25° | x = 3.37700 m, yaw = +205.053° |
| `range.tag_distance` | nearest tag distance = 2.52488 m | 5.20259 m |

The failed values above are historical before-fix measurements, except for
`range.tag_distance`, whose production distance meaning remains unchanged.
`invalid.nonfinite_mount` now expects explicit rejection rather than a finite
rotation from a NaN input; no independent pose expectations changed. The
audited work list is in
[ROADMAP.md](ROADMAP.md).

Remaining tests need the Pi or robot: actual camera FPS and exposure,
GStreamer negotiation and buffering, thermal behavior, NetworkTables arrival
rate, subscriber freshness policy, physical calibration, tag layout, camera
mount measurements, viewer lifecycle under load, and disconnect/reconnection.
