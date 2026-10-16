# Testing Umbra

Unit tests exercise small pieces of production logic with known inputs. Umbra's native test executable compiles `tests/pose_tests.cpp` against the **same `pose_math.cpp`** used by the ARM64 application. `CMakeLists.txt` makes the app and tests separate options; `tests/CTestCases.cmake` registers the named cases with CTest. The checked-in registration currently contains **43 cases**. Native tests do not require a Pi, camera, or NetworkTables server.

## Run the suite

On the **Arch host (Fish)**, from the repository root:

```fish
bash scripts/test-local.sh
```

The Bash script configures `build/native-tests` with Ninja, `UMBRA_BUILD_APP=OFF`, `UMBRA_BUILD_TESTS=ON`, and `RelWithDebInfo`, then builds and runs CTest. It uses Arch's `/usr/lib/cmake/opencv5` when present or a supplied `OpenCV_DIR`. To configure explicitly:

```fish
cmake -S . -B build/native-tests -G Ninja \
  -DUMBRA_BUILD_APP=OFF -DUMBRA_BUILD_TESTS=ON \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build/native-tests --parallel 4
ctest --test-dir build/native-tests --output-on-failure
```

If CMake cannot find OpenCV, add the correct `-DOpenCV_DIR=…` path for your host. The native suite uses host OpenCV, while the target app is built against the Pi's OpenCV 4.10, so also cross-build any pose-math change.

List cases, filter one, or view details:

```fish
ctest --test-dir build/native-tests -N
ctest --test-dir build/native-tests -R '^optical\.radial$' --output-on-failure
ctest --test-dir build/native-tests -R '^range\.tag_distance$' -V
ctest --test-dir build/native-tests -L baseline --output-on-failure
```

The `-R` patterns are regular expressions. CTest names map to individual arguments passed to `umbra_pose_tests`. For a failed case, read its expected/actual values, inspect the matching test in `tests/pose_tests.cpp`, and trace the production function in `pose_math.cpp`. Re-run the single case after a focused change, then the full suite. Do not alter a tolerance or expectation merely to make a failure disappear.

## What the cases cover

The 43 registered tests include coordinate-basis conversion, fixed robot/camera transforms, mount offsets, yaw and wrap behavior, optical projection with zero/radial/tangential distortion and alternate intrinsics, tag corner order and multi-tag requirements, malformed and nonfinite input, field reference data, configuration examples, and a synthetic pose benchmark. The categories use synthetic camera/robot poses and floating-point tolerances; exact decimal equality would be inappropriate for iterative pose solvers. Tolerances should reflect meaningful physical error bounds and any solver variability, not be widened without justification.

`solveRobotPose` requires at least two accepted tags. The optical solver uses SQPnP followed by LM refinement with the **same** distortion vector, and the tests protect this mathematical path. A separate synthetic benchmark checks timing but is not a hardware performance guarantee. `CTestCases.cmake` labels the known failing case `known_defect` and the others `baseline`; it does not mark the failure as expected-to-pass.

## Known baseline failure

The last recorded local CTest result, before this documentation work, was **42 passed, 1 failed**: `range.tag_distance`. The current `Result.distance` comes from `cv::norm(tvec)`, which measures camera-to-field-origin translation in the OpenCV solve, while the test expects distance to a tag. This documentation pass did **not** rerun CTest and does **not** claim the defect has been fixed. Compare new runs with that baseline; investigate any newly failing case.

For the full historical test matrix, diagnostic notes, and earlier audit findings, see [the retained Stage 1 testing notes](../MdFiles/TESTING.md) and [roadmap](../MdFiles/ROADMAP.md).

## Hardware validation still required

Unit tests cannot prove USB camera compatibility, GStreamer capture rate, exposure behavior, detection under robot lighting, correct physical mount measurements, field-map choice, NetworkTables clock synchronization, roboRIO fusion, or mechanical performance. After pose-math changes, run the full native suite, cross-build, then verify on the Pi and robot using [Operations](OPERATIONS.md#verify-localization). Check stale measurements, both camera topics, pose continuity, and known field positions. Do not infer localization correctness from `systemctl active` alone.
