# Troubleshooting

Start with read-only checks, identify the failing layer, then use the linked guide for the corrective step. Commands labeled **Pi** run in Bash on the Orange Pi; **host** commands run in Fish on Arch. Stop/restart/transfer commands change the running robot; schedule them deliberately.

- [Installation](#installation)
- [Cross-compilation](#cross-compilation)
- [Testing](#testing)
- [Deployment](#deployment)
- [Runtime](#runtime)

## Installation

### Pi does not boot

**Symptoms:** no console, no DHCP lease, or repeated first-boot failure. **Likely causes:** wrong Orange Pi variant image, failed flash, unsuitable storage/power. **Read-only checks:** inspect the exact image filename, checksum, board label, power specification, and router lease table; inspect console output if available. **Correction:** re-download the **Orange Pi 5 Ultra** ARMv8/Trixie image from the [official DietPi source](https://dietpi.com/docs/hardware/), verify it, and reflash per [Pi setup](PI_SETUP.md#image-flash-and-first-boot). Reflashing erases the target storage.

### No network or SSH

**Symptoms:** host cannot connect, connection refused, timeout, or key rejection. **Likely causes:** missing DHCP/route, changed IP, SSH server state, user/key mismatch, or host sandbox credentials. **Read-only checks:** on **Pi**, `hostname -I`, `ip -br addr`, `ip route`, and `systemctl status ssh --no-pager`; on **host**, `ssh -v dietpi@PI_IP` with your actual address. A timeout points more toward network; authentication rejection points toward SSH identity. **Correction:** fix network addressing or authorized SSH access through the normal DietPi setup process; do not assume the development environment shares your terminal's SSH agent. See [Pi setup](PI_SETUP.md#image-flash-and-first-boot).

### Missing target dependencies

**Symptoms:** `ldd` reports `not found`, `v4l2-ctl` or `gst-launch-1.0` missing, or CMake package metadata absent. **Likely causes:** target packages or WPILib ARM64 libraries were never installed, or loader search paths differ. **Read-only checks (Pi):**

```bash
ldd /home/dietpi/Umbra/build/Umbra
ldconfig -p | grep -E 'lib(apriltag|opencv|ntcore|wpiutil|wpinet)'
command -v v4l2-ctl gst-launch-1.0
```

**Correction:** install matching ARM64 dependencies, verify their versions and paths, and refresh the sysroot. The repository does not record the WPILib installation source; this must be resolved for a fresh setup. See [Pi setup](PI_SETUP.md#verify-os-and-update-packages) and [Cross-compiling](CROSS_COMPILING.md#prepare-the-target-before-copying-a-sysroot).

## Cross-compilation

### Podman or image error

**Symptoms:** `podman: command not found`, image `umbra-cross` missing, or mount denial. **Likely causes:** host package/image absent or mount path incorrect. **Read-only checks (host):** `podman --version`, `podman images`, `pwd`, `ls ../umbra-sysroot`. **Correction:** install Podman and build the checked-in image with `podman build -t umbra-cross -f Containerfile.cross .` from the repository root. Keep `Umbra` and `umbra-sysroot` as siblings or adapt the mount/toolchain consistently. See [Cross-compiling](CROSS_COMPILING.md#first-time-host-setup).

### CMake cannot find OpenCV, JSON, AprilTag, or WPILib

**Symptoms:** `find_package` failure or missing header/library during configure/build. **Likely causes:** incomplete sysroot, stale CMake cache, wrong CMake package directory, or missing target installation. **Read-only checks (host):**

```fish
ls ../umbra-sysroot/usr/lib/aarch64-linux-gnu/cmake/opencv4
ls ../umbra-sysroot/usr/share/cmake/nlohmann_json
ls ../umbra-sysroot/usr/include/apriltag/apriltag.h
ls ../umbra-sysroot/usr/local/lib/libntcore.so
rg -n 'OpenCV_DIR|nlohmann_json_DIR|CMAKE_SYSROOT' build/arm64/CMakeCache.txt
```

**Correction:** install the ARM64 dependency on the Pi, refresh the relevant sysroot path, then rerun [the exact configure command](CROSS_COMPILING.md#configure-and-build). Do not point target CMake at host x86-64 libraries. The current app links `apriltag`, `ntcore`, `wpiutil`, and `wpinet`; their headers and transitive dependencies must be compatible.

### Linker errors or broken sysroot links

**Symptoms:** undefined references, `cannot find -l…`, or missing BLAS/LAPACK sonames. **Likely causes:** incomplete target library copy, a symlink target outside the copied tree, incompatible WPILib set, or stale cache. **Read-only checks (host):** `readlink -e ../umbra-sysroot/usr/lib/aarch64-linux-gnu/libblas.so.3` and `file`/`readelf` on suspect libraries. **Correction:** copy the missing **ARM64** library from the Pi; the recorded BLAS/LAPACK repair uses `rsync -L` and is documented in [Cross-compiling](CROSS_COMPILING.md#copy-the-sysroot-from-the-pi). Refresh the sysroot after target upgrades. Do not hide errors with host libraries.

### Wrong executable architecture

**Symptoms:** Pi says `Exec format error`. **Likely cause:** a native x86-64 binary was transferred. **Read-only check (host):** `file build/arm64/Umbra`; expected architecture is AArch64. **Correction:** build with the checked-in toolchain and transfer `build/arm64/Umbra`, not a host build output. See [Cross-compiling](CROSS_COMPILING.md#configure-and-build).

## Testing

### Native build fails

**Symptoms:** `test-local.sh` cannot find OpenCV or compiler. **Likely causes:** host packages absent or OpenCV's CMake path differs. **Read-only checks (host):** `cmake --version`, `ninja --version`, `ls /usr/lib/cmake/opencv5/OpenCVConfig.cmake`. **Correction:** install host build packages or set `OpenCV_DIR` to the actual host package; then rerun [the script](TESTING.md#run-the-suite). The native test build intentionally does not link the full camera/NT application.

### CTest case fails

**Symptoms:** a named case reports expected/actual mismatch. **Likely causes:** real regression, current known defect, changed OpenCV behavior, or wrong fixture assumptions. **Read-only checks (host):** `ctest --test-dir build/native-tests -N` and `ctest --test-dir build/native-tests -R '^CASE_NAME$' -V`, replacing the case name. **Correction:** inspect the test and `pose_math.cpp`, make a focused production fix, then rerun the case and full suite. The recorded `range.tag_distance` failure is unresolved; 42 other cases passed in the last saved run. See [Testing](TESTING.md#known-baseline-failure).

### Numerical tolerance error

**Symptoms:** tiny floating-point differences or a larger pose discrepancy. **Likely causes:** solver iteration/platform variation, changed coordinate convention, bad calibration inputs, or actual math defect. **Read-only checks:** compare absolute/relative error in test output and inspect the synthetic scene. **Correction:** trace units and transforms first; adjust a test tolerance only with a justified numerical/physical bound, never simply to make a failure pass. See [Testing](TESTING.md#what-the-cases-cover).

## Deployment

### SSH or rsync transfer fails

**Symptoms:** `Permission denied`, `rsync: command not found`, or transport failure. **Likely causes:** wrong user/address/identity, rsync absent, or target directory permissions. **Read-only checks (host):** `ssh -v dietpi@PI_IP`; **Pi:** `command -v rsync` and `ls -ld /home/dietpi/Umbra/build`. **Correction:** use the verified address and authorized SSH identity, install rsync on both sides, and prepare the destination with correct ownership. The team's `deploy-test.sh` hard-codes its private address and transfers only `Umbra-test`; new teams should follow [Development](DEVELOPMENT.md#subsequent-update).

### Hash mismatch or wrong path

**Symptoms:** host/Pi SHA-256 differs or service still launches an old binary. **Likely causes:** incomplete transfer, hashing different files, or service `ExecStart` points elsewhere. **Read-only checks:** run `sha256sum` on `build/arm64/Umbra` on the host and the staged `Umbra.candidate` on the Pi; inspect `systemctl cat umbra.service` and `systemctl show umbra.service -p ExecStart`. **Correction:** do not install a mismatched candidate; transfer it again, compare full hashes, then install only to the verified service path. See [Development](DEVELOPMENT.md#subsequent-update).

### Permission denied or shared library missing

**Symptoms:** systemd exit 203/EXEC, shell permission error, or loader error. **Likely causes:** missing executable bit, wrong owner/path, or target shared library not installed. **Read-only checks (Pi):** `ls -l /home/dietpi/Umbra/build/Umbra`, `file /home/dietpi/Umbra/build/Umbra`, `ldd /home/dietpi/Umbra/build/Umbra`, `sudo journalctl -u umbra.service -n 100 --no-pager`. **Correction:** use the [staged deployment procedure](DEVELOPMENT.md#subsequent-update), set execute permission on the staged file, and install matching target libraries. Do not replace production until the old executable is backed up.

## Runtime

### Service inactive or restart loop

**Symptoms:** `failed` state, repeated starts, or no process. **Likely causes:** missing absolute `config.json`, malformed JSON, library loader failure, wrong `ExecStart`, or camera/service user permissions. **Read-only checks (Pi):** `sudo systemctl status umbra.service --no-pager`, `systemctl cat umbra.service`, `sudo journalctl -u umbra.service -n 150 --no-pager`, and `ldd` on the actual executable. **Correction:** repair the specific missing prerequisite or roll back the binary using [Development](DEVELOPMENT.md#recovery-and-rollback); see [Operations](OPERATIONS.md#service-control).

### Camera missing or only one camera working

**Symptoms:** one stream absent, `open_failed`/`read_failed` diagnostics, or no per-camera publications. **Likely causes:** unstable `/dev/videoN` choice, disconnected USB port, permission, unsupported MJPEG mode, or another process owns the device. **Read-only checks (Pi):** `v4l2-ctl --list-devices`, `ls -l /dev/v4l/by-path/`, `v4l2-ctl -d /dev/video0 --list-formats-ext`, `id dietpi`, plus diagnostics for each configured name. **Correction:** select the right capture node, ensure user access, verify MJPEG 1280×800@100 support, and retest one camera at a time. See [Pi setup](PI_SETUP.md#camera-access-and-capture-mode) and [Configuration](CONFIGURATION.md#configuration-shape).

### Frames arrive but no AprilTags or poses

**Symptoms:** web video works but no fresh observations. **Likely causes:** wrong field layout or missing `field.json`, too few visible mapped tags, low decision margins, blur/exposure, wrong tag family/size, or failed solve. **Read-only checks (Pi):** inspect `/home/dietpi/Umbra/field.json` and camera diagnostics, preview tag outlines, and compare tag IDs against the map. **Correction:** provide the correct field map, improve lighting/focus/exposure, and show at least two accepted `tag36h11` tags; confirm camera calibration. See [Configuration](CONFIGURATION.md#field-layout) and [Operations](OPERATIONS.md#verify-localization).

### No or stale NetworkTables observations

**Symptoms:** service active but roboRIO sees no topic or an old array. **Likely causes:** wrong `team_number`, Pi/roboRIO network route, NT server unavailable, no valid multi-tag solves, or a subscriber that does not check age. **Read-only checks (Pi):** `ip route`, `date -Is`, `timedatectl status`, logs for publication and camera status; on robot, inspect NT connection, topic `/Umbra/<name>/observations`, array timestamp, and receive time. **Correction:** set the correct team/network, restore camera/tag solves, and reject stale values on the robot; don't treat a retained old array as a new measurement. See [Configuration](CONFIGURATION.md#networktables-observation-format) and [Operations](OPERATIONS.md#verify-localization).

### Incorrect position, heading, or config

**Symptoms:** mirrored/rotated poses, systematic offset, or unexpected exposure. **Likely causes:** wrong camera path, stale `config.json`, mismatched field layout, unmeasured mount, pixel calibration at another mode, degrees/radians confusion, or ignored tuning keys. **Read-only checks (Pi):** inspect the **absolute** `/home/dietpi/Umbra/config.json` and `field.json`, verify the service executable, compare camera calibration/mount with measurements, and check a known field pose. **Correction:** back up production JSON, correct measured values, restart for startup-only settings, and validate at several known poses. `tuning.decimate`/`threads`/`margin`/`latency_ms` are currently ignored. See [Configuration](CONFIGURATION.md#calibration-and-camera-mounts).
