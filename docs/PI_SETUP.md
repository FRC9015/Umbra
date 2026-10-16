# Fresh Orange Pi setup

This guide targets an **Orange Pi 5 Ultra** running **ARM64 DietPi based on Debian 13 Trixie**, matching the working development target. The steps below are a documented path, **not a verified clean-room installation**. The repository does not contain the original WPILib ARM64 installation recipe, the installed service unit, or a lockfile for target libraries. Complete and verify those prerequisites before attempting production use.

## Image, flash, and first boot

1. On the **Arch host**, open [DietPi's supported hardware page](https://dietpi.com/docs/hardware/) and select **Orange Pi 5 Ultra**, then choose the ARMv8/Trixie image from the [official image index](https://dietpi.com/downloads/images/). Do not use an Orange Pi 5, 5 Plus, 5 Max, or a different Debian release by accident. DietPi image names and revisions may change.
2. Follow [DietPi's installation instructions](https://dietpi.com/docs/install/) to verify the download and flash it to supported storage. Imaging **erases the selected storage device**; identify it carefully. Use DietPi's current first-boot guidance for credentials and interactive setup rather than credentials printed in third-party guides.
3. Connect power, Ethernet if available, and a local display/keyboard or a network path. Complete DietPi first boot. Keep the Pi on a trusted network during setup.
4. Find its address from your router's DHCP lease list, the Pi console, or on the **Pi (Bash)**:

   ```bash
   hostname -I
   ip -br addr
   ```

5. On the **Arch host (Fish)**, replace the documentation IP with the actual Pi IP:

   ```fish
   set PI_IP 192.0.2.10
   ssh dietpi@$PI_IP
   ```

   `192.0.2.10` is an example address, not the team's private robot-network address. See [DietPi SSH guidance](https://dietpi.com/docs/software/ssh/). Umbra's target user and absolute runtime paths currently assume `dietpi`.

## Verify OS and update packages

On the **Pi (Bash)**:

```bash
uname -m
. /etc/os-release
printf '%s %s\n' "$ID" "$VERSION_CODENAME"
sudo apt update
sudo apt full-upgrade
```

Expect `aarch64` and Debian/DietPi Trixie. A full upgrade can update the kernel and libraries; follow DietPi's reboot prompt and refresh the sysroot afterward. The checked-in container is Debian Trixie, so mixing releases can cause ABI and linker failures.

Install candidate runtime and inspection packages on the **Pi (Bash)**:

```bash
sudo apt install rsync v4l-utils gstreamer1.0-tools \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-libav libopencv-dev libapriltag-dev \
  nlohmann-json3-dev
```

These package names are available from [Debian's package index](https://packages.debian.org/trixie/); installation on a fresh Orange Pi was **not executed here**. `v4l2-ctl` comes from `v4l-utils`. The GStreamer V4L2 and JPEG pipeline needs working plugins, and the runtime OpenCV build must support GStreamer. `nlohmann-json3-dev` supplies C++ headers/CMake metadata for the cross-build; it is not a separate runtime shared library. Check installed versions and library paths:

```bash
dpkg-query -W 'libopencv*' 'libapriltag*' 'nlohmann-json3-dev' 'gstreamer1.0-*'
pkg-config --modversion opencv4
gst-inspect-1.0 v4l2src jpegdec videoconvert appsink
ldconfig -p | grep -E 'lib(apriltag|opencv_core|ntcore|wpiutil|wpinet)'
```

The team's target uses OpenCV 4.10 and AprilTag 3. **WPILib `ntcore`, `wpiutil`, and `wpinet` shared libraries and headers are additionally required.** On the working Pi their libraries are in `/usr/local/lib`, but the repository does not document where the ARM64 builds came from, their version, or the installation procedure. Obtain a mutually compatible ARM64 WPILib set and verify headers, link names, transitive dependencies, and ABI before calling this a reproducible setup. Do not substitute x86-64 libraries. After installing libraries in `/usr/local/lib`, inspect `ldconfig -p`; whether `sudo ldconfig` or a loader configuration file is needed depends on the target image.

## Camera access and capture mode

Connect cameras one at a time. On the **Pi (Bash)**:

```bash
ls -l /dev/v4l/by-path/
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
id dietpi
ls -l /dev/video0
```

Select each camera's **video capture** node, ideally a stable `/dev/v4l/by-path/…-video-index0` entry. Device numbering can change across boots; compare the path with each physical USB port. The service user must be permitted to read the device; on typical Debian images this may mean membership in `video`. Check the actual device group first. If needed, on the **Pi (Bash)**:

```bash
sudo usermod -aG video dietpi
```

Log out and back in or restart the service for new group membership to apply. Avoid broad device permissions. Check that each camera lists **MJPEG 1280×800 at 100 fps**, because `main.cpp` requests that exact pipeline. Listing a mode does not prove stable capture. A direct probe on the **Pi (Bash)** is:

```bash
gst-launch-1.0 -v v4l2src device=/dev/video0 num-buffers=100 \
  ! 'image/jpeg,width=1280,height=800,framerate=100/1' \
  ! jpegdec ! videoconvert ! fakesink
```

Replace `/dev/video0` with the identified capture node. The command reads the camera and may conflict with Umbra if it is already running. Use `v4l2-ctl -d /dev/video0 --list-ctrls` to check whether `auto_exposure`, `exposure_time_absolute`, `gain`, and `brightness` exist; Umbra calls them at startup. Calibrate each camera at the **actual selected capture mode**; see [Configuration](CONFIGURATION.md#calibration-and-camera-mounts).

## Prepare runtime files and robot network

On the **Pi (Bash)**:

```bash
mkdir -p /home/dietpi/Umbra/build
ls -ld /home/dietpi/Umbra
```

The executable reads `/home/dietpi/Umbra/config.json` and `/home/dietpi/Umbra/field.json` regardless of its working directory. Create and edit these files using [Configuration](CONFIGURATION.md); do not blindly copy Team 9015's camera paths, robot offsets, or field geometry. If the files already exist on a working Pi, preserve them during code updates.

Connect the Pi to the robot network and confirm routing to the roboRIO. Umbra starts an NT4 client with `SetServerTeam(team_number)`; the configured team number must match the robot's NT server. On the **Pi (Bash)**, read-only checks include:

```bash
ip -br addr
ip route
date -Is
timedatectl status
```

Use the addressing and time-sync scheme required by your robot network. Umbra stamps observations from `nt::Now()` plus the NT server time offset when available; the robot must reject stale measurements and verify clock behavior at connection/reconnection. The repository does not define a complete robot-network or NTP configuration, so verify these in your deployment.

## Ready for deployment?

Confirm ARM64/Trixie, required libraries and loader paths, both cameras and controls, correct configuration and field files, reachability of the roboRIO, and sufficient user permissions. Then [create the sysroot and cross-build](CROSS_COMPILING.md), [deploy](DEVELOPMENT.md#first-deployment-to-a-new-pi), and [verify localization](OPERATIONS.md#verify-localization). The WPILib installation gap above is a real prerequisite; a new team must resolve it for its own target.
