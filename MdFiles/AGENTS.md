# Umbra — Codex Project Instructions

## Project

Umbra is an FRC vision application written in
C++20 for the Orange Pi 5 Ultra.

Dependencies:
- OpenCV 4.10
- AprilTag 3
- WPILib: ntcore, wpiutil, wpinet
- nlohmann_json 3.11.3

Repository: https://github.com/FRC9015/Umbra

## Development environment

Host: Arch Linux (x86-64), Fish shell

Repository:
~/Documents/QE/Umbra

ARM64 sysroot:
~/Documents/QE/umbra-sysroot

Target: Orange Pi 5 Ultra
OS: DietPi / Debian 13 Trixie (ARM64)
Target compiler: GCC 14.2.0
SSH: dietpi@10.90.15.11

## Build environment

Cross-compile locally using Podman.

Container image: umbra-cross
Container OS: Debian 13 Trixie
Container mount: ~/Documents/QE → /work

CMake toolchain:
cmake/aarch64-dietpi.cmake

CMake packages:
OpenCV_DIR=/work/umbra-sysroot/usr/lib/aarch64-linux-gnu/cmake/opencv4

nlohmann_json_DIR=/work/umbra-sysroot/usr/share/cmake/nlohmann_json

The sysroot contains the working Pi's ARM64
libraries and headers.

WPILib libraries are installed in /usr/local/lib
on the Pi.

BLAS/LAPACK symlinks were repaired using
rsync -L to copy the resolved ARM64 libraries.

## Deployment

Use scripts/deploy-test.sh when available.

Test executable on Pi:
/home/dietpi/Umbra/Umbra-test

Original production executable:
/home/dietpi/Umbra/build/Umbra

Production systemd service:
umbra.service

Prefer rsync for transfers because SFTP was
missing on the Pi.

## Rules

- Preserve working features.
- Review relevant code before editing.
- Keep changes focused and reviewable.
- Validate changes with the ARM64 build.
- Compilation does not prove hardware behavior.
- Never overwrite production without permission.
- Never stop/restart the Pi service without permission.
- Don't overwrite config.json or field.json.
- Never push commits without permission.
- Keep the sysroot and build outputs out of Git.
- Prefer Fish-compatible commands on the host.

## Code modification policy

Before making changes to existing code:

1. Analyze the relevant files.
2. Explain the issues found.
3. Propose a specific implementation plan.
4. Wait for explicit user approval before editing.

After approval:
- Edit the files directly.
- Build using the ARM64 Podman environment.
- Fix compilation errors.
- Present a summary of changes.
- Do not deploy, restart services, commit, or push
  without explicit permission.