# Development and deployment

This is the practical path from a source edit to a robot-verified build. **The checked-in deployment script only copies a test executable** to `/home/dietpi/Umbra/Umbra-test` on Team 9015's hard-coded IP. It does not install the running production executable or change `umbra.service`. The recorded production layout is:

```text
/home/dietpi/Umbra/build/Umbra
/home/dietpi/Umbra/config.json
/home/dietpi/Umbra/field.json
```

`main.cpp` reads both JSON files by these **absolute paths**. The installed service unit is not checked in; verify its actual `ExecStart` before any replacement. Commands here that alter a Pi are instructions for the operator, **not steps run during documentation work**. Use your team's IP and verify the target path on your own Pi.

## First deployment to a new Pi

Complete [Pi setup](PI_SETUP.md), [cross-compiling](CROSS_COMPILING.md), [testing](TESTING.md), and [configuration](CONFIGURATION.md). On the **Arch host (Fish)**, build and inspect:

```fish
cd ~/Documents/QE/Umbra
bash scripts/test-local.sh
podman run --rm -v "$PWD/..:/work" -w /work/Umbra umbra-cross \
  cmake --build build/arm64 --parallel 4
file build/arm64/Umbra
sha256sum build/arm64/Umbra
```

A new build directory requires the [configure command](CROSS_COMPILING.md#configure-and-build) first. Resolve unexpected test failures and inspect the known baseline failure separately. On the **Pi (Bash)**, create the runtime directory and write your **robot-specific** `config.json` and matching `field.json` there. The repository's JSON files are an example shape, not calibration for another robot. Make sure `dietpi` can read both and access cameras. Transfer the binary to a candidate path from the **Arch host (Fish)**:

```fish
set PI dietpi@192.0.2.10
ssh $PI 'mkdir -p /home/dietpi/Umbra/build'
rsync -av build/arm64/Umbra "$PI:/home/dietpi/Umbra/build/Umbra.candidate"
sha256sum build/arm64/Umbra
ssh $PI 'sha256sum /home/dietpi/Umbra/build/Umbra.candidate; file /home/dietpi/Umbra/build/Umbra.candidate'
```

Compare the full SHA-256 strings; they must match. For a first install, on the **Pi (Bash)**:

```bash
chmod 755 /home/dietpi/Umbra/build/Umbra.candidate
mv /home/dietpi/Umbra/build/Umbra.candidate /home/dietpi/Umbra/build/Umbra
ldd /home/dietpi/Umbra/build/Umbra
```

Resolve every `not found` library before starting. Install a service unit only after inspecting [the example and caveats](OPERATIONS.md#example-systemd-unit); `sudo systemctl daemon-reload` and `sudo systemctl enable --now umbra.service` are Pi changes. Verify localization as described below. Do not treat the example unit as the existing Team 9015 unit.

## Subsequent update

First identify what is **actually running** on the **Pi (Bash)**:

```bash
systemctl cat umbra.service
systemctl show umbra.service -p ExecStart -p MainPID
pgrep -af '/home/dietpi/Umbra'
ls -l /home/dietpi/Umbra/build/Umbra
sha256sum /home/dietpi/Umbra/build/Umbra
```

Proceed only if `ExecStart` resolves to the intended `/home/dietpi/Umbra/build/Umbra` and you have confirmed this is the known-good robot-tested executable. If it differs, adapt the procedure to the verified path; do not replace the wrong file. Do **not** copy the host's `config.json` or `field.json` onto a working Pi.

After editing `main.cpp`, `pose_math.cpp`, or another source, on the **Arch host (Fish)**:

```fish
cd ~/Documents/QE/Umbra
bash scripts/test-local.sh
podman run --rm -v "$PWD/..:/work" -w /work/Umbra umbra-cross \
  cmake --build build/arm64 --parallel 4
file build/arm64/Umbra
sha256sum build/arm64/Umbra
```

If the test suite still has the recorded `range.tag_distance` failure, confirm the other 42 cases remain passing and decide whether the changed code affects that defect. A compiler exit code and `file` result do not prove hardware behavior.

Back up the production executable **before stopping the service**. On the **Pi (Bash)**:

```bash
cp -a /home/dietpi/Umbra/build/Umbra \
  /home/dietpi/Umbra/build/Umbra.backup &&
  cmp -s /home/dietpi/Umbra/build/Umbra \
    /home/dietpi/Umbra/build/Umbra.backup &&
  sudo systemctl stop umbra.service
sha256sum /home/dietpi/Umbra/build/Umbra \
  /home/dietpi/Umbra/build/Umbra.backup
```

Confirm both hashes match. The fixed `Umbra.backup` name preserves one known-good version; use a dated backup policy if retaining several versions. On the **Arch host (Fish)**, stage the new binary without overwriting the production file:

```fish
set PI dietpi@192.0.2.10
rsync -av build/arm64/Umbra "$PI:/home/dietpi/Umbra/build/Umbra.candidate"
sha256sum build/arm64/Umbra
ssh $PI 'sha256sum /home/dietpi/Umbra/build/Umbra.candidate; file /home/dietpi/Umbra/build/Umbra.candidate'
```

If hashes differ, do not install the candidate; retry the transfer. Once they match, on the **Pi (Bash)**:

```bash
chmod 755 /home/dietpi/Umbra/build/Umbra.candidate
ldd /home/dietpi/Umbra/build/Umbra.candidate
mv /home/dietpi/Umbra/build/Umbra.candidate /home/dietpi/Umbra/build/Umbra
sudo systemctl start umbra.service
sudo systemctl status umbra.service --no-pager
sudo journalctl -u umbra.service -n 100 --no-pager
```

`ldd` must show no `not found` entry. The `mv` is an atomic rename **within the same filesystem**. The stop/start creates a deliberate outage. Inspect diagnostics for **both** cameras, see live frames at `http://PI_IP:5801/` on a trusted robot network, and verify fresh per-camera NetworkTables observations and plausible poses on the robot. See [Operations verification](OPERATIONS.md#verify-localization). If validation fails, roll back.

## Recovery and rollback

On the **Pi (Bash)**, first inspect the backup hash and service state. To restore the previously backed-up executable:

```bash
sha256sum /home/dietpi/Umbra/build/Umbra.backup
sudo systemctl stop umbra.service
cp -a /home/dietpi/Umbra/build/Umbra.backup \
  /home/dietpi/Umbra/build/Umbra.rollback-candidate &&
  mv /home/dietpi/Umbra/build/Umbra.rollback-candidate \
    /home/dietpi/Umbra/build/Umbra &&
  sudo systemctl start umbra.service
sudo systemctl status umbra.service --no-pager
sudo journalctl -u umbra.service -n 100 --no-pager
```

After restoring, verify camera and NT behavior again. The backup restores **only the executable**. If someone separately changed Pi JSON files or libraries, restore those from their own known-good backups. If the service cannot start because of target library changes, restore matching libraries/OS state or rebuild against the current Pi sysroot.

## Daily command cheat sheet

On the **Arch host (Fish)**, once configured:

```fish
cd ~/Documents/QE/Umbra
bash scripts/test-local.sh
ctest --test-dir build/native-tests -N
podman run --rm -v "$PWD/..:/work" -w /work/Umbra umbra-cross \
  cmake --build build/arm64 --parallel 4
file build/arm64/Umbra
```

Then perform the [staged update](#subsequent-update); no checked-in command safely automates a production replacement. The optional `bash scripts/deploy-test.sh` builds and rsyncs to Team 9015's **test** path using a hard-coded private address. Adapt the IP before another team's use; do not treat its successful transfer as deployment to the robot-tested service.
