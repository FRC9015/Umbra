# Operating Umbra on the Orange Pi

The team's service is named `umbra.service`, but **its installed unit is not in this repository**. The example below is a starting point for a new installation; always inspect the actual Pi before modifying a known-good service. Umbra's runtime JSON paths are hard-coded under `/home/dietpi/Umbra`.

## Identify the active executable

On the **Pi (Bash)**, before any update:

```bash
systemctl cat umbra.service
systemctl show umbra.service -p ExecStart -p MainPID
pgrep -af '/home/dietpi/Umbra'
sha256sum /home/dietpi/Umbra/build/Umbra \
  /home/dietpi/Umbra/Umbra-test 2>/dev/null
```

`systemctl cat` and `ExecStart` reveal the installed service's command. `pgrep` can show the live process command line. For a running PID, `readlink -f /proc/PID/exe` (replace `PID` with the real process ID) identifies its executable, including a `(deleted)` indicator after replacement. Hash the actual running/robot-tested file before deciding what to preserve. The historical test path `Umbra-test` is separate from production.

## Run manually

Manual runs can conflict with the service's USB camera access. On the **Pi (Bash)**, after intentionally stopping the service:

```bash
sudo systemctl stop umbra.service
cd /home/dietpi/Umbra
UMBRA_DIAGNOSTICS=1 ./build/Umbra
```

The working directory does **not** alter the absolute JSON paths. Stop the foreground run with Ctrl-C, then restart the service when ready. The executable exits if `config.json` cannot be opened; malformed JSON or missing runtime libraries can also prevent startup. `field.json` missing is less obvious: the loader returns without tags, causing no valid tag solves.

## Service control

On the **Pi (Bash)**:

```bash
sudo systemctl status umbra.service --no-pager
sudo journalctl -u umbra.service -n 100 --no-pager
sudo journalctl -u umbra.service -f
sudo systemctl start umbra.service
sudo systemctl stop umbra.service
sudo systemctl restart umbra.service
sudo systemctl enable umbra.service
```

Start/stop/restart and enable change the Pi; use them only during an authorized maintenance window. `enable` configures automatic startup; it does not by itself prove the vision pipeline works. A reboot with `sudo reboot` interrupts capture and robot communication; coordinate it with the robot operator, then check service and observations afterward.

## Example systemd unit

For a **new** Pi, after installing the binary and runtime libraries and preparing the JSON files, this is a proposed unit based on current source requirements. It is **not a copy of the team's installed production unit**:

```ini
[Unit]
Description=Umbra AprilTag vision
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=dietpi
WorkingDirectory=/home/dietpi/Umbra
Environment=UMBRA_DIAGNOSTICS=1
ExecStart=/home/dietpi/Umbra/build/Umbra
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
```

Install it on the **Pi (Bash)** as `/etc/systemd/system/umbra.service` using an editor, then `sudo systemctl daemon-reload` and `sudo systemctl enable --now umbra.service`. Check whether the `dietpi` user has camera permissions and whether all shared libraries resolve. Startup ordering after `network-online.target` does not guarantee the roboRIO NT server is already reachable; Umbra's NT client must connect when it becomes available. A service that is “active (running)” proves only that the process has not exited.

## Diagnostics and logs

`UMBRA_DIAGNOSTICS=1` enables periodic per-camera lines with capture/processing timing, processing rate, accepted/rejected frames, pose, last publication age, and camera open/read transitions. Use the example unit's `Environment` setting, or inspect and add an override to your actual unit after preserving it. On the **Pi (Bash)**:

```bash
sudo journalctl -u umbra.service -n 150 --no-pager
sudo journalctl -u umbra.service -f
```

Look for **each configured camera name**, `status=open` or camera error transitions, recent publication counts, and processing errors. Absence of diagnostics can mean the environment flag was not set, the service is using a different executable, or the process failed before its worker loops.

## Verify localization

1. Confirm the service is running and `ldd /home/dietpi/Umbra/build/Umbra` has no `not found` entries.
2. On the **Pi (Bash)**, check camera nodes with `v4l2-ctl --list-devices` and paths in `config.json`. Browse `http://PI_IP:5801/` from a **trusted network** and view **both** camera streams. The web server has no authentication/TLS.
3. With diagnostics enabled, verify both cameras read frames and that known field tags are accepted. Umbra needs at least **two** known tags with decision margin >50 for a publication.
4. On the roboRIO/NetworkTables client, observe `/Umbra/<name>/observations` for each camera. Check that arrays have nine finite values, timestamps advance, tag count is at least two, and observations become fresh after reconnection. Check robot-side NetworkTables connection and team number.
5. Place the robot at measured field positions and headings, compare output X/Y/yaw with reality, and test movement and lighting. Robot code should reject stale and implausible data before odometry fusion. The array's distance element currently has a known semantic defect; do not use it as tag range.

When the roboRIO disconnects, the process can remain active and continue camera processing. The code publishes through its NT client but does not implement robot-side reconnection validation or clear stale arrays. Confirm actual reconnect behavior and clock offset on the robot before relying on timestamps.

## Backups and recovery

Before replacing a robot-tested binary, record its hash and make a copy. The [development update procedure](DEVELOPMENT.md#subsequent-update) stages a candidate, compares SHA-256, preserves `Umbra.backup`, and documents rollback. Keep independent, secure backups of `config.json` and `field.json` before configuration changes; the executable update procedure does not overwrite them.
