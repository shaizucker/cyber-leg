# Jetson Orin Nano bridge and simulator

Current deployment, November 8, 2026: Jetson Orin Nano Super developer kit,
about 8 GB RAM, aarch64, Jetson Linux 36.5.0, kernel 5.15.185-tegra.
SSH access is `halalit@10.0.0.22`. Runtime checkout is `/home/halalit/cyber-leg`,
branch `feature/hip`. The bridge is started manually in the Jetson terminal.

## Current hardware

| Purpose | Adapter / transport | Persistent device path |
|---|---|---|
| Knee ID 1 and hip ID 2 | Teensy 4.1 + CAN Pal-style TJA1051T/3, direct SLCAN, classic CAN 1 Mbps | `/dev/serial/by-id/usb-Teensyduino_USB_Serial_18818090-if00` |
| Ankle IDs 1/2/3 | RH02 Plus, Seenpin serial CAN-FD+BRS 1 Mbps / 5 Mbps | `/dev/serial/by-id/usb-ElmueSoft__netcult.ch_elmue__Slcan_2.5_-_Jhoinrch_206B347745465015-if00` |

The observed Teensy/RH02 tty names are ttyACM1/ttyACM0 respectively; use by-id
paths because numbering can change. Account halalit has dialout access.
These are separate physical CAN buses. Native mttcan can0 is not used.

The Teensy firmware and module-specific wiring are documented in
[firmware/teensy_usb_can](../../firmware/teensy_usb_can/README.md). The user's
CAN Pal-style breakout takes 3.3 V at Vcc and has an onboard 5 V boost circuit;
the bare transceiver IC wiring is different. Firmware V0101 was flashed on
Windows COM11 and verified in both directions with PCAN before Jetson testing.

The target lacks CONFIG_CAN_SLCAN. Direct python-can SLCAN needs no slcand or
custom kernel driver. One shared rotary serial connection serves both motors;
CDC baud is configured as 115200, independently of the 1000000 CAN bitrate.
The backend skips the normal two-second serial-open delay for this Teensy.

## Clone and install

The deployment uses a sparse checkout to exclude the root ZIP, vendor Windows
binaries and unrelated project assets. For a new installation:

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone --filter=blob:none --depth 1 --single-branch \
  --branch feature/hip --no-checkout https://github.com/shaizucker/cyber-leg.git ~/cyber-leg
cd ~/cyber-leg
git sparse-checkout init --no-cone
git sparse-checkout set --no-cone '/drivers/' '/docs/' \
  '/CYBER_LEG_V01_Developer_Handoff/CYBER_LEG_V01/dist/' \
  '/firmware/teensy_usb_can/' '/.gitignore' '/.gitattributes' \
  '/PROJECT_SPEC.md' '/project.md' '/PROJECT_CONTEXT.md' '/WORKLOG.md'
git checkout feature/hip
python3 -m venv .venv
.venv/bin/python -m pip install -r drivers/requirements-bridge.txt
```

On the actual target, `python3 -m venv` failed because ensurepip was unavailable.
The successful non-root alternative was:

```bash
python3 -m pip install --user virtualenv
python3 -m virtualenv .venv
.venv/bin/python -m pip install -r drivers/requirements-bridge.txt
```

For updates, inspect `git status` first and preserve unrelated edits and local
`drivers/bridge_settings.json`. Then fetch/fast-forward the branch. Do not
overwrite that ignored calibration/profile file. The older transfer archive
under Windows `tmp/` is not the current distribution source; use Git.

## Launch on Jetson

```bash
cd ~/cyber-leg
.venv/bin/python -u drivers/knee_bridge.py --bind 10.0.0.22 \
  --can-interface slcan \
  --can-channel /dev/serial/by-id/usb-Teensyduino_USB_Serial_18818090-if00 \
  --ankle-port /dev/serial/by-id/usb-ElmueSoft__netcult.ch_elmue__Slcan_2.5_-_Jhoinrch_206B347745465015-if00
```

Startup is disarmed. Use `--only hip`, `--only knee` or `--only ankle` to select
part of the system; default is all outputs. No automatic arming occurs. Hardware
is opened by arming, reference/zero operations or an explicit hardware read.
Stop with Ctrl+C. Keep one bridge instance; port 8000 is exclusive.

The shell launcher still defaults to socketcan/can1. If using it with Teensy,
override both backend and channel, as well as the RH02 persistent path:

```bash
bash drivers/start_jetson_bridge.sh --bind 10.0.0.22 \
  --can-interface slcan \
  --can-channel /dev/serial/by-id/usb-Teensyduino_USB_Serial_18818090-if00 \
  --ankle-port /dev/serial/by-id/usb-ElmueSoft__netcult.ch_elmue__Slcan_2.5_-_Jhoinrch_206B347745465015-if00
```

Windows default transport remains pcan/PCAN_USBBUS1 and ankle COM7. Linux
defaults remain socketcan/can1 and ttyACM0 for compatibility; Teensy requires
explicit `--can-interface slcan --can-channel ...` to avoid opening the ankle port.

## Browser access

Open `http://10.0.0.22:8000/` from another Windows computer on the trusted LAN.
No Windows bridge or SSH tunnel is required. Jetson runs the bridge and serves
the files; the browser computes/renders the simulation and supplies fresh targets.
Jetson-local browser graphics performance has not been measured.

LAN control requires an explicit `--bind` IPv4 address. Wildcard binding is
rejected. Exact Host, Origin, JSON content-type and session-token checks remain
enabled. Refresh the page after restarting the bridge to obtain a new token.
The browser's previous localhost-only connection gate has been removed.

For optional localhost-only operation, omit --bind. Then use an SSH tunnel:

```powershell
ssh -N -L 8000:127.0.0.1:8000 halalit@10.0.0.22
```

Open `http://127.0.0.1:8000/` locally in that mode. The bridge and tunnel ports
must match for Host/Origin validation. Do not confuse the localhost and LAN modes.

## Coordinates, limits and timing

- Both rotary motors use P-T-M mode 2, default P/D 20/4. These gains do not cap torque.
- Latest motor readbacks: knee -12..12 rad; hip approximately -0.523599..+0.523599 rad.
  The explicit -12..12 rad servo setting is supported for either rotary motor.
- Motor parameter readbacks determine CAN encoding/decoding. Software target
  bounds remain separate. Knee simulator range is 0..105 degrees, with the
  legacy upper endpoint clamped to 1.83 rad before its 0.5 angle ratio.
- Knee target/velocity ratio 0.5: 50 simulator degrees requests 25 motor degrees.
  Hip ratio stays 1.0, simulator range -30..30 degrees. Signed zero/reference
  configuration may narrow or shift command limits; verify the actual pose.
- Nominal cycle is 20 ms / 50 Hz. Rotary reply deadline is 30 ms per joint,
  configurable using `--feedback-timeout-ms` within 10..50 ms.
- Missing feedback and motor faults still stop control; motion commands are not
  automatically retried. Simulator freshness limit remains 500 ms for all outputs.
- Between browser updates (auto mode, running) hip/knee targets are extrapolated
  along their velocity feed-forward: linear for 40 ms, then decaying with a 40 ms
  time constant, held after 250 ms, always clamped to the command limits
  (Leons-UI branch; see docs/UI/leons_ui.md).
- Each rotary joint rechecks freshness immediately before transmitting. After
  a cycle overrun, the worker yields 1 ms instead of immediately reacquiring the
  state lock, allowing queued HTTP target updates to run. Slow transactions can
  reduce actual cycle frequency; this is not hard real-time control.

## Verification and remaining commissioning

- PCAN bench tests verified Teensy TX and RX at 1 Mbps. Initial swapped TX/RX
  wiring caused bus-off; corrected wiring restored transfer.
- On Jetson, 200 read-only rotary parameter reads succeeded: knee mean/max
  0.953/13.885 ms; hip mean/max 0.798/0.938 ms. Teensy status F00.
- Latest ankle snapshot: IDs 1/2/3, fault 0, positions 19.96/19.97/20.01 mm,
  within configured 0..39 mm drive limits.
- Fake 15 ms replies reproduced watchdog starvation on Jetson. The overrun
  yield kept the same test active; maximum update wait fell from about 408 to 30 ms.
- 145 affected Python regressions passed; seven timing tests passed on Jetson.
  Firmware parser checks passed under ASan/UBSan. Browser tests verified CAD,
  manual/auto simulation and LAN bridge connection.
- User-operated motion was reported before the watchdog stop. Sustained full-leg
  motion after the latest scheduling fix, physical zero/persistence, load tuning,
  caps and travel containment remain to be verified.

Run hardware-free suites explicitly; some other repository tests move hardware:

```bash
.venv/bin/python -m unittest discover -s drivers -p test_rotary_timing.py
.venv/bin/python -m unittest discover -s drivers -p test_jetson_bridge.py
.venv/bin/python -m unittest discover -s drivers -p test_bridge_lan.py
.venv/bin/python -m unittest discover -s drivers -p test_output_selection.py
.venv/bin/python -m unittest discover -s drivers -p test_knee_bridge.py
.venv/bin/python -m unittest discover -s drivers -p test_hip_bridge.py
.venv/bin/python -m unittest discover -s drivers -p test_ankle_bridge.py
```

## Historical adapters

PEAK pcan was built in netdev mode and Microchip mcba_usb was built for kernel
5.15.185-tegra because the stock kernel omitted their drivers. Either appeared
as can1 when connected and required privileged bitrate/UP setup after reconnect.
They are not required by the current Teensy deployment. A kernel upgrade would
require rebuilding those out-of-tree modules if reusing those adapters.

Microchip USB 2.0 / CAN 2.3 firmware exhibited delayed/missing replies in this
setup. A patched firmware was prepared but not flashed; it needs a PIC programmer.
The precise source of the delay and benefit of that patch were not established.

## Online motor panel

See [online configuration](../Motors/online_configuration.md) for PID, limits,
references and per-device telemetry. Preserve ignored `drivers/bridge_settings.json`.
No external DC sensors are installed, so actual voltage/electrical consumption
remain unavailable; motor phase current is not DC input current. No calibrated
ankle torque conversion is available. Machine settings should be read back on
the actual hardware instead of inferred from historical documentation.
