# Cyber-Leg Prosthetic Control System - Project Status

**Last updated:** November 8, 2026

**Branch:** `feature/hip`
**Phase:** Integrated bench commissioning on Jetson; sustained motion validation remains open.

See [PROJECT_CONTEXT.md](PROJECT_CONTEXT.md) for durable configuration,
[WORKLOG.md](WORKLOG.md) for history, and [Jetson setup](docs/Jetson/setup.md)
for installation and launch commands.

## Current hardware architecture

| Subsystem | Hardware | Connection | Confirmed status |
|---|---|---|---|
| Host | Jetson Orin Nano Super developer kit, about 8 GB RAM | `halalit@10.0.0.22`, aarch64; Jetson Linux 36.5.0, kernel 5.15.185-tegra | SSH key access; repository and Python environment installed |
| Knee | Motorevo rotary motor ID 1 | Teensy 4.1 + TJA1051T/3 CAN Pal, classic CAN 1 Mbps | P-T-M mode 2; CAN Theta range -12..12 rad; parameter replies verified |
| Hip | Same rotary motor model, ID 2 | Same Teensy classic-CAN bus | P-T-M mode 2; reported range approximately -30..30 degrees |
| Ankle | Three Seenpin linear actuators, IDs 1/2/3 | Separate RH02 Plus USB serial CAN-FD adapter, 1 Mbps arbitration / 5 Mbps data with BRS | Fault-free replies at 19.96/19.97/20.01 mm; drive limits 0..39 mm |
| Browser | Windows computer on the LAN | `http://10.0.0.22:8000/` | Simulator loads directly from Jetson; no SSH tunnel required |

Ankle positions above are a dated commissioning snapshot, not a live state.
Jetson serves the application and runs the bridge; the browser computes/renders
the simulation and must continue sending fresh targets during control.

## USB adapter assignment

- Rotary: `/dev/serial/by-id/usb-Teensyduino_USB_Serial_18818090-if00`
  (observed `/dev/ttyACM1`); use `--can-interface slcan`.
- Ankle: `/dev/serial/by-id/usb-ElmueSoft__netcult.ch_elmue__Slcan_2.5_-_Jhoinrch_206B347745465015-if00`
  (observed `/dev/ttyACM0`); uses the existing Seenpin CAN-FD transport.
- Use persistent paths; ttyACM numbering may change after reconnecting.
- PCAN-USB is now a bench reference, not the deployed rotary adapter.
- Microchip 04d8:0a30 was tested but is not used for control. Its firmware
  reports USB 2.0 / CAN 2.3. Tests observed delayed/missing replies; no firmware
  update was performed because no PIC programmer was available.
- The Teensy path needs neither SocketCAN `can1` nor a new kernel driver.

## Firmware and transceiver

- Firmware: [Teensy USB-CAN](firmware/teensy_usb_can/README.md), PlatformIO
  environment `teensy41`, version response `V0101`.
- CAN3 uses Teensy 31 TX and 30 RX. The photographed CAN Pal-style module
  takes **3.3 V at Vcc**, has an onboard 5 V boost circuit and 3.3 V logic.
  This differs from bare-chip separate 5 V VCC / 3.3 V VIO wiring.
- Correcting swapped TX/RX wires restored bench traffic. PCAN received Teensy
  ID 0x555 frames; Teensy received PCAN ID 0x123 / AA BB CC at 1 Mbps.
- Firmware boots closed, flushes received frames promptly over USB, closes CAN
  on DTR disconnect, and rejects malformed frames and busy TX.
- Terminate at the two physical bus endpoints only. The CAN Pal switch adds
  120 ohms when ON. Keep rotary and ankle CAN buses separate.

## Control behavior

- Knee target and velocity mapping has a 0.5 motor-angle ratio: 50 simulator
  degrees requests 25 motor degrees from reference. Hip remains one-to-one,
  with simulator range -30..30 degrees.
- Actual motor Theta readbacks scale CAN encoding/decoding; software command
  limits remain separate from motor protection bounds.
- Online panel: rotary PID/limits, grouped ankle PID, individual ankle telemetry
  and current limits, signed reference capture. Machine-local profiles live in
  ignored `drivers/bridge_settings.json`; preserve them during deployment.
- No external power sensors are connected. Electrical consumption and rotary
  supply voltage remain unavailable; phase current is not DC bus current.
- Nominal cycle 20 ms / 50 Hz; rotary reply deadline 30 ms per joint,
  configurable within 10..50 ms. Faults and missing feedback stop control.
- Simulator freshness limit remains 500 ms. The rotary worker yields 1 ms
  after an overrun to let HTTP updates acquire the state lock. No automatic
  rearming or motion-command retries are added.
- Explicit LAN bind `--bind 10.0.0.22` retains exact Host/Origin/token checks.
  Startup is disarmed. Run in the Jetson terminal for live logs.

## Verification and remaining work

- 200 rotary parameter reads through Teensy succeeded: knee maximum 13.885 ms,
  hip maximum 0.938 ms. This does not guarantee motion-loop performance.
- Watchdog starvation was reproduced with fake motors on Jetson. After the
  overrun-yield fix, the same reproduction stayed active and maximum update
  wait fell from about 408 to 30 ms.
- 145 affected hardware-free regression tests passed. Firmware parser checks
  also passed with address/undefined-behavior sanitizers.
- User-operated motion was reported before the watchdog stop. Sustained
  full-leg tracking after the latest fix still needs physical verification.
- Physical zero/persistence, load tuning, torque/current caps and actual travel
  containment remain commissioning concerns. Old limits are not current readbacks.

## Historical bring-up facts

The October 18 record identified knee model LTE1.1AN1800G, a 36:1 reducer and
14 pole pairs. Initial PCAN telemetry/arming were verified; the main supply
was recorded as 48 V and the rotary CAN watchdog as 1000 ms. That early record
used Servo mode and different ranges. Current runtime uses P-T-M mode and the
readbacks above; the old 15 Nm limit and trial gains are not current settings.
