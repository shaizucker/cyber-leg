# Project Context

## Overview
- Branch `feature/hip`; Windows project `C:\cyber-leg`.
- Jetson sparse checkout `/home/halalit/cyber-leg` on the same branch.
- Host: Orin Nano Super, about 8 GB RAM, aarch64, Jetson Linux 36.5.0,
  kernel 5.15.185-tegra. Key-based SSH: `halalit@10.0.0.22`.
- Jetson runs the bridge/web server; the LAN browser computes/renders the simulation.

## Confirmed Build and Upload Commands
```powershell
cd C:\cyber-leg\firmware\teensy_usb_can
pio run -e teensy41
pio run -e teensy41 -t upload --upload-port COM11
```
- COM11 was the observed Teensy port; reidentify after reconnects.
- Verified firmware reply `V0101` to `V\r`. Exit code zero with "No Teensy
  boards were found" does not prove a successful flash.

Jetson foreground launch, initially disarmed:
```bash
cd ~/cyber-leg
.venv/bin/python -u drivers/knee_bridge.py --bind 10.0.0.22 \
  --can-interface slcan \
  --can-channel /dev/serial/by-id/usb-Teensyduino_USB_Serial_18818090-if00 \
  --ankle-port /dev/serial/by-id/usb-ElmueSoft__netcult.ch_elmue__Slcan_2.5_-_Jhoinrch_206B347745465015-if00
```
- Browser URL `http://10.0.0.22:8000/`; refresh after restart for a new token.
- Dependencies: `drivers/requirements-bridge.txt`. Target `.venv` was made with
  user-installed `virtualenv` because system ensurepip/python3-venv was missing.

## Board and Configuration Facts
- Rotary: Teensy 4.1 USB SLCAN, 1 Mbps classic CAN, knee ID 1 / hip ID 2.
- Teensy USB 16c0:0483, serial 18818090; observed ttyACM1.
- CAN3 pins 31 TX / 30 RX. User's CAN Pal-style module uses Vcc=3.3 V,
  an onboard 5 V boost circuit, common GND and SLNT low.
- RH02 USB 16d0:117e, serial 206B347745465015; observed ttyACM0. Separate
  ankle CAN-FD+BRS bus, 1M/5M, nodes 1/2/3.
- Persistent by-id paths above are authoritative; ttyACM numbers may change.
- Account has dialout access. Direct SLCAN needs no root/kernel module.
- Latest rotary readbacks: mode 2 on both; knee Theta -12..12 rad, hip
  approximately -0.523599..+0.523599 rad. Software targets remain separate.
- Knee angle/velocity ratio 0.5; hip ratio 1.0. Default P/D 20/4 is not a
  torque cap. Signed references can shift configured command limits.
- No external supply-voltage/current sensors installed.

## Folder Ownership and Architecture Notes
- `drivers/knee_bridge.py`: HTTP, targets, coordinated rotary control.
- `drivers/motorevo_driver.py`: shared rotary bus and protocol.
- `drivers/ankle_bridge.py`, `seenpin_driver.py`: independent ankle worker/transport.
- `drivers/motor_console.py`: online configuration and local saved profiles.
- `CYBER_LEG_V01_Developer_Handoff/CYBER_LEG_V01/dist`: browser runtime/CAD assets.
- `firmware/teensy_usb_can`: pinned firmware, protocol tests and wiring guide.
- `docs/Jetson/setup.md`: deployment procedure; `project.md`: current status.
- Ignore `tmp/`, `.pio/`, and `drivers/bridge_settings.json`; preserve local profiles.

## Recurring Bugs and Fixes
- Swapped TX/RX wiring caused failed bench tests; correcting it restored both
  directions against PCAN at 1 Mbps.
- USB disappearance/Windows error 31 required a reconnect; use a data cable.
- Removed the frontend localhost-only gate. Server still validates exact Host,
  Origin and session token for the explicit LAN address.
- Rotary reply deadline 30 ms per joint (CLI range 10..50 ms); nominal 20 ms
  cycles are not hard real-time when transactions overrun.
- Worker yields 1 ms after an overrun; immediate state-lock reacquisition had
  starved HTTP updates and triggered the simulator watchdog on Jetson.
- Each rotary joint checks current simulator freshness before transmitting.
- Faults, missing feedback and stale simulator updates over 500 ms still stop
  control. No automatic motion-command retries or automatic rearming.
- Cancelled browser connections must not trigger a second HTTP error response.

## Design Decisions
- Keep classic rotary CAN and ankle CAN-FD on separate USB adapters/buses.
- Teensy uses direct python-can SLCAN, an explicit channel and no CDC-open delay.
- Target lacks CONFIG_CAN_SLCAN; userspace SLCAN avoids a kernel rebuild.
- PCAN remains a bench analyzer. Microchip control was abandoned after delayed/
  missing replies; its firmware was not flashed without a physical PIC programmer.
- Run the bridge in the user's Jetson terminal for visible logs; do not silently
  substitute a background instance. Starting the bridge is distinct from arming.

## Current Known Limitations
- 200 read-only Teensy queries passed; maxima 13.885 ms knee / 0.938 ms hip.
  These are not motion-loop or physical containment guarantees.
- Last ankle snapshot: fault-free around 20 mm, limits 0..39 mm.
- Scheduling fix passed 145 fake-hardware tests and a Jetson reproduction;
  sustained full-leg tracking after the fix still needs physical verification.
- Zero/persistence, mechanical load tuning and torque/current caps require
  commissioning; historical motor parameters are not current readbacks.
- Electrical consumption needs external DC sensors. Ankle phase current is
  not DC input current, and ankle torque in Nm is unavailable.
- Operation requires fresh browser updates and a live network connection.
