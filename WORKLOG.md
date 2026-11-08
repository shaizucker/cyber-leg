# Worklog

## 2026-11-08 - Jetson deployment and Teensy rotary CAN migration

- Installed sparse `feature/hip` at `/home/halalit/cyber-leg` on Jetson 10.0.0.22;
  created `.venv` and installed python-can/pyserial.
- Verified CAD loading, manual controls and auto gait in a Windows browser.
  Added explicit LAN binding and matching Host/Origin/token checks; removed
  the browser's localhost-only connection gate.
- Replaced the rotary adapter with Teensy 4.1 + TJA1051T/3 CAN Pal-style module.
  RH02 remains on the separate ankle CAN-FD bus.
- Previously built mcba_usb for the Microchip analyzer. Its firmware reports
  USB 2.0 / CAN 2.3. Tests measured individual replies around 1.05 s and missing
  replies in a burst. Community firmware was downloaded/validated but not
  flashed because no PIC programmer was available. Latency cause was not conclusively isolated.
- Built/flashed Teensy V0101; corrected listen-only enum selection and added
  live error status/read-only diagnostics. Image: 59,388 bytes flash; RAM1
  83,072 bytes including code/padding, RAM2 12,416 bytes. USB serial 18818090,
  observed Windows COM11.
- Correcting TX/RX wiring restored bench traffic: PCAN received three 0x555
  frames; Teensy received 0x123 / AA BB CC at 1 Mbps.
- Added direct SLCAN bridge support: explicit persistent Teensy path, one shared
  rotary USB connection, 115200 CDC configuration and no open delay. This avoids
  the target kernel's missing CONFIG_CAN_SLCAN module.
- Powered rotary parameter reads verified knee ID 1 / hip ID 2, mode 2, knee
  -12..12 rad and hip approximately -30..30 degrees. A 200-query read-only test
  had no failures: knee mean/max 0.953/13.885 ms; hip mean/max 0.798/0.938 ms.
- Ankles 1/2/3 replied fault-free at 19.96/19.97/20.01 mm within 0..39 mm limits;
  earlier out-of-range faults were absent.
- Increased rotary reply budget from 10 to 30 ms, bounded within 10..50 ms.
  Kept missing-feedback/fault stops and the 500 ms simulator watchdog.
- Reproduced update starvation on Jetson with two fake 15 ms motor replies.
  A 1 ms overrun yield prevented immediate lock reacquisition; the reproduction
  stayed active, with maximum update wait decreasing from about 408 to 30 ms.
  No real hardware was armed by this test.
- Validation: 145 affected Python regression tests passed; seven timing tests
  passed on Jetson; firmware parser checks passed with ASan/UBSan. Earlier
  browser checks verified LAN connection and CAD/manual/auto simulation.
- Updated project status/spec and Jetson setup; created durable project context
  and this worklog for future chats.

Open follow-ups:
- Restart the foreground bridge after code deployment and refresh the browser;
  verify sustained full-leg motion after the scheduling fix.
- Commission physical references/persistence, torque/current caps and tracking
  under load. Read-only tests do not prove movement containment or hard real-time behavior.
- Add external DC sensors for measured electrical consumption.
