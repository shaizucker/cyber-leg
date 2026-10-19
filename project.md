# Cyber-Leg Prosthetic Control System — Project Status

**Last Updated:** October 18, 2026  
**Workspace:** `C:\cyber-leg`  
**Phase:** Phase 1 — Baseline Actuator Bringup & Communication

---

## 1. Hardware Architecture Overview

| Subsystem | Actuator / Hardware | Interface | Bus Speed | Status |
| :--- | :--- | :--- | :--- | :--- |
| **Knee Joint** | Motorevo Rotary Actuator (`LTE1.1AN1800G`, 36:1 reducer) | CAN 2.0A | 1 Mbps | **Communication & Arming Verified** |
| **Ankle Joint** | 3x Linear Actuators (Seenpin) | CAN-FD | TBD | Pending Phase 1 integration |
| **Host / Controller** | PC / Workstation (Dev) $\rightarrow$ Raspberry Pi 5 / Jetson Orin Nano (Target) | PEAK-CAN USB $\rightarrow$ Native SocketCAN | 1 Mbps | **PCAN Driver operational** |
| **Power Bus** | 48V DC Main Power Supply | XT30PW(2+2) | 48V | **Online (Green PWR LED active)** |

---

## 2. Knee Actuator Diagnostics & Configuration

Direct register dump retrieved from motor non-volatile Flash via CAN SDO (`ID: 0x601`):

- **Motor CAN ID:** `1` (`0x001`)
- **Active Control Mode:** `Servo Mode` (3-loop controller: position PD, velocity PI, current FOC + sliding-mode observer)
- **Gear Ratio:** `36.0:1`
- **Pole Pairs (NPP):** `14`
- **Continuous Torque Limit:** `15.0 Nm`
- **Bus Protection Thresholds:** Overvoltage Lock = `80.0 V`, Undervoltage Lock = `10.0 V`
- **Scaling Ranges (Default):**
  - Position ($\theta$): `[-12.5, +12.5]` rad
  - Velocity ($V$): `[-10.0, +10.0]` rad/s
  - Torque ($\tau$): `[-50.0, +50.0]` Nm
- **Physical Characteristics:**
  - Rotor is free to backdrive by hand when unpowered or in Rest Mode.
  - Driver status indicators: Green LED = Power OK; Blue LED = Motor State active.

---

## 3. Communication & Driver Progress

### What Has Been Achieved
1. **Transition to Industry-Standard Adapter:**
   - Switched from the proprietary USB-UART "Jason" dongle to the **PEAK-CAN (PCAN-USB)** adapter.
   - Driver installed and operational on `PCAN_USBBUS1` at 1 Mbps.
2. **Protocol Validation:**
   - Standard 11-bit CAN frame decoding and encoding verified against the Motorevo V3.5 driver specification.
   - Real-time telemetry extraction (status, position, velocity, torque, fault code, PCB temperature) confirmed working.
3. **Motor Arming & Torque Generation Verified:**
   - Sent `Enter Motor State` (`0xFC`) command $\rightarrow$ driver successfully transitioned to **`Status: 0x01` (Motor State)**.
   - Closed-loop holding torque generation confirmed (`+0.65 Nm`).
4. **Watchdog Characterization:**
   - Identified driver watchdog mechanism: if CAN command frames stop for more than `1000 ms`, the motor sets `Fault 16` (`0x10`, CAN Communication Interruption) and falls back to safe `Rest State`.
   - Control loops must stream command frames continuously at 50 Hz (20 ms interval) during active motion.

---

## 4. Software Assets in `c:\cyber-leg`

- **[`drivers/motorevo_driver.py`](file:///c:/cyber-leg/drivers/motorevo_driver.py)**:
  - Reusable, object-oriented Python driver (`MotorevoMotor`).
  - Supports PEAK-CAN (`interface="pcan"`) on Windows.
  - Ready for drop-in SocketCAN (`interface="socketcan"`, `channel="can0"`) deployment on Linux (Raspberry Pi 5 / Jetson Orin Nano).
  - Implements state transitions (`enter_motor_mode`, `enter_rest_mode`, `set_zero_position`), Servo Mode commanding, and MIT Cheetah style P-T-M commanding.
- **[`drivers/knee_control_cli.py`](file:///c:/cyber-leg/drivers/knee_control_cli.py)**:
  - Interactive terminal CLI for live motor status reading, zeroing, arming, nudging, and safe disarming.
- **[`drivers/test_pcan_motor.py`](file:///c:/cyber-leg/drivers/test_pcan_motor.py)**:
  - Bus discovery and ping utility.
- **[`drivers/test_rotation_45deg.py`](file:///c:/cyber-leg/drivers/test_rotation_45deg.py)**:
  - Smooth S-curve trajectory executor for verified rotation tests.

---

## 5. Next Steps

1. **Knee Joint Motion Tuning:**
   - Execute continuous 50 Hz position trajectory with tuned gains ($K_p \approx 40\text{–}60$, $K_d \approx 5\text{–}8$) to smoothly rotate through the 36:1 reducer.
2. **Phase 1 Completion (Knee):**
   - Establish baseline automated position sweep and angle repeatability test scripts.
3. **Ankle Joint Integration:**
   - Review Seenpin linear actuator documentation (`docs/Ankle/Seenpin V2.0 CAN-FD User Manual.docx`).
   - Configure CAN-FD interface and verify communication with the 3 linear actuators.
4. **GUI Development:**
   - Build a lightweight unified GUI for simultaneously monitoring the knee angle and the 3 ankle linear actuators.
