# Motorevo DC Motor Controller & Agent Control Specification

**Document Version:** 3.5  
**Firmware Compatibility:** ≥ 240925  
**Studio Software:** Motorevo Studio v3.5.3 / MParameter Studio  

---

## 1. System Overview & Hardware Architecture

### 1.1 Electrical Characteristics
* **Main Power Voltage:** 12V – 95V DC (Typical: 24V / 48V / 72V)
* **Input Current:** Peak 10A, Continuous 4A
* **PWM Switching Frequency:** 20 kHz (19.8 – 20.2 kHz)
* **Single-turn/Multi-turn Encoder Resolution:** 16 Bits / 16 Bits
* **CAN Baud Rate:** 1 Mbps (1 MHz)
* **Brake Control:** 24V actuation / 7V hold

### 1.2 Hardware Interfaces & Pinouts
* **Main Power / CAN Port:** XT30PW(2+2) or XT30PB(2+2)
  * `VCC`: Main DC Supply
  * `GND`: Power Ground
  * `CAN_H`: CAN High Signal
  * `CAN_L`: CAN Low Signal
* **USB Interface:** MX1.25-4P (`VCC`, `D-`, `D+`, `GND`)
* **Multi-turn Encoder Battery Interface:** MX1.25-2P (`BAT+`, `BAT-` [3.3V–5V])
* **Safe Torque Off (STO):** MX1.25-3P (`STO1`, `STO2`, `GND_STO`)

---

## 2. Power-On Diagnostics, Initialization, and STO

### 2.1 Boot Sequence & Self-Check Audio Cues
1. **Power-On:** System sounds **2 short beeps**.
2. **Hardware Self-Check:**
   * **Pass:** System sounds **3 short beeps**. Motor enters `Rest State`.
   * **Fail:** Motor sounds **4 diagnostic beeps** (long/short combinations), flashes blue LED, and sets an un-clearable fault code ($>128$). Motor will **not** enter `Motor State`.

#### Self-Check Error Codes Table
| LED / Buzzer Pattern | Error Classification | Recovery Action |
| :--- | :--- | :--- |
| **Long-Short-Short-Short** | CAN Transceiver Hardware Fault | Factory Repair |
| **Long-Long-Short-Short** | Illegal Firmware | Reflash / Factory Repair |
| **Short-Long-Short-Short** | Angle Sensor Fault | Check mechanical mounting, screws, magnetic ring |
| **Short-Short-Long-Short** | Driver Hardware Fault | Inspect MOSFETs for physical/thermal damage |
| **Short-Short-Short-Long** | Power Stage Failure | Inspect MOSFETs and 3-phase soldering |
| **Long-Long-Long-Short** | EtherCAT Communication Fault | Check EtherCAT daughterboard connection |

### 2.2 Safe Torque Off (STO) Truth Table
| STO1 (12–24V) | STO2 (12–24V) | LED1 | LED2 | Motor Output State |
| :---: | :---: | :---: | :---: | :--- |
| **ON** | **ON** | ON | ON | **Torque Enabled** |
| **OFF** | **ON** | OFF | ON | Torque Disabled (Hardware Cutoff) |
| **ON** | **OFF** | ON | OFF | Torque Disabled (Hardware Cutoff) |
| **OFF** | **OFF** | OFF | OFF | Torque Disabled (Hardware Cutoff) |

---

## 3. Operational State Machine

```
      +-------------------------------------------+
      |               24V Power On                |
      +-------------------------------------------+
                            |
                            v
      +-------------------------------------------+
      |            Hardware Self-Check            |
      +-------------------------------------------+
             |                             |
      (Self-Check Pass)             (Self-Check Fail)
             |                             |
             v                             v
   +-------------------+         +-------------------+
   |    Rest State     |         | Fault Lock / Blue |
   |  (Low Power 0.02A)|         |   LED Flashing    |
   +-------------------+         +-------------------+
        |          ^
  Enter |          | Fault / Low Volts /
  Motor |          | Reset Cmd
        v          |
   +-------------------+
   |    Motor State    |
   | (Active Control)  |
   +-------------------+
```

### 3.1 State Switching via CAN

All state commands are **CAN Standard Frames** with **Baudrate = 1 Mbps**.
* **CAN ID:** Set to target `Motor_ID` (Default in Torque4 mode = 100).

#### State Command Packets
* **Enter Motor State:**
  `[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]`
* **Enter Rest State (Clears recoverable fault codes $\le 128$):**
  `[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]`
* **Set Current Position as Zero Position:**
  `[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]`

---

## 4. Control Modes & Data Frame Formats

### Data Conversion Helper Equations
Convert standard floating-point physical units into unsigned integers for CAN transmission:

$$U_{uint} = \left\lfloor \frac{X_{float} - X_{min}}{X_{max} - X_{min}} \times (2^N - 1) \right\rfloor$$

$$X_{float} = \frac{U_{uint}}{2^N - 1} \times (X_{max} - X_{min}) + X_{min}$$

Where $N$ is the number of bits allocated to the parameter.

---

### 4.1 Servo Mode (Control Mode = 0x01)
* **Description:** 3-loop control (Current, Velocity, Position) with Non-linear Sliding Mode Observer (ULSMO) and Feedforward Torque.
* **Frame ID:** `Motor_ID` | **DLC:** 8 Bytes

| Byte | Field | Bits | Mapping Range (Default) | Unit |
| :--- | :--- | :--- | :--- | :--- |
| **Byte 0–1** | `Theta_ref` (High byte first) | 16 | $[-12.5, +12.5]$ | rad |
| **Byte 2** | `V_ref` | 8 | $[0.0, 10.0]$ | rad/s |
| **Byte 3** | Position $K_p$ | 8 | $[0.0, 250.0]$ | - |
| **Byte 4** | Position $K_d$ | 8 | $[0.0, 50.0]$ | - |
| **Byte 5** | Velocity $K_p$ | 8 | $[0.0, 250.0]$ | - |
| **Byte 6** | Velocity $K_d$ | 8 | $[0.0, 50.0]$ | - |
| **Byte 7** | Velocity $K_i$ | 8 | $[0.0, 0.05]$ | - |

---

### 4.2 Position-Torque Mixed Control Mode (P-T-M Mode, Control Mode = 0x02)
* **Description:** Dual-loop control (Current & Position PD with Torque Feedforward).
* **Frame ID:** `Motor_ID` | **DLC:** 8 Bytes

#### Bit Bit-Packing Mapping:
* **Byte 0–1:** `Theta_ref` (16 bits) $\rightarrow [-12.5, +12.5]$ rad
* **Byte 2 & Byte 3 [Bits 7:4]:** `V_ref` (12 bits) $\rightarrow [-10.0, +10.0]$ rad/s
* **Byte 3 [Bits 3:0] & Byte 4:** Position $K_p$ (12 bits) $\rightarrow [0.0, 250.0]$
* **Byte 5 & Byte 6 [Bits 7:4]:** Position $K_d$ (12 bits) $\rightarrow [0.0, 50.0]$
* **Byte 6 [Bits 3:0] & Byte 7:** `T_ref` (12 bits) $\rightarrow [-50.0, +50.0]$ N·m

---

### 4.3 Velocity Mode (Control Mode = 0x03)
* **Description:** Dual-loop control (Current & Velocity PI) with ULSMO disturbance rejection.
* **Frame ID:** `Motor_ID` | **DLC:** 8 Bytes

#### Bit-Packing Mapping:
* **Byte 0–1:** `V_ref` (16 bits) $\rightarrow [-10.0, +10.0]$ rad/s
* **Byte 2 & Byte 3 [Bits 7:4]:** Velocity $K_p$ (12 bits) $\rightarrow [0.0, 250.0]$
* **Byte 3 [Bits 3:0] & Byte 4:** Velocity $K_d$ (12 bits) $\rightarrow [0.0, 50.0]$
* **Byte 5 & Byte 6 [Bits 7:4]:** Velocity $K_i$ (12 bits) $\rightarrow [0.0, 0.05]$
* **Byte 6 [Bits 3:0]:** Reserved / Unused
* **Byte 7:** Identification Byte = `0xAC`

---

### 4.4 Torque Mode (Control Mode = 0x04)
* **Description:** Direct Quadrature Current ($I_q$) torque command control.
* **Frame ID:** `Motor_ID` | **DLC:** 8 Bytes

| Byte | Field | Bits | Mapping Range (Default) | Unit |
| :--- | :--- | :--- | :--- | :--- |
| **Byte 0–1** | `T_ref` (High byte first) | 16 | $[-50.0, +50.0]$ | N·m |
| **Byte 2–6** | Unused / Reserved | - | - | - |
| **Byte 7** | Mode Identifier | 8 | Fixed value: `0xAB` | - |

---

### 4.5 Torque4 Mode (Control Mode = 0x05)
* **Description:** Controls up to 4 actuators in a single CAN frame for maximum bus throughput.
* **Frame ID:** `100` (Decimal) | **DLC:** 8 Bytes

| Byte Pairing | Target Actuator Logic | Target Motor Array Example |
| :--- | :--- | :--- |
| **Byte 0–1** | `(Motor_ID - 1) % 4 == 0` | Motor ID 1, 5, 9, ... |
| **Byte 2–3** | `(Motor_ID - 1) % 4 == 1` | Motor ID 2, 6, 10, ... |
| **Byte 4–5** | `(Motor_ID - 1) % 4 == 2` | Motor ID 3, 7, 11, ... |
| **Byte 6–7** | `(Motor_ID - 1) % 4 == 3` | Motor ID 4, 8, 12, ... |

*Each torque payload is a 16-bit mapped uint corresponding to $[-50.0, +50.0]$ N·m.*

---

## 5. Feedback Telemetry Decoding

Every valid received frame prompts an automatic reply frame from the motor.
* **Response Frame ID:** `Motor_ID`
* **Baud Rate:** 1 Mbps | **DLC:** 8 Bytes

### Feedback Frame Payload Table
| Byte | Field | Format | Mapping / Meaning |
| :--- | :--- | :--- | :--- |
| **Byte 0** | **Status Word** | uint8 | `0x00`: Rest State<br>`0x01`: Motor State (Servo Mode)<br>`0x02`: Motor State (P-T-M Mode)<br>`0x03`: Motor State (Velocity Mode)<br>`0x04`: Motor State (Torque Mode)<br>`0x05`: Motor State (Torque4 Mode) |
| **Byte 1–2** | **Actual Position** | uint16 | $[0\text{x0000}, 0\text{xFFFF}] \rightarrow [-12.5, +12.5]$ rad |
| **Byte 3 & 4[7:4]** | **Actual Velocity** | uint12 | $[0\text{x000}, 0\text{xFFF}] \rightarrow [-10.0, +10.0]$ rad/s |
| **Byte 4[3:0] & 5** | **Actual Torque** | uint12 | $[0\text{x000}, 0\text{xFFF}] \rightarrow [-50.0, +50.0]$ N·m |
| **Byte 6** | **Fault Code** | uint8 | Error identification (See Section 6) |
| **Byte 7** | **Board Temp** | uint8 | Temperature in °C (Unsigned Char) |

---

## 6. Fault Diagnosis & Error Handling

### 6.1 Diagnostic Codes Table

| Code (Dec) | Code (Hex) | Description | Root Cause | Agent Action |
| :---: | :---: | :--- | :--- | :--- |
| **0** | `0x00` | No Fault | Normal Operation | None |
| **1** | `0x01` | Bus Overvoltage | Regeneration/Power Spikes | Add bus capacitors or adjust voltage threshold |
| **2** | `0x02` | Bus Undervoltage | Supply voltage drop/sag | Check supply capabilities, add capacitors |
| **3** | `0x03` | Electrical Angle Fault | Encoder offset invalid | Recalibrate motor electrical angle |
| **4** | `0x04` | DRV Driver Fault | Overload / Driver hardware failure | Restart. If persistent, hardware maintenance needed |
| **5** | `0x05` | Bus Overcurrent | Excessive mechanical load | Reduce load or adjust current protection limits |
| **6** | `0x06` | Phase A Overcurrent | Excessive mechanical load | Reduce load / check for short circuits |
| **7** | `0x07` | Phase B Overcurrent | Excessive mechanical load | Reduce load / check for short circuits |
| **8** | `0x08` | Phase C Overcurrent | Excessive mechanical load | Reduce load / check for short circuits |
| **9** | `0x09` | Driver PCB Over-temp | Thermal protection triggered | Reduce load, add active cooling (fan) |
| **10** | `0x0A` | Coil Over-temp | Motor thermal protection triggered | Allow motor to cool; add active cooling |
| **11** | `0x0B` | Encoder Error | Dirt, foreign particles, offset drift | Inspect magnetic disc, clean, recalibrate |
| **12** | `0x0C` | Current Sensor Fault | Hardware shunt damaged | Inspect board for debris or blown shunts |
| **13** | `0x0D` | Out of Position Range | Actual position $> |CAN COM MAX/MIN|$ | Move output shaft manually back into range |
| **14** | `0x0E` | Out of Velocity Range | Actual velocity $> |CAN COM MAX/MIN|$ | Verify target velocity limits |
| **15** | `0x0F` | Stall Protection | $I_q > \text{Stuck Current}$ & $v < \text{Stuck Vel}$ | Clear mechanical blockage, verify angle offset |
| **16** | `0x10` | CAN Timeout Fault | No CAN frames received $> \text{Timeout}$ | Verify bus wiring and master send rate |
| **129** | `0x81` | Encoder Header Check Fail | Un-clearable self-check fault | Check multi-turn encoder chip / factory repair |
| **130–135**| `0x82–0x87`| Multi-turn Encoder Errors | Internal multi-turn hardware issue | Re-initialize multi-turn encoder via USB |
| **136** | `0x88` | Firmware Error | Firmware hash mismatch | Reflash official firmware |
| **137** | `0x89` | Gate Driver Init Failure | Hardware gate driver destroyed | Hardware maintenance needed |

---

## 7. Parameter Management & On-The-Fly Flash Configuration

### 7.1 Parameter Index Map (Register Table)

| Index (Dec) | Parameter Name | Data Type | Unit | Description / Default |
| :---: | :--- | :---: | :---: | :--- |
| **10** | Firmware Version | Int32 | - | Read-only firmware version string |
| **11** | Control Mode | Int32 | - | 1: Servo, 2: P-T-M, 3: Velocity, 4: Torque |
| **12** | $I_d$ Controller $K_p$ | Float32 | V/A | FOC Direct axis $K_p$ proportional gain |
| **13** | $I_d$ Controller $K_i$ | Float32 | Hz | FOC Direct axis $K_i$ integral gain |
| **14** | $I_q$ Controller $K_p$ | Float32 | V/A | FOC Quadrature axis $K_p$ proportional gain |
| **15** | $I_q$ Controller $K_i$ | Float32 | Hz | FOC Quadrature axis $K_i$ integral gain |
| **16** | Current DeadZone | Float32 | A | Current deadband threshold |
| **17** | Velocity DeadZone | Float32 | rad/s | Velocity deadband threshold |
| **18** | Position DeadZone | Float32 | rad | Position deadband threshold |
| **19** | Current Integral Limit | Float32 | Ratio | Limits current integration max output |
| **20** | Velocity Integral Limit | Float32 | N·m | Velocity integration torque clamp |
| **21** | Torque Limit | Float32 | N·m | Maximum permissible driver output torque |
| **22** | Electric Angle Offset | Float32 | rad | Calibration offset (**Do not modify manually**) |
| **23** | Machine Angle Offset | Float32 | rad | Zero offset: Actual Output = Measured - Offset |
| **24** | CAN COM Theta MIN | Float32 | rad | Minimum mapped CAN position value (Def: -12.5) |
| **25** | CAN COM Theta MAX | Float32 | rad | Maximum mapped CAN position value (Def: +12.5) |
| **26** | CAN COM Velocity MIN | Float32 | rad/s | Minimum mapped CAN velocity value (Def: -10.0) |
| **27** | CAN COM Velocity MAX | Float32 | rad/s | Maximum mapped CAN velocity value (Def: +10.0) |
| **28–29** | CAN COM $K_p$ MIN/MAX | Float32 | N·m/rad | Mapped position loop gain range (0.0 to 250.0) |
| **30–31** | CAN COM $K_d$ MIN/MAX | Float32 | - | Mapped damping gain range (0.0 to 50.0) |
| **32–33** | CAN COM $K_i$ MIN/MAX | Float32 | - | Mapped integral gain range (0.0 to 0.05) |
| **34–35** | CAN COM Torque MIN/MAX| Float32 | N·m | Mapped CAN torque limits (-50.0 to 50.0) |
| **36** | Motor ID | Int32 | - | CAN node ID (0–1024) |
| **37** | CAN Timeout | Int32 | ms | Timeout threshold for CAN lost error (Code 16) |
| **38–39** | Default Pos $K_p$ / $K_d$ | Float32 | - | EtherCAT CSP mode default position PD gains |
| **40–41** | Default Vel $K_p$ / $K_i$ | Float32 | - | EtherCAT CSV mode default velocity PI gains |
| **42** | Max Acceleration | Float32 | rad/s² | Trajectory profiling acceleration limit |
| **43** | Torque Slope | Float32 | N·m/s | Torque rate-of-change limit |
| **44** | Number of Pole Pairs (NPP)| Int32 | - | Motor pole pair count (**Critical**) |
| **45** | Gear Ratio | Float32 | - | Gearbox reduction ratio |
| **46** | Torque Constant ($K_t$) | Float32 | N·m/A | $T = I_q \times K_t$ |
| **47** | Rotation Direction | Int32 | - | Direction sign multiplier: `+1` or `-1` |
| **49** | Bus Overvoltage Lock | Float32 | V | Overvoltage cutoff threshold |
| **50** | Bus Undervoltage Lock | Float32 | V | Undervoltage cutoff threshold |
| **51** | Phase Overcurrent Lock | Float32 | A | Peak phase current cutoff threshold |
| **52** | Board Temp Lock | Float32 | °C | Thermal shutdown threshold |
| **53** | Stuck Current | Float32 | A | Armature current threshold for stall lock |
| **54** | Stuck Velocity | Float32 | rad/s | Max velocity threshold for stall lock |
| **55** | Stuck Time | Float32 | s | Time duration threshold for stall lock |
| **56** | Protection Switch Mask | Int32 | Bitmask | Bit 0: Position Limit, Bit 1: Overspeed, Bit 2: Phase OC, Bit 3: Temp, Bit 4: Voltage, Bit 5: Stall, Bit 6: Encoder Check, Bit 7: CAN Timeout. Set `0xFFFFFFFF` for all ON. |

---

### 7.2 CAN Parameter Read/Write Protocol Specification
Master sends standard frames to ID = `0x600 + Motor_ID`.

#### Read/Write Master Request Frame Format
| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| `0x67` | `Index` | `Data0` | `Data1` | `Data2` | `Data3` | `Control` | `0x76` |

* **Header/Tail:** Byte 0 = `0x67`, Byte 7 = `0x76`
* **Control Byte (Byte 6):**
  * Read Parameter: `0x04` (Bytes 2–5 are ignored)
  * Write Parameter: `0x15` (Bytes 2–5 contain IEEE 754 32-bit float or Int32, Little-Endian)
* **System Action Commands (Index-based, Bytes 2–6 = `0x00`):**
  * **Save Settings to Flash:** Index = `0x00`
  * **Factory Reset:** Index = `0x02`
  * **Calibrate Electrical Angle:** Index = `0x03`

#### Motor Response Parameter Frame Format
* **CAN ID:** `0x600 + Motor_ID`

| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| `Motor_ID` | `Index` | `Data0` | `Data1` | `Data2` | `Data3` | `0x00` | `0xFF` |

---

## 8. EtherCAT Protocol Reference

### 8.1 Mode Mapping
Set CoE Object `0x6060` (Mode of Operation):
* `0x08`: **CSP** (Cyclic Synchronous Position Mode $\rightarrow$ Servo Mode)
* `0x09`: **CSV** (Cyclic Synchronous Velocity Mode $\rightarrow$ Velocity Mode)
* `0x0A`: **CST** (Cyclic Synchronous Torque Mode $\rightarrow$ Torque Mode)

### 8.2 Operational Control Loop
1. **Set Mode:** Write target value to Object `0x6060`.
2. **Enable Driver:** Write Control Word `0x6040 = 15` (`0x0F`) to switch from `Rest State` to `Motor State` (Brake releases).
3. **Write Target Value:**
   * **Target Position (`0x607A`):** Unit = $0.0001$ rad (e.g., $1.0\text{ rad} \rightarrow 10000$)
   * **Target Velocity (`0x60FF`):** Unit = $0.0001$ rad/s (e.g., $1.0\text{ rad/s} \rightarrow 10000$)
   * **Target Torque (`0x6071`):** Unit = $0.0001$ N·m (e.g., $5.0\text{ N·m} \rightarrow 50000$)
4. **Disable Driver:** Write Control Word `0x6040 = 7` (`0x07`) to set motor into `Rest State` (Brake engages).

---

## 9. Operating with Motorevo Studio v3.5.3

### 9.1 Hardware Setup with Jason USB-to-CAN Converter
1. Install CP2102 drivers on host PC.
2. Connect PC via USB Type-C to the **Jason USB-CAN converter**.
3. Wire Jason `CAN_H` and `CAN_L` to the actuator's XT30PW connector.
4. Power the motor with a 24V–95V supply.

### 9.2 Software Workflow Guide
1. **Establish Connection:**
   * Open Motorevo Studio v3.5.3.
   * Select `COMx (Silicon Labs CP210x)` from dropdown.
   * Click **CONNECT** (Button background turns green).
2. **Synchronize Range Parameters:**
   * Set lower/upper bounds in `Set Parameter Range` panel matching the motor Flash configuration to avoid command truncation.
3. **Set Zero Position & Motor Enable:**
   * Click **Zero Position (Z)** button to register current angle as mechanical zero reference.
   * Click **Motor Mode (M)** to transition driver from `Rest State` into `Motor State`.
4. **Parameter Configuration via MParameter Studio:**
   * Navigate to `MParameter Studio`.
   * Click **Read Parameter**.
   * Modify parameters, hit **ENTER** on the keyboard for each altered field.
   * Click **Save Parameter** and wait 2 seconds for Flash write completion.

---

## 10. Programmatic Python Control API

```python
import struct
import can

class MotorevoController:
    """
    Python CAN interface driver for Motorevo Series Servo Actuators (Firmware >= 240925).
    """
    
    # Range Defaults
    P_MIN, P_MAX = -12.5, 12.5
    V_MIN, V_MAX = -10.0, 10.0
    KP_MIN, KP_MAX = 0.0, 250.0
    KD_MIN, KD_MAX = 0.0, 50.0
    KI_MIN, KI_MAX = 0.0, 0.05
    T_MIN, T_MAX = -50.0, 50.0

    def __init__(self, channel='can0', bustype='socketcan', motor_id=1):
        self.bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=1000000)
        self.motor_id = motor_id

    @staticmethod
    def float_to_uint(val, v_min, v_max, bits):
        val = max(min(val, v_max), v_min)
        span = v_max - v_min
        return int(((val - v_min) / span) * ((1 << bits) - 1))

    @staticmethod
    def uint_to_float(val, v_min, v_max, bits):
        span = v_max - v_min
        return (float(val) / ((1 << bits) - 1)) * span + v_min

    def enter_motor_state(self):
        """Transition driver to active Motor State."""
        data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
        msg = can.Message(arbitration_id=self.motor_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def enter_rest_state(self):
        """Transition driver to idle Rest State and clear faults <= 128."""
        data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
        msg = can.Message(arbitration_id=self.motor_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def set_zero_position(self):
        """Set current physical position as 0.0 rad zero reference."""
        data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]
        msg = can.Message(arbitration_id=self.motor_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def send_servo_command(self, pos, vel, pos_kp, pos_kd, vel_kp, vel_kd, vel_ki):
        """
        Send command packet in Servo Mode (0x01).
        """
        p_int = self.float_to_uint(pos, self.P_MIN, self.P_MAX, 16)
        v_int = self.float_to_uint(vel, 0.0, self.V_MAX, 8)
        kp_p_int = self.float_to_uint(pos_kp, self.KP_MIN, self.KP_MAX, 8)
        kd_p_int = self.float_to_uint(pos_kd, self.KD_MIN, self.KD_MAX, 8)
        kp_v_int = self.float_to_uint(vel_kp, self.KP_MIN, self.KP_MAX, 8)
        kd_v_int = self.float_to_uint(vel_kd, self.KD_MIN, self.KD_MAX, 8)
        ki_v_int = self.float_to_uint(vel_ki, self.KI_MIN, self.KI_MAX, 8)

        data = [
            (p_int >> 8) & 0xFF,
            p_int & 0xFF,
            v_int & 0xFF,
            kp_p_int & 0xFF,
            kd_p_int & 0xFF,
            kp_v_int & 0xFF,
            kd_v_int & 0xFF,
            ki_v_int & 0xFF
        ]
        msg = can.Message(arbitration_id=self.motor_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def send_ptm_command(self, pos, vel, kp, kd, torque):
        """
        Send command packet in Position-Torque Mixed Control Mode (0x02).
        """
        p_int = self.float_to_uint(pos, self.P_MIN, self.P_MAX, 16)
        v_int = self.float_to_uint(vel, self.V_MIN, self.V_MAX, 12)
        kp_int = self.float_to_uint(kp, self.KP_MIN, self.KP_MAX, 12)
        kd_int = self.float_to_uint(kd, self.KD_MIN, self.KD_MAX, 12)
        t_int = self.float_to_uint(torque, self.T_MIN, self.T_MAX, 12)

        data = [
            (p_int >> 8) & 0xFF,
            p_int & 0xFF,
            (v_int >> 4) & 0xFF,
            ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF),
            kp_int & 0xFF,
            (kd_int >> 4) & 0xFF,
            ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF),
            t_int & 0xFF
        ]
        msg = can.Message(arbitration_id=self.motor_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def write_flash_parameter(self, index, value_float):
        """
        Write a single parameter value to Flash over CAN.
        """
        val_bytes = struct.pack('<f', float(value_float)) # IEEE754 Float little-endian
        data = [
            0x67,
            index & 0xFF,
            val_bytes[0],
            val_bytes[1],
            val_bytes[2],
            val_bytes[3],
            0x15, # Write Operation
            0x76
        ]
        msg = can.Message(arbitration_id=0x600 + self.motor_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def save_flash_parameters(self):
        """
        Trigger permanent Flash save command over CAN.
        """
        data = [0x67, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x76]
        msg = can.Message(arbitration_id=0x600 + self.motor_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def parse_feedback(self, data):
        """
        Unpack and decode telemetry feedback frame.
        """
        if len(data) < 8:
            return None

        status = data[0]
        pos_raw = (data[1] << 8) | data[2]
        vel_raw = (data[3] << 4) | (data[4] >> 4)
        torque_raw = ((data[4] & 0x0F) << 8) | data[5]
        fault_code = data[6]
        temp_celsius = data[7]

        pos_rad = self.uint_to_float(pos_raw, self.P_MIN, self.P_MAX, 16)
        vel_rads = self.uint_to_float(vel_raw, self.V_MIN, self.V_MAX, 12)
        torque_nm = self.uint_to_float(torque_raw, self.T_MIN, self.T_MAX, 12)

        return {
            'status': status,
            'position_rad': pos_rad,
            'velocity_rads': vel_rads,
            'torque_nm': torque_nm,
            'fault_code': fault_code,
            'temperature_c': temp_celsius
        }
```