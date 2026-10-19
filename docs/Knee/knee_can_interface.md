# Knee Motor — CAN Interface Guide (Angle Control)

How to drive the Cyber-Leg knee actuator over CAN from an external program, such
as a full-leg simulator, using **only the knee angle** as the command.

Source of truth: `docs/Knee/Driver User Guide V3.5` (Motorevo driver manual).
Reference implementation: `drivers/motorevo_driver.py` and `drivers/knee_control_cli.py`.

---

## 1. Hardware summary

| Item | Value |
| :--- | :--- |
| Actuator | Motorevo `LTE1.1AN1800G`, 36:1 reducer |
| Motor CAN ID | `1` |
| Control mode (in flash) | `1` = Servo Mode (position PD → velocity PI → FOC current) |
| CAN | CAN 2.0A (11-bit standard IDs), **1 Mbps**, 120 Ω termination at both ends |
| Host adapter | PEAK PCAN-USB (`PCAN_USBBUS1`) on Windows; SocketCAN (`can0`) on Linux |
| Power | 48 V DC. The driver needs up to **4 A continuous**. |
| CAN watchdog | `CAN COM Timeout` = **1000 ms** |
| Torque limit | 15 Nm |
| LEDs | Green = power OK. Blue = Motor State (armed). |

> **Power supply:** if the supply is current-limited, or too weak for the load,
> the bus voltage sags under load. The motor then crawls, its torque flips
> back and forth (jitter), and it can trip **fault 2 (bus undervoltage)** and
> drop off the bus. Use a supply rated for at least 4–5 A at 48 V.

---

## 2. Angle convention

- All angles are **output-shaft (knee joint) angles in radians**. The 36:1 gear
  ratio is handled inside the driver, so you never scale by 36.
- Commanded and reported position is a 16-bit value mapped linearly onto
  `[-12.5, +12.5]` rad. That's a resolution of about 0.00038 rad (0.022°).
- Velocity is in output-shaft rad/s.
- `0 rad` is wherever the joint was when the zero command was last sent
  (see section 5).
- The direction (sign) relative to your simulator's knee-flexion convention must
  be checked once on the hardware. Handle it with a sign constant in your
  bridge code (section 8).

If the joint ever goes beyond ±12.5 rad, the driver raises **fault 13
(position out of range)**. A knee only moves about ±2.5 rad, so that's not
a concern in normal use.

---

## 3. CAN frames

All frames are **standard 11-bit, DLC 8, 1 Mbps**.

### 3.1 State commands (ID = `0x001`)

| Command | Bytes 0–7 | Effect |
| :--- | :--- | :--- |
| Enter Motor State | `FF FF FF FF FF FF FF FC` | Arms the drive. The blue LED turns on. |
| Enter Rest State | `FF FF FF FF FF FF FF FD` | Disarms the drive (no torque, free to backdrive) and clears clearable faults. |
| Set Zero Position | `FF FF FF FF FF FF FF FE` | Sets the current angle as `0.0 rad`. |

The driver **ignores Enter Motor State while a fault code is set**. Send Enter
Rest State first, and check that the reply shows `fault = 0`.

### 3.2 Servo-mode position command (ID = `0x001`)

| Byte | Field | Bits | Range (default) |
| :--- | :--- | :--- | :--- |
| 0 | θ_ref high byte | 16 | −12.5 … +12.5 rad |
| 1 | θ_ref low byte | | |
| 2 | V_ref (transition velocity) | 8 | −10 … +10 rad/s (send a positive value) |
| 3 | Position Kp | 8 | 0 … 250 |
| 4 | Position Kd | 8 | 0 … 50 |
| 5 | Velocity Kp | 8 | 0 … 250 |
| 6 | Velocity Kd | 8 | 0 … 50 |
| 7 | Velocity Ki | 8 | 0 … 0.05 |

Encoding (identical to the manual's `float_to_uint`):

```python
def float_to_uint(x, x_min, x_max, bits):
    x = min(max(x, x_min), x_max)
    return int((x - x_min) * ((1 << bits) - 1) / (x_max - x_min))
```

Manual example: θ = 3.14 rad, V = 2.0 rad/s, Kp = 15, Kd = 4.5, vel Kp = 50,
vel Kd = 0, vel Ki = 0.001 → `A0 26 99 0F 16 33 00 05`.

### 3.3 Reply frame (motor → host, ID = `0x001`)

Every command, including state commands, gets exactly one reply within about 2 ms.

| Byte | Content |
| :--- | :--- |
| 0 | Status: `0` = Rest, `1` = Motor (Servo), `2` = Motor (P-T-M), `3` = Velocity, `4` = Torque |
| 1–2 | Position, 16 bits → −12.5 … +12.5 rad |
| 3, 4[7:4] | Velocity, 12 bits → −10 … +10 rad/s |
| 4[3:0], 5 | Torque, 12 bits → −50 … +50 Nm |
| 6 | Fault code (0 = OK, see section 9) |
| 7 | Driver PCB temperature, °C |

```python
pos_raw = (d[1] << 8) | d[2]
vel_raw = (d[3] << 4) | (d[4] >> 4)
tau_raw = ((d[4] & 0x0F) << 8) | d[5]
value   = x_min + raw * (x_max - x_min) / ((1 << bits) - 1)
```

### 3.4 Parameter read/write (ID = `0x601`)

Used for configuration only, never in the control loop.

- Read: `67 <index> 00 00 00 00 04 76`
- Write: `67 <index> <data0..3 little-endian> 15 76`
- Reply on `0x601`: `01 <index> <data0..3> 00 FF`
- Save all parameters to flash: `67 00 00 00 00 00 15 76`

Useful indices (decimal): 11 = Control Mode, 21 = Torque Limit, 23 = Machine
Angle Offset (zero), 36 = Motor ID, 37 = CAN COM Timeout (ms).

---

## 4. Startup sequence

1. Open the bus at 1 Mbps.
2. Send **Enter Rest State** and read the reply. Check that `fault == 0`.
   - If fault 16 (CAN timeout) or fault 2 (undervoltage) won't clear, power-cycle the 48 V supply.
3. (Optional) Zero the joint (section 5). Only do this while disarmed.
4. Read the current position `θ_now` from the reply.
5. Initialise your setpoint to **`θ_now`**, not to the simulator's angle and not to 0.
   Otherwise the joint jumps the instant it's armed.
6. Send **Enter Motor State**. Check that the reply shows `status == 1`.
7. Start the control loop immediately (section 7). The first servo frame must go
   out well within 1000 ms.
8. Move the setpoint gradually from `θ_now` to the simulator's angle (section 6).

Shutdown: send **Enter Rest State**, then close the bus.

---

## 5. Zeroing

**Goal:** make `0.0 rad` match the simulator's reference pose. For example,
knee fully extended = 0.

1. Disarm (Enter Rest State). The joint is now free to backdrive.
2. Move the leg by hand to the reference pose (use a jig or stop for repeatability).
3. Send **Set Zero Position** (`FF FF FF FF FF FF FF FE`).
4. Check: the reply, or any following Rest reply, should report a position of about `0.000 rad`.

In the CLI: run `python drivers/knee_control_cli.py`, press `s` to disarm, position
the leg, then press `z`.

**Persistence:** the manual doesn't say whether the CAN zero command survives a
power cycle. It sets parameter 23 (Machine Angle Offset). To check, zero the
joint, power-cycle, and read the position again. If the zero was lost, persist
it by sending **Save Flash** (`67 00 00 00 00 00 15 76` on `0x601`) after zeroing.
Wait 1–2 s and don't cut power while the flash write is in progress.

Alternative without hardware zeroing: leave the driver's zero alone and apply
an offset in software (`θ_motor = sign * θ_sim + offset`, section 8).

---

## 6. Speed of movement

In Servo Mode, speed is controlled in two layers.

### 6.1 V_ref — the driver's transition-velocity limit (byte 2)

`V_ref` is the maximum speed the driver uses to reach θ_ref. It's a ceiling,
not a profile. If you send one big step in θ_ref, the joint lunges toward it
at up to `V_ref` and then settles, which feels jerky.

### 6.2 Host-side trajectory — the setpoint you stream (recommended)

For smooth motion, the host should move θ_ref a little each frame. Then the
driver only ever chases a small, continuously moving target.

- **Simulator-driven:** the simulator already produces a smooth knee angle over
  time. Stream it directly at 100 Hz and set `V_ref` somewhat above the fastest
  knee speed the simulator produces (for example, peak sim speed × 1.5).
- **Point-to-point moves:** use a minimum-jerk profile (as in
  `knee_control_cli.py`):

```python
def min_jerk(tau):            # tau in [0, 1]
    tau = min(max(tau, 0.0), 1.0)
    return tau**3 * (10 - 15*tau + 6*tau**2)

# Move from p0 to p1 with a peak speed of v_max (rad/s)
T = max(0.6, 1.875 * abs(p1 - p0) / v_max)   # peak velocity of min-jerk = 1.875·Δ/T
theta_ref(t) = p0 + (p1 - p0) * min_jerk(t / T)
```

Keep `V_ref` above `v_max` so the driver never clips the profile.

### 6.3 Typical values

| Parameter | Value used | Notes |
| :--- | :--- | :--- |
| Loop rate | 100 Hz (10 ms) | 50 Hz is the minimum. Never let the gap between frames approach 1000 ms. |
| Peak trajectory speed | 1.0 rad/s | About 57°/s. Raise it gradually for gait-speed motion. |
| V_ref | 2.5 rad/s | Must stay above the peak trajectory speed. |
| Position Kp / Kd | 40 / 5 | Starting values. The 45° test used 50 / 6. |
| Velocity Kp / Kd / Ki | 50 / 0 / 0.005 | Starting values. Manual example: 50 / 0 / 0.001. |

The gains aren't fully tuned yet. Tuning was cut short by a power-supply
undervoltage event. Retune once the 48 V supply is confirmed stiff.

---

## 7. Control loop requirements

- **Watchdog:** if no frame arrives for 1000 ms, the driver latches **fault 16**
  and drops to Rest. Keep a dedicated loop or thread sending frames for as long
  as the motor is armed. Never block it on user input, rendering, or simulator stalls.
- **If the simulator stalls:** keep sending the last setpoint (hold). Don't stop sending.
- **One reply per frame:** read it to get the measured angle, velocity, torque, and fault.
- **Check every reply:** if `status` becomes `0` while you think the motor is armed,
  it faulted. Stop the trajectory, log the fault code, and re-arm only after the
  cause is understood. When you re-arm, start from the measured angle again.
- **Joint limits:** clamp the setpoint to the knee's mechanical range in software
  before encoding it (for example `[0, 2.1] rad` for 0–120° flexion, to be
  confirmed for this leg).

---

## 8. Simulator bridge example (Python)

```python
import math, time
from motorevo_driver import MotorevoMotor

SIGN      = +1.0          # flip if motor and simulator flexion directions differ
OFFSET    = 0.0           # rad, motor angle when sim angle = 0 (if not zeroed in hardware)
THETA_MIN = math.radians(0)
THETA_MAX = math.radians(120)
DT        = 0.01          # 100 Hz
V_REF     = 2.5           # rad/s, above the fastest sim knee speed
MAX_STEP  = 1.0 * DT      # rad per frame, slew limit (1 rad/s) while catching up to the sim

def sim_to_motor(theta_sim):
    return SIGN * min(max(theta_sim, THETA_MIN), THETA_MAX) + OFFSET

motor = MotorevoMotor(motor_id=1, interface="pcan", channel="PCAN_USBBUS1")  # Linux: "socketcan", "can0"
t = motor.enter_rest_mode()
assert t and t["fault"] == 0, f"fault {t and t['fault']}"
setpoint = t["position_rad"]
t = motor.enter_motor_mode()
assert t and t["status"] == 1

try:
    next_t = time.perf_counter()
    while True:
        goal = sim_to_motor(get_sim_knee_angle())         # your simulator hook
        setpoint += max(-MAX_STEP, min(MAX_STEP, goal - setpoint))
        fb = motor.send_servo_command(setpoint, vel_limit_rad_s=V_REF,
                                      kp_pos=40, kd_pos=5, kp_vel=50, kd_vel=0, ki_vel=0.005,
                                      timeout=0.005)
        if fb:
            send_to_sim((fb["position_rad"] - OFFSET) * SIGN)  # measured knee angle back to the sim
            if fb["status"] == 0:
                raise RuntimeError(f"motor faulted: {fb['fault']} {fb['fault_desc']}")
        next_t += DT
        time.sleep(max(0.0, next_t - time.perf_counter()))
finally:
    motor.close()   # sends Enter Rest State and releases the bus
```

The slew limit (`MAX_STEP`) protects against jumps when the simulator resets,
teleports the leg, or first connects. Once the setpoint has caught up, a smooth
simulator trajectory passes through unchanged as long as it stays below 1 rad/s.
Raise the limit for faster gait.

---

## 9. Fault codes (reply byte 6)

| Code | Meaning | Action |
| :--- | :--- | :--- |
| 0 | No fault | — |
| 1 | Bus overvoltage (> 80 V) | Check supply. Watch for regen energy when decelerating. |
| 2 | Bus undervoltage (< 10 V) | Supply sagging or current-limited. Fix the power, then power-cycle. |
| 3 | Encoder offset error | Recalibrate (Motorevo Studio) |
| 4 | DRV driver fault | Power-cycle. Contact vendor if it persists. |
| 5–8 | Bus / phase A / B / C overcurrent | Reduce gains or load. Check wiring. |
| 9 / 10 | PCB / motor coil overheat | Let it cool. Improve cooling. |
| 11 | Encoder fault | Restart. Check the encoder. Recalibrate. |
| 12 | Current sensor fault | Hardware. Contact vendor. |
| 13 | Position out of range (±12.5 rad) | Move the shaft back into range by hand. |
| 14 | Velocity out of range (±10 rad/s) | Lower speed or gains. |
| 15 | Stall protection | Remove the obstruction. Check the electrical-angle offset. |
| 16 | CAN timeout (no frame for 1000 ms) | Keep frames flowing at 50–100 Hz. Clear with Rest, or power-cycle if it won't clear. |
| ≥ 129 | Encoder self-test faults | Can't be cleared over CAN. Restart. Contact vendor. |

---

## 10. Quick checks

```powershell
python drivers\test_pcan_motor.py      # ping: expect a reply on ID 0x1 with fault 0
python drivers\knee_control_cli.py     # manual: s (disarm) -> z (zero) -> m (arm) -> g / + / - (move)
```

| Symptom | Likely cause |
| :--- | :--- |
| Only the green LED, motor won't arm | A fault is set (often 16). Send Rest. If it persists, power-cycle. |
| Arms, then drops to Rest after about 1 s | Frames stopped. The watchdog tripped (fault 16). |
| Jittery or crawling motion, torque sign flipping | Supply sagging (see the power note in section 1), or gains too high. |
| No reply, PCAN error counter climbing | Motor unpowered or rebooting, or the CAN wiring/termination is faulty. |
