# Online motor configuration and signed references

Implemented on feature/hip, 2026-10-27. Open **Motors - configure & monitor**
in the simulator header. All actions use the existing authenticated local
bridge API and work through the existing same-port SSH tunnel on the Jetson.

## Normal workflow

1. Stop the simulation and Disarm. Support the mechanism while torque is off.
2. Open the motor panel and use Read hardware for the enabled devices. This
   explicitly queries a fault-free Rest state and settings; it never enables
   a motor. Read all three loads each ankle PID set and independent current caps.
3. Place the mechanism at the desired reference pose and click Current pose =
   zero for one output, or Capture current pose as zero for all enabled outputs.
4. Set command limits relative to that zero, apply PID and any explicit motor
   output-cap changes, then start and arm normally. The user still controls motion.

The read and configuration controls for disabled outputs are unavailable.
Rotary PID can be applied while armed. Ankle PID, device cap writes, limits
and reference capture require both simulation Stop and all outputs Disarmed.
No automated excitation, homing motion, motor identification or auto-tuning
routine is started by the panel.

## Rotary tuning and limits

Knee and hip have independent P, I, D and integral-contribution caps.
P/D are transmitted in each P-T-M frame. Native P-T-M has no position I field;
the new I term is implemented in the bridge as bounded torque feedforward.
Its unit is Nm/(rad*s). It defaults to zero and must be enabled explicitly.
Conditional anti-windup prevents further integration into a saturated output.
It resets on Stop, Disarm, zero capture and gain changes.

Before enabling I, read and verify [21] Torque Limit and [46] Torque Constant.
Because Studio labels [21] in A but the supplied manual describes Nm, the
integral budget conservatively uses the smaller of the Nm and A*Kt
interpretations, and any saved lower cap. This is a nominal driver-model
budget, not independently measured output torque. The motor's own output
limit remains the hard drive-side cap; PID gains alone do not impose one.

Default P/D remain 20/4 independently for knee and hip. Software checks reject
non-finite values, negative gains, P above 250, D above 50, and I above 50.
CAN velocity, P, D and torque conversion ranges are checked before online
control is armed; their scaling must match the driver.

Command minimum/maximum are signed degrees relative to the captured reference.
They must include zero and fit within the verified native motor angle range.
Hip command limits remain within -30..30 degrees. The existing native encoder
and protection ranges ([24]/[25]) are displayed read-only; command-limit edits
narrow bridge commands, and do not rewrite the CAN conversion scale.
Hardware travel protection is not a mechanical stop.

The bridge now separates the operating envelope from the CAN conversion scale.
The normal launcher applies a knee angle ratio of 0.5 (50 simulator degrees
commands 25 motor degrees), including velocity feed-forward. Console signed
angles, zero references and command limits use that same mapping; raw motor
feedback retains motor units. The hip ratio remains 1.0.
After validating the motor parameters, it uses their exact Theta MIN/MAX
readbacks for both command packing and feedback decoding. The motor protection
range may include up to one degree outside each end of the original operating
envelope, or use the explicit wide setting **[24] = -12 rad, [25] = +12 rad**
for either rotary motor. Other expanded or narrower ranges still block arming. Software
targets remain in the original knee 0..105-degree UI range (the legacy upper
endpoint saturates at 1.83 rad) and hip -30..30-degree range. A knee configured
with the full protection clearance uses -0.0174532925..1.8474532925 rad.
Existing motor ranges without clearance remain supported.

Changing a motor's Theta range requires the updated bridge. Other applications
must also use the matching scale. Range changes are never sent during normal
startup. Position feedback within one encoding step of an operating endpoint
can quantize slightly outside it; the initial hold command is clamped back
inside the operating envelope after the existing position checks.

The Motor limit [21] field is an explicit device write, while stopped and
disarmed. It is labelled in firmware units because of the Studio/manual unit
conflict. Each write requires acknowledgement and a separate matching readback.
A failed write attempts to restore the previous limit. An unverified rollback
blocks arming until the affected device is read/verified. No flash-save,
electrical calibration, pole-pair, gear-ratio or torque-constant write is exposed.

## Linear group and individual channels

All three nodes receive one shared position PID at SDO index 0x2004,
subindices 1/2/3. Each unsigned 32-bit value is scaled by 100000 (resolution
0.00001). Read all three shows the existing values separately, including any
differences; no unknown factory PID values are invented.

A group update reads the original nine values before writing. Each write
checks the SDO acknowledgement and independent readback. On a failure, every
attempted write, including one with a lost acknowledgement, is rolled back.
An unverified group operation blocks arming until resolved. Writes happen
while disarmed, so partial sequential CAN updates cannot move the platform.

Each actuator has its own signed command limits in mm and its own peak-current
limit in A. The peak-current field uses 0x2009:03 in 0.01 A units. Software
position limits are bounded by the measured device limits at 0x607D:01/02,
and cannot exceed the current 0..50 mm absolute bridge envelope. They are
command restrictions; they do not rewrite actuator position calibration.

When the online console is enabled, the ankle worker requests PDO1 reports
at 100 ms per node using 0x1800:05. Commands continue at 50 Hz. The serial
worker retains sole ownership of the transport. Deployed firmware accepts
the timer write but rejects a read of this object. Timer setup therefore
requires its matching write acknowledgement without uploading it afterward;
live PDO arrival supplies the monitoring data. PID, travel-limit and current
cap writes still require independent readback. A rejected timer write or a
communication failure still stops startup. SDO errors include the device's
abort code when supplied. PDO1 reports are decoded as:

| Bytes | Quantity | Conversion |
| --- | --- | --- |
| 0..1 | Fault/status | unsigned 16-bit |
| 2..3 | Position | signed 16-bit * 0.01 mm |
| 4..5 | Speed | signed 16-bit * 0.1 mm/s |
| 6..7 | Reported force | signed 16-bit N |
| 8..9 | Motor current | signed 16-bit * 0.01 A |

Force and current are displayed separately for A1/A2/A3. No documented motor
torque constant is available for these actuators, so motor torque is marked
Unavailable instead of relabelling N as Nm.

## Zero semantics

Capture zero stores a bridge coordinate reference at the current measured
position. It does not send a motor machine-zero command, move to a hard stop,
tare a force sensor or change an electrical calibration. Rotary motors must
confirm stationary Rest and the actuators must report stationary speed.

After capture, actual positions, commanded positions and signed command limits
use the same reference. Ankle zero maps the simulator's nominal 20 mm geometry
to each captured drive position; displayed displacements and CSV a1/a2/a3
columns subtract that nominal 20 mm, making the reference pose 0 mm.
The raw transport positions remain available in existing API fields for
compatibility; the console and added relative telemetry fields are signed.

Capturing a new reference preserves existing absolute command bounds and
expresses them in the new coordinate frame. It never widens those bounds.
Capture is rejected if the current position is outside the existing limits.
The all-output operation gathers every enabled measurement before committing
any reference, so a missing device cannot leave a partially captured pose.

References are bound to the rotary Machine Angle Offset. If a motor's machine
zero is changed externally, arming rejects the stale reference. Capture a new
reference while stopped and disarmed. The older machine-zero buttons remain
separate operations, with their existing stationary checks and readback.

### Position fault during knee zeroing

A Rest reply ending in fault byte `0D` reports fault 13, Position Out of Range.
The legacy knee encoding/protection range is 0..1.83 rad; its optional
one-degree clearance is described above. A reported value at the lower
endpoint does not prove the actual shaft angle is inside
that interval; feedback can be clamped at an encoding endpoint.

Stop, disarm and support the mechanism before checking its physical pose and
machine reference with torque disabled. Resolve the out-of-range condition
before retrying the machine-zero button at the intended reference pose.
Keep the configured protections enabled. Changing CAN encoding endpoints also
changes feedback and command conversion and is not a fault-reset procedure.

A failed machine-zero verification intentionally blocks later Arm attempts.
Arm now includes the original zero failure in its error, and failed Rest reads
update the displayed feedback with the actual fault. A successful repeat of
machine-zero verification clears that lock; reading hardware or capturing a
software coordinate reference does not clear it.

## Monitoring and missing measurements

The panel shows actual/target angle, reported rotary torque, and mechanical
power (torque * angular speed). Ankle channels show signed displacement,
reported force/current and mechanical power (force * linear speed).
Mechanical output is not electrical consumption. Live samples come from the
control workers while armed; Read hardware refreshes stopped position samples.
Stale measurements are visibly marked and never converted to zero consumption.

No external power sensors are currently connected, as confirmed by the
operator. The rotary CAN feedback has no DC supply-voltage/current fields.
The ankle protocol lists raw BusBarVoltage at 0x200B:01 without a verified
conversion scale, and motor current is not DC supply current. Therefore
supply voltage and DC electrical power show Unavailable. Phase current is
never multiplied by assumed supply voltage to invent consumption.

An authenticated local sensor producer can later post to /api/power:

```json
{"device":"hip","voltage_v":48.0,"bus_current_a":1.5}
```

Device names are knee, hip, a1, a2, a3, restricted to enabled outputs. Samples
require finite voltage/current, identify an external DC source, and expire
after two seconds without updates. Electrical power is voltage * DC bus
current, independently for each device. This endpoint does not simulate a
sensor or change any motor setting.

## Persistence and API

The default profile is drivers/bridge_settings.json (ignored by Git). Writes
use a temporary file and atomic replacement. --config-file selects a path;
--no-config-save keeps changes in memory for this session. Disabled-output
settings are preserved when saving a profile from an --only mode.
Malformed or incompatible profiles fail startup instead of silently using
unsafe defaults. No automatically applied tuning change writes motor flash.

PID/current/torque settings written to a device may not survive its power
cycle without a separate manufacturer save procedure. On subsequent arming,
configured shared PID, per-node current caps, rotary torque caps and zero
bindings are checked against saved expectations, and mismatches block enable.
Use Read then explicitly Apply the intended settings again. Saved caps take
precedence in the editable fields; actual device readings are shown separately.

Authenticated POST routes:

| Route | Operation |
| --- | --- |
| /api/configure | target plus settings; rotary p/i/d/integral_limit_nm/limits_deg/torque_limit_value; ankle pid/limits_mm/current_limits_a |
| /api/read-hardware | stopped/disarmed read of target knee, hip or ankle |
| /api/reference | capture target knee, hip, ankle or all |
| /api/power | external per-device DC measurements |

Session/update responses include console settings and monitoring. GET
/api/configuration returns the same console snapshot. Existing local Host,
Origin, JSON content type and session-token checks apply to mutations.

## Verification and remaining hardware checks

Hardware-free tests cover wire encodings, acknowledged writes and readback,
partial-write rollback, invalid input atomicity, independent rotary PID,
shared ankle PID, separate current caps and power channels, signed references,
unchanged physical bounds on zero, profile persistence, inactive-output
preservation, stale sensors, anti-windup, zero-binding changes and API interlocks.
Headless Chrome checks cover the complete panel and bridge integration.

Actual read/write compatibility, PDO reporting support, PID behaviour and
profile expectations after a real actuator power cycle remain to be checked
on the physical jig. No physical motors were enabled during development.
Protocol evidence is from the repository's Motorevo v3.5 PDF, Seenpin V2.0
manual and CAN Protocol Version 26.02.11 workbook.
