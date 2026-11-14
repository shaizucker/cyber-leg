# Hip rotary joint integration

Implemented on branch `feature/hip`, 2026-10-25.

The hip uses the same Motorevo LTE1.1AN1800G rotary motor as the knee. The
knee remains classic CAN node **1** and the hip is classic CAN node **2**,
on the same `PCAN_USBBUS1` network at **1 Mbps** using standard 11-bit frames.
The three Seenpin ankle nodes remain on their separate CAN-FD adapter.
The bridge does not change motor IDs or persistent motor configuration.

## Start and output selection

From the repository root:

```powershell
python drivers/knee_bridge.py
```

Hip, knee and ankle outputs are enabled by default. The existing
`drivers/start_tuned_bridge.ps1` launcher also enables the hip automatically.
Select an individual physical subsystem with:

```powershell
python drivers/knee_bridge.py --only knee
python drivers/knee_bridge.py --only hip
python drivers/knee_bridge.py --only ankle
```

`--only all` is the default. Only the selected hardware is opened, checked,
armed, commanded and zeroed; absent joints do not need to be connected.
Ankle-only mode uses the Seenpin adapter without opening the classic CAN bus.
Hip-only mode addresses rotary node 2 without contacting knee node 1.
The Arm button names the selected outputs and knee zero is disabled when the
knee is excluded. Stop holds the selected subsystem; disarm, stale updates
and faults stop the selected outputs.

Use `--no-ankle` for hip and knee only, `--no-hip` for existing knee/ankle
hardware, or `--no-knee` for hip and ankle. The exclusion flags can be combined;
a selection with no remaining outputs is rejected. The simulator continues visualizing
all joints when a hardware output is disabled.

## Hip coordinates and reference

Updated 2026-10-25: the hip now uses **-30 to +30 degrees**, with **neutral
hip (0 degrees) = motor zero**. This supersedes the earlier -20..85 degree
range and -20 degree zero reference. Both manual input and bridge requests
are restricted to the new range; the existing -12..20 degree gait fits inside.

The hip's CAN position encoding and feedback scale are independently set to
**-0.5235987756..+0.5235987756 rad**. The knee retains its 0..1.83 rad scale.
Do not use the new hip bridge with the old 0..1.83 hip motor setting; its
configuration check rejects that mismatch before arming.

With the default mapping, motor position equals the hip angle in radians.
Stop and disarm, physically place the hip at neutral, and set hip motor zero
there. Do not copy the knee's machine offset or change Electric Angle Offset.
The hip-zero button verifies stationary Rest, acknowledgement, subsequent
feedback and Machine Angle Offset change; physical reference and power-cycle
persistence still require independent checks.

`--hip-sign` and `--hip-offset-deg` configure the mapping independently of the
knee. The offset is motor position at hip -30 degrees; its default is -30.
For a reversed mapping, use `--hip-sign -1 --hip-offset-deg 30`.
The zero button requires the default positive direction and reference offset.

## Motor-side limits and reduced stiffness

On **hip motor ID 2**, in MParameter Studio:

| Parameter | Hip setting |
| --- | --- |
| [11] Control Mode | P-T-M / PT-Mix (2) |
| [24] CAN COM Theta MIN | -0.523599 rad |
| [25] CAN COM Theta MAX | +0.523599 rad |
| [36] Motor ID | 2 |
| [37] CAN COM Time Out | 1000 ms |
| [56] Protect Switchs | Keep protections enabled; position bit 0, overspeed bit 1, CAN-timeout bit 7 required; 0xFFFFFFFF enables all |

These Theta fields define the CAN conversion scale. With position protection
enabled, the supplied Driver User Manual v3.5 also specifies fault 13 when
the measured output angle leaves that range. It faults to Rest; this is not
a mechanical stop and does not guarantee absence of overshoot under inertia
or external loads. Save Parameter, wait for flash completion, and read the
values back before using the updated bridge. Power-cycle persistence must
still be checked on hardware.

Hip P/D defaults are **20/4**, restored after P/D 2/0.5 produced poor
physical slider response. With P=2, a 1-degree error contributes only about
0.035 Nm, and almost 29 degrees of error are required to request 1 Nm from
the proportional term. Motor friction and gravity can therefore mask small
slider changes even while command targets update continuously.

Override gains independently with `--hip-kp-pos` and `--hip-kd-pos`.
Position gains determine tracking stiffness; **they do not enforce a hard
torque cap**. The bridge never increases the motor's [21] limit automatically.
Keep a verified low motor-side cap for supported commissioning. A 1 Nm cap
cannot hold the operator's complete unsupported jig throughout its range.
The UI reports hip actual angle, commanded angle and nominal feedback torque
to distinguish target-delivery issues from lag caused by load or saturation.
Feedforward remains zero.
The motor-side parameter for output limiting is **[21] Torque Limit**.
Do not use [34]/[35] CAN COM Torque MIN/MAX as a torque cap: those are encoding
ranges and must remain -50/+50 for the existing driver.

There is an unresolved units mismatch: the supplied manual (printed page 29,
PDF page 30) describes [21] in Nm and [46] Torque Constant in Nm/A, whereas
the operator's MParameter Studio screenshot labels [21] in A. A numeric
jig-safe motor cap has not been established. Confirm actual firmware units
and calibrated torque constant before treating a numeric setting as Nm.
If a limit is in amps, nominal torque is Iq times Torque Constant; that
constant must already account for the output shaft when interpreting output
torque. Do not change the calibrated constant to reduce output.

For a mass m at perpendicular distance r from the hip axis, static torque is
m*g*r. A 2 kg load at 0.10 m needs approximately 1.96 Nm; at 0.30 m it needs
5.89 Nm when the force acts perpendicular to the arm. With the centre of
mass below the hip at angle a from vertical, r is L*sin(a). This estimates
holding torque, not a safe structural limit. Support the jig during low-torque
commissioning, as reduced output may no longer hold it against gravity.

## Control and API

The simulator provides a manual hip slider, automatic gait hip motion, a visible
rotary housing and hip axis, actual hip feedback, and CSV hip angle/velocity
columns. The prosthesis rotates about the hip while the dummy torso stays upright.

`POST /api/update` includes `hip_angle_deg` and optional
`hip_velocity_deg_s` alongside the existing knee and ankle fields. A hip target
is required when hip output is enabled; invalid/missing hip targets do not update
any output. `POST /api/zero` with `{"joint":"hip"}` selects hip zero;
an omitted joint still selects knee zero. Session/update responses include a
`hip` status with node ID, configuration, targets, feedback, gains and reference.
`GET /api/telemetry` retains knee `samples` and adds separate `hip_samples`.
Existing local-origin and token checks apply to every hardware action.

One lazy CAN connection is shared by both rotary drivers. Complete request/reply
transactions, including parameter reads, are serialized; received frames must
match the requested standard CAN ID. Each motor close sends Rest without closing
the shared connection, which is released when the bridge exits.

Both rotary outputs run from one 50 Hz control worker. Before arming, both use
the existing P-T-M mode, position-scale and protection checks. Both hold their
measured startup positions until ankle initialization is verified. Stop holds;
reset and explicit disarm stop all enabled outputs. A rotary feedback timeout,
rotary fault, ankle fault or stale browser update disarms the complete system.

## Verification

Hardware-free tests cover signed hip targets, independent joint velocity,
range/missing-target validation, shared CAN node routing and bus ownership,
HTTP routing and telemetry, stop hold, failed arming, and coordinated fault,
timeout, browser-loss and ankle-startup handling. Existing knee, ankle and
configuration safety regressions also pass.

A headless Chrome check with fake motors verified WebGL startup, manual hip
motion and its output target, arming, stop hold, reset/disarm and automatic gait,
with no page errors. Physical hip commissioning has not been performed.

## Online controls added 2026-10-27

The simulator now provides separate rotary PID tuning, a bounded bridge I
term (disabled by default), signed command limits, explicit motor [21] output
cap writes with readback and captured coordinate references. This extends the
older description of zero feedforward: it remains zero at I=0, and becomes
the bounded integral contribution only when I is explicitly enabled.
See ../Motors/online_configuration.md for shared ankle PID, signed-reference
semantics, actual monitoring capabilities and persistence details.
