# Local simulator bridge

Updated October 24, 2026 to match the working `gemini-code-gui.py` motor control.

Run on the Windows PC connected to PEAK-CAN:

```powershell
python drivers\knee_bridge.py
```

Open **http://127.0.0.1:8000/**. Restart an existing bridge and reload the browser after updating. The bridge opens PCAN_USBBUS1 at 1 Mbps, motor ID 1, when **Arm motor** is pressed. Python requires `python-can`, `pyserial`, and the installed PEAK PCAN driver. Close the standalone GUI or other motor clients before arming through the bridge. Ctrl+C disarms and closes the bus.

## Motor configuration and commands

The motor must already be configured in MParameter Studio for **P-T-M mode (status 2)** with **Theta MIN 0.0 / MAX 1.83 rad**. The bridge configures its encoder/decoder for these ranges; it does not change motor flash settings. Arming rejects a Servo-mode reply rather than transmitting the wrong command layout.

Commands match the GUI's 8-byte P-T-M payload: position uses 16 bits; velocity, position Kp, position Kd, and torque use 12 bits each. Defaults are **Kp 20 / Kd 4**, simulator knee velocity in automatic mode, and zero feedforward torque. Commands stream nominally at **50 Hz**. Position targets now pass directly from the simulator without the previous two-stage filter or 2 rad/s command cap. Manual positioning uses zero velocity reference, as in the working GUI. Automatic gait supplies knee velocity (degrees/second, calculated from successive rendered poses), mapped to motor direction and bounded by the configured +/-10 rad/s protocol range. Stop, manual mode, and outward motion at position endpoints request zero velocity. Velocity feedforward also expires after 40 ms without a new browser update; the existing 0.5-second disconnect disarm remains.

The simulator slider and bridge accept **0 to 105 degrees**. Degrees convert to radians directly, with 105 degrees saturating at **1.83 rad (about 104.85 degrees)** exactly as the working GUI does. Feedback uses the same position scale. Position/velocity/torque and fault decoding retain the driver's documented format: fault is byte 6, board temperature is byte 7 (the GUI labels byte 7 as a fault).

Optional tuning, requiring a restart and explicit re-arm:

```powershell
python drivers\knee_bridge.py --kp-pos 20 --kd-pos 4
```

Only the two position PD gains apply to P-T-M. Old Servo-mode `--kp-vel`, `--kd-vel`, and `--ki-vel` arguments are no longer supported. `drivers/start_tuned_bridge.ps1` uses the new defaults. Protocol gain ranges are Kp 0..250 and Kd 0..50; these are encoding limits, not validated tuning ranges for an assembled leg.

By default, simulator zero maps to motor zero and positive flexion maps to positive motor angle. Mapping is `motor = offset + sign * min(radians(simulator angle), 1.83)`. Both endpoints must fit within 0..1.83 rad. For reversed direction, use `--sign -1 --offset-deg 104.85127650894066` (offset 1.83 rad); sign -1 with zero offset is rejected. These flags do not change the motor's hardware zero.

## Operation and diagnostics

To zero, stop and disarm the simulator, place the shaft at the physical zero reference, and press **Set current position as zero**. This requires `--offset-deg 0`. The bridge sends Rest, checks for faults, sends Zero, waits 150 ms, and verifies a subsequent fault-free Rest reply within 0.5 degrees of zero. It stays disarmed afterward.

Arming checks current browser updates, a fault-free Rest reply, and measured position within the mapped range with a one-degree margin. It reads measured position, waits 50 ms, enters P-T-M mode, and streams commands. The next control tick sends the latest simulator position directly. This also applies to the first target after arming and large manual slider jumps. Automatic motion includes its velocity reference to avoid damping against a stationary target. Physical response still needs hardware validation. Very fast gait can exceed the configured 10 rad/s velocity range; those references saturate and exact physical synchronization is not established.

Simulator Stop holds the current commanded position. Reset, switching mode, hiding the tab, and Disarm request Rest. Loss of browser updates for 0.5 seconds, missing feedback, a non-P-T-M status, or a motor fault disarms. Re-arming is explicit. The drive's documented CAN watchdog is a separate fallback if communication fails.

The terminal logs raw CAN TX/RX frames, errors, timeouts, and `TRACK` summaries at 4 Hz. TX confirms adapter acceptance. Logging runs separately and drops lines if terminal output stalls. `/api/telemetry` returns up to 6,000 feedback samples with monotonic timestamps, goal and command angles, status, and fault. The buffer clears on process restart. The browser shows the active P-T-M gains and motor feedback.

Run hardware-free checks:

```powershell
python -m unittest discover -s drivers -p test_knee_bridge.py -v
```

Tests cover target mapping, CAN bytes matching the working GUI, feedback scaling, direct forward/reverse commands, gait timing at simulator speeds, velocity feedforward/limits/expiry, stop/hold, mode validation, disconnect/fault handling, zeroing, and the authenticated local HTTP API. They do not establish physical accuracy, settling time, or overshoot.


## Three-actuator ankle platform

Normal startup now includes knee and ankle control:

```powershell
python drivers\knee_bridge.py --ankle-port COM7
```

The Seenpin transport matches `seenpin_extend_return.py`: serial 115200 baud, SLCAN `C,V,S8,Y5,M0,A0,O`, CAN-FD+BRS at 1 Mbps arbitration / 5 Mbps data, nodes 1/2/3. The port opens only on **Arm knee + ankle**. Close the example script and CANgaroo first. `--no-ankle` selects knee-only operation.

User-confirmed mapping: simulator A1/A2/A3 correspond to node IDs 1/2/3. Simulator extensions are absolute millimetres, with neutral at 20 mm. Each browser update includes the three `extensions_mm` values already computed by inverse geometry from ankle pitch, roll and height. Do not add the height offset again in the bridge. A pure height change moves all three together; tilt produces differential movement, including when height is also adjusted.

An independent ankle worker opens the serial adapter, reads all three fault codes, positions and software travel limits, then enables all three drives before sending the three position targets. Initialization is asynchronous so browser updates and knee CAN streaming continue. While arming, the browser shows ankle connection progress. It shows actual actuator positions once available. A reported failure in either subsystem disarms both.

The prior verified run reported 0..39 mm travel for each drive. The browser initially constrains ankle geometry to that range when ankle control is enabled; after initialization it uses the actual per-drive limits. The pose solver scales tilt and height together toward the neutral pose if their combined request exceeds travel. The backend independently rejects any group containing an out-of-range target before sending any member of that group. Initialization also rejects drive limits that exclude the neutral 20 mm reference.

Commands use the example's `0x200 + node` control frame, signed position in 0.01 mm units, 100 ms movement time, and control word `0x000F`. All three commands refresh nominally at 50 Hz; SDO telemetry polling continues between refreshes. A serial/SDO wait cannot block the knee thread. The frame stream provides closely spaced commands on separate buses; it does not establish hardware clock synchronization or guaranteed physical tracking speed.

Stop holds each actuator's last commanded position. Disarm, reset, hidden-tab handling, stale browser updates, serial failure, or a reported drive fault trigger best-effort disable (`0x0004`) on all nodes; disabling can release the platform's supporting force. The bridge does not automatically return the platform after a fault. The knee zero button only sets the knee reference; it does not home the ankle.

Hardware-free verification:

```powershell
python -B -m unittest discover -s drivers -p test_knee_bridge.py -v
python -B -m unittest discover -s drivers -p test_ankle_bridge.py -v
node drivers/test_ankle_geometry.mjs
```

These cover transport bytes and compact SDO replies, measured limits, grouped target validation, parallel workers, cancellation, stop/hold, faults and disconnects, HTTP target forwarding, and combined platform height/tilt geometry. Physical actuator speed, load support, position reference and assembled-platform tracking still require hardware verification.


### Missed Seenpin feedback replies

Seenpin SDO reads now retry up to three times with a 250 ms reply window per attempt, matching the working example's retry count. The driver continues streaming motion targets and checking Disarm/browser freshness during these waits. It logs retries, recovery, and observed CAN IDs if all attempts fail. An actual nonzero drive fault, SDO abort, or adapter BEL still stops immediately; a persistent missing reply still disarms both knee and ankle.

Telemetry is scheduled at one node per approximately 33 ms (about 10 Hz per node, reading fault and position). Three-actuator movement commands remain nominally 50 Hz. This reduces SDO traffic compared with the earlier unpaced loop. The reported node 3 timeout alone does not establish whether the cause was traffic, an isolated lost reply, power, wiring or node configuration. Hardware recovery remains to be verified.


### Softer ankle movement

Ankle commands now start from the three measured positions and pass through a shared trajectory filter. A common lead limits extension speed to 20 mm/s; two 0.10-second filter stages ease starts, reversals and target changes without restarting on each browser update. A shared 0.03 mm target deadband rejects tiny target fluctuations. Commands remain nominally 50 Hz, but the drive movement time is now 100 ms instead of the previous 20 ms. Positions are rounded to the drive's 0.01 mm resolution. The filter is bounded to a 40 ms advance after scheduling pauses, so it does not jump ahead to catch up.

All three axes use the same filter timing and a common lead progress factor. A fixed height/tilt target therefore retains proportional actuator travel before position quantization. States remain inside the bounds of their measured starting positions and accepted targets. Stop immediately holds the last sent position and resets filter history; Disarm and fault handling bypass motion smoothing. Knee target timing is unchanged.

The softer ankle profile intentionally adds lag relative to the simulator. Physical jitter and settling still require hardware verification; passing command tests do not prove that motor control or mechanical oscillation has been eliminated.

Defaults apply with ordinary startup. Optional softer tuning:

```powershell
python drivers\knee_bridge.py --ankle-port COM7 --ankle-response-time 0.15 --ankle-speed-mm-s 15
```

Higher response time gives more smoothing; lower speed limits extension/retraction further. Both options require restarting the bridge. `--ankle-response-time` is the time constant per filter stage, not a guaranteed move duration.

Additional deterministic tests: `python -B -m unittest discover -s drivers -p test_ankle_trajectory.py -v`. These verify soft starts, common-axis progress, reversal continuity, speed bounds, target-noise suppression, measured initialization, stop/reset and scheduling pauses.


### Knee zero and protection audit (October 24, 2026)

A live, read-only CAN parameter audit returned:

| Parameter | Readback |
| --- | --- |
| Control Mode (11) | 2, P-T-M |
| Machine Angle Offset (23) | 4.095345497131348 rad |
| CAN COM Theta MIN (24) | 0.0 rad |
| CAN COM Theta MAX (25) | 1.8300000429153442 rad, about 104.85 degrees |
| CAN Timeout (37) | 1000 ms |
| Rotate Dir (47) | +1; physical clockwise sense remains unverified |
| Protect Switchs (56) | 0xFFFFFF7C |

At that audit, protection bits 0 (position), 1 (overspeed), and 7 (CAN timeout) were OFF. Merely setting the mapping endpoints or a nonzero timeout did not enable these protections. Setting those three bits would produce 0xFFFFFFFF, preserving the other enabled bits. This is an observed audit snapshot, not evidence of a later configuration change or persistence after a power cycle.

The source is the supplied manufacturer `Driver User Manual V3.5(3).pdf`: sections 6.3/6.5 describe the CAN mapping, section 6.5.2 lists position-out-of-range fault 13, section 6.7.2 defines the machine offset and protection bits, and section 6.9 documents CAN parameter reads. No safety-rated physical stop or zero persistence was established by this audit.

The bridge now reads these seven parameters before arming and before zeroing. It rejects mismatched position encoding ranges, non-finite offsets, wrong control mode, disabled position/overspeed/CAN-timeout protection, or an invalid timeout. It does not write configuration automatically. Repeated parameter reads occur only during these disarmed checks, not in the motion loop.

Zeroing now requires a fault-free, stationary Rest reply and a fault-free Zero acknowledgement, then three later stationary Rest replies within 0.1 degree of zero. The bridge reads Machine Angle Offset again and rejects an unchanged offset when the pre-zero angle was measurably nonzero. Failed zero verification blocks subsequent arming until zero is verified again. A zero offset changed externally after verification also blocks arming. No Save Flash command is issued by zeroing. These checks confirm command/readback behavior; a clamped CAN position cannot independently establish the true physical zero. The reference pose, physical direction, and persistence after a user-controlled power cycle remain separate checks.

Re-run the non-moving audit with:

```powershell
python drivers\verify_knee_configuration.py
```

This utility sends only parameter-read frames. It does not send Rest, Enable, Zero, motion, configuration writes or Save Flash. Its report explicitly marks physical zero, clockwise direction, power-cycle persistence and guaranteed travel as unverified. A failed configuration check exits with an error.

A software command clamp, an encoder range and fault-triggered torque removal do not establish that physical motion can never exceed 105 degrees. Inertia, external loading, encoder/reference faults and power loss remain outside this command check. A physical travel guarantee needs an appropriately designed independent stop/protection mechanism and validation of the assembled system.

### Knee operating zero and protection clearance (October 27, 2026)

The normal bridge launcher now divides simulator knee angles and velocity
feed-forward by two: a 50-degree simulator target commands 25 degrees of motor
rotation relative to the selected reference. The hip remains one-to-one.
Console knee angles use the inverse mapping so they remain in simulator joint
degrees; raw feedback and TRACK motor angles remain motor degrees. The knee UI
still spans 0..105 degrees, with maximum motor travel 0.915 rad from reference.
CAN encoding continues to use the servo's actual Theta MIN/MAX parameters.

The bridge now supports a motor protection range with up to one degree of
clearance outside each operating endpoint. The operating mapping remains
0..1.83 rad (the 105-degree UI endpoint still saturates at 1.83 rad). Full
clearance is CAN Theta MIN -0.0174532925 and MAX 1.8474532925 rad. Protections
remain enabled. Range parameters are read and validated before any Rest or
Enable feedback is interpreted; the exact reported values scale command and
feedback bytes. The explicit wide setting CAN Theta MIN **-12 rad** and
MAX **+12 rad** is also supported for either rotary motor. Set servo parameters
[24] and [25] to these values in Motor Parameter Studio; the bridge reads them
on the next connection. Software targets remain knee 0..105 degrees and hip
-30..30 degrees. Other wider arbitrary scales and ranges narrower than the
original operating envelope are rejected. No range write occurs at normal startup.

A confirmed diagnostic on a disarmed knee exposed approximately -4.56 degrees
when its former zero-at-the-boundary feedback had shown 0.00 degrees. The
temporary diagnostic range was restored immediately. The user subsequently
confirmed the current pose as the intended new knee zero. Software range
clearance does not repair an incorrect reference or clear every latched
firmware fault; a fault-free stationary readback is still required for Arm.
The bridge never enables a motor just to bypass a reported fault.


After explicit user approval on October 24, 2026, parameter 56 was changed from 0xFFFFFF7C to 0xFFFFFFFF, the write acknowledgement was checked, Save Flash was sent, and all seven audited parameters were read again. Protection readback was 0xFFFFFFFF. Machine Angle Offset stayed 4.095345497131348 rad, Theta stayed 0..1.8300000429153442 rad, and all other audited values were unchanged. No movement, zero, Rest or Enable commands were sent. The verified before/after record is `knee_protection_verification_2026-10-24.json` in this directory. Save-command transmission and live readback are confirmed; persistence after power cycling and the physical reference/direction/travel remain unverified.

When the mechanism is safely supported and stopped, a user-controlled power cycle followed by `python drivers\verify_knee_configuration.py` can check whether the reported protection mask, angle range and zero offset were retained. Do not treat a successful report as an independent physical-angle measurement or a mechanical stop test. The clockwise viewpoint still needs confirmation from the user; Rotate Dir +1 alone does not identify that physical sense.


### Seenpin startup fault 32770 / 0x8002

The supplied Seenpin V2.0 CAN-FD manual, Appendix 3, identifies 32770 / 0x8002 as **CAN FD PDO interrupt**. The observed startup log showed this error on node 3 after nodes 1 and 2 reported valid positions and 0..39 mm limits. This fault is distinct from an invalid position or limit. It means the drive reported interruption of its movement-command stream; the log alone does not identify the original reason the stream ended.

Initialization now reports separate hexadecimal drive faults and detailed position/limit errors. On explicit Arm only, a latched 0x8002 may enter a bounded recovery attempt: validate all three measured positions and limits, disable/re-enable the affected node, and stream holds at the measured positions. The bridge then checks all three fault registers up to three times before allowing simulator tracking. During verification, position holds continue through SDO waits. Other faults block startup immediately. If 0x8002 persists, startup fails and all drives are disabled; the bridge does not disable the watchdog, modify factory settings, or bypass the fault. The manual describes disable/re-enable recovery for minor faults, but successful recovery of this drive's PDO interruption still requires hardware confirmation.

The knee now holds its measured arming position with zero velocity feedforward while ankle startup is pending. Both joints start following simulator targets only after ankle startup is verified. This prevents the knee running the gait alone while the ankle connection is being checked.

After restarting the bridge, reload the simulator, press **Arm knee + ankle**, and wait for `ANKLE nodes 1/2/3: fault-free startup confirmed; simulator tracking enabled`. If recovery fails, support and stop the mechanism before inspecting/restarting the affected drive. Re-arm explicitly afterward. Actual hardware recovery has not been verified by the automated tests.


### Ankle node 3 disappears after startup (October 25, 2026)

The restored two-adapter setup is retained. The knee reports healthy tracking while an ankle node 3 fault-register read (2010:05) times out after three 250 ms attempts; only heartbeat IDs 0x701 and 0x702 are observed during that failure. Green power indicators do not establish that node 3's CAN interface is communicating. The user confirmed PCAN/knee and RH02/ankles are on electrically separate buses. The cause is still unverified.

The ankle driver now enables RH02 CAN error reporting with `ME` after its existing startup sequence and prints `ANKLE adapter report` messages. It records bounded link history, last heartbeat and SDO-reply times per node, and position-command write gaps. If all read attempts fail, `ANKLE LINK` lines show this evidence before the existing timeout stops both systems. A position TX timestamp measures the host write, not delivery or drive acceptance. No error reports received is not proof that the physical bus was error-free. Heartbeat ages are diagnostic only; they do not replace SDO feedback.

No knee code, control gains, actuator motion format, speed/smoothing settings, poll rate, retry budget, persistent motor parameters or automatic recovery behavior changed. This is additional failure evidence, not a verified repair. On the next occurrence retain the `ANKLE adapter report` and `ANKLE LINK` lines together with the stop message. Long command gaps suggest host/USB scheduling; recent commands with missing node-3 replies and heartbeats point toward the node-3 drive or its bus branch, without proving the root cause.
