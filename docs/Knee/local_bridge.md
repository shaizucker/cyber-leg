# Local simulator bridge

Current update: ordinary bridge startup now uses the measured small-move profile (position Kp/Kd 40/10, velocity Kp/Kd/Ki 50/0/0.001). Earlier references below to default gains describe the original CLI baseline. Active gains are shown in the browser. Port 8000 is bound exclusively to prevent overlapping Windows bridge instances. Browser target updates are requested at 50 Hz (actual rate depends on HTTP and rendering); CAN remains nominally 100 Hz. `/api/telemetry` returns the latest 6,000 actual motor-feedback samples with monotonic timestamps, goal and command, including fault/status. This buffer is in memory and is cleared by restarting the process.

These changes do not remove the 1 rad/s command limit or manual trajectory restart lag, and do not establish tracking performance for large moves or automatic gait. Larger-range validation remains required.

Servo gains can be selected when launching the bridge: `--kp-pos 40 --kd-pos 5 --kp-vel 50 --kd-vel 0 --ki-vel 0.005` reproduces the CLI baseline. Changes require a restart and explicit re-arm. These are position PD and velocity PID parameters, not a single PID loop. The ranges enforced are protocol encoding ranges, not verified safe tuning ranges for the assembled leg. Requested gains are printed on startup and quantized to 8-bit fields by the driver. No flash parameters are written.

For tuning evidence, record the same small manual move and the following hold, including `TRACK` lines before, during and after motion. Their monotonic timestamps include milliseconds; summaries remain at 4 Hz and may miss fast oscillation. A stationary excerpt alone cannot establish overshoot or settling time. Preserve the baseline and change one gain per comparison after assessing the move data.

Run the simulator and bridge on the Windows PC connected to PEAK-CAN:

```powershell
python drivers\knee_bridge.py
```

Open **http://127.0.0.1:8000/**. The bridge serves the local simulator and opens the PCAN bus only when Arm motor is pressed. Python needs `python-can` and the installed PEAK PCAN driver. Do not run the CLI or another program on the same channel at the same time. Ctrl+C stops the bridge and attempts to disarm.

The bridge terminal prints each CAN frame as `TX` or `RX` with its ID, DLC, and hexadecimal bytes, plus send errors and response timeouts. `TX` means the adapter accepted the frame; it does not prove that the motor received it. At 100 Hz the trace is verbose. Logging runs on a separate thread and reports dropped log lines if terminal output cannot keep up. Restart the bridge after updating its code so the running process uses the new version.

Before arming, verify the physical reference position and direction on the hardware. By default simulator zero maps to motor zero and positive simulator flexion maps to positive motor angle. To configure the mapping, for example:

```powershell
python drivers\knee_bridge.py --sign -1 --offset-deg 0
```

The mapping is `motor angle = sign * simulator knee flexion + offset`. These settings do not zero the motor; use the CLI's disarmed zero procedure if needed. Confirm the real knee's mechanical limits separately; the simulator's 0–110° range is not a verified physical limit. The bridge refuses to arm when the measured position is outside the mapped simulator range, allowing a one-degree margin for measurement and zeroing.

Alternatively, stop and disarm in the simulator, place the physical knee at its zero reference pose, and press **Set current position as zero**. The bridge sends Rest, checks for faults, sends Zero, and verifies a subsequent Rest reply within 0.5 degrees of zero. The motor stays disarmed. This requires `--offset-deg 0` and does not save calibration to flash. If the free shaft moves afterward, the measured angle changes; check its pose before arming.

Arming sends Rest, checks fault and position, initializes the target from the measured angle, enters Servo Motor State, and streams at 100 Hz with the CLI's gains. It limits setpoint movement to 1 rad/s, so a faster simulation or a large initial mismatch will produce lag. Servo velocity limit is 2.5 rad/s. No exact tracking performance has been established. Motor feedback appears in the simulator panel in motor coordinates.

Simulator Stop holds the current motor setpoint. Reset, switching mode, hiding the tab, and Disarm request Rest; loss of browser updates for 0.5 seconds or missing/faulted motor feedback also disarms. After a disconnect or fault the motor must be armed again explicitly. If disarm communication fails, the drive's documented 1000 ms CAN watchdog provides a separate fallback; verify actual behavior during supervised hardware testing. The simulator is a kinematic visualization, not a validated control system.

Run the simulated-motor checks with `python -m unittest discover -s drivers -p test_knee_bridge.py -v`.

Manual slider moves now use the CLI's minimum-jerk profile (at least 0.6 seconds, peak speed at most 1 rad/s). A changed target replans from the current commanded position; repeated identical browser updates do not restart the profile. Continuous slider dragging can still replan frequently, and physical tracking requires hardware validation. Automatic gait retains the existing speed-limited tracking. The terminal also prints `TRACK` summaries four times per second with goal, command, measured angle, velocity, torque, and fault for tuning. If the command is steady but the measured angle repeatedly crosses it, smoothing alone has not resolved the physical oscillation.
