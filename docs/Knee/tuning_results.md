# Supervised bench tuning — October 18, 2026

User confirmed supported, unworn hardware and a physical test range of 0–10 degrees. One controller owned CAN at a time. Each trial checked Rest, armed from measured position, recorded nominal 100 Hz feedback, and returned to Rest. Guards aborted on missing feedback, motor faults, scheduling gaps above 100 ms, measured position outside 0–9 degrees, speed above 20 degrees/s, or reported torque above 4 Nm. These software guards do not guarantee a physical stopping distance.

Initial measured angle: 0.54 degrees. A three-second ramp to 5 degrees with Ki 0.001 and Kd 5 held at 4.93 degrees without observed oscillation during the subsequent five seconds.

The comparisons below used 0.6-second minimum-jerk ramps between approximately 5 and 7 degrees and five-second holds. Position Kp 40, velocity Kp 50, velocity Kd 0, velocity Ki 0.001, and servo velocity ceiling 2.5 rad/s were fixed. Position Kd was the only changed gain. Requested values are quantized into the motor's 8-bit CAN fields.

| Position Kd | Upward overshoot | Downward undershoot | Up/down settling after ramp |
| --- | --- | --- | --- |
| 5 | 0.814 degrees | 0.814 degrees | 1.009 / 0.993 s |
| 8 | 0.464 degrees | 0.246 degrees | 0.822 / 0.603 s |
| 10 | 0.202 degrees | 0.202 degrees | 0.541 / 0.541 s |

Settling means entry into a ±0.2-degree band maintained through the remainder of the recorded hold, with at least one second of data remaining. Values describe sampled feedback, not guaranteed continuous peaks. All trials completed without faults or guard trips. Kd 10 final positions were 7.027 and 4.929 degrees; peak absolute reported torque was 1.38 Nm upward and 1.62 Nm downward. The final Rest reply confirmed fault 0 at 4.93 degrees.

Selected provisional bench profile: position Kp/Kd 40/10, velocity Kp/Kd/Ki 50/0/0.001. Launch with `drivers/start_tuned_bridge.ps1`, or `python drivers/knee_bridge.py --kd-pos 10 --ki-vel 0.001` from the project root. Original bridge defaults are retained for comparison. No motor flash settings were written.

Raw records: `drivers/tuning_ki001_trial1.csv`, `drivers/tuning_ki001_up.csv`, `drivers/tuning_ki001_down.csv`, `drivers/tuning_kd8_up.csv`, `drivers/tuning_kd8_down.csv`, `drivers/tuning_kd10_up.csv`, `drivers/tuning_kd10_down.csv`. These are individual trials, not repeated statistical validation. Larger moves, loaded operation, return to zero, and automatic gait remain untested with this profile. The original 7-degree log is not a controlled baseline comparison because the trajectory and initial conditions differ.
