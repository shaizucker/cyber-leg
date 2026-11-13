# Leons-UI: activity modes, smoother gait, energy view

Branch `Leons-UI`, based on `nirbadt/cyber-leg` `smooth-gait` (`56a3796`, gait v1),
which itself sits on `shaizucker/cyber-leg` `feature/hip` (`8fba314`).
Written 2026-11-13.

## What changed

| Area | Change |
|---|---|
| Gait (browser) | New activity engine `dist/gait.js`: joint-space C2 splines, moving pelvis, belt-locked feet. Replaces the v1 fixed-hip IK in `motion.js`. |
| Modes | **Walk**, **Stairs** (ascend / descend), **Sit** (sit-to-stand cycle), plus **stumble recovery** inside Walk. Manual mode is unchanged. |
| Hardware smoothness | `drivers/knee_bridge.py` extrapolates hip/knee targets along their velocity between browser packets instead of braking and jumping. |
| UI | Activity panel with gait tuning, telemetry tabs (Ankle / Joints / Energy), a motor-limit badge and a fixed-hip-mount bench view. |
| Energy | `dist/energy.js`: live mechanical power and work per joint from motor feedback. |
| Tools | `drivers/sim_bridge.py`: the real bridge driving simulated motors. `scripts/check_activities.mjs`, `scripts/check_energy.mjs`, `scripts/tune_profile.mjs`. |

## Gait engine (`dist/gait.js`)

- **Joint curves.**
  - Hip, knee and ankle are periodic C2 cubic splines through keyframes in anatomical angles: thigh from vertical, knee flexion, ankle dorsiflexion.
  - They are converted to rig command angles with the CAD's built-in thigh and shank lean (3.2° and 1.1°).
  - Commanded joint speed therefore never jumps.
  - Walking amplitudes shrink at low belt speed; knee swing is kept for clearance.
- **Pelvis.**
  - The prosthetic foot drives the pelvis for its whole stance, so its sole contact point moves exactly with the belt (measured error < 10 mm/s).
  - The sound leg drives it while the prosthesis swings. Handovers are 4 % C2 ramps placed while the prosthetic foot is in the air.
  - The displayed sound foot is held on the ground by rolling contact, so neither foot slides in double support.
  - Vertically, walking rests on whichever sole needs the pelvis highest (a smooth maximum over the real CAD sole vertices), so stones and uneven ground lift the body.
- **Cadence follows the curves.** The stride is how far the loaded foot travels relative to the pelvis per cycle: about 730 mm at 0.3 m/s and 890 mm at 0.6 m/s, so cadence = speed / stride.
- **Drift.** It is corrected by nudging the ground (belt or staircase), never the body, so planted feet never slip.
- **Stairs.**
  - Step-over-step on a moving staircase (like a stair-mill), so the leg stays in view.
  - Rise and run are derived from the profiles: ascent 160 × 293 mm, descent 142 × 244 mm.
  - Stair keyframes started from published patterns and were adjusted with `tune_profile.mjs` for ≥ 20 mm toe clearance over the nosing with this toe-less foot.
- **Sit / stand.**
  - Stand 1.2 s → sit down 2.6 s → seated (adjustable) → stand up 2.2 s.
  - Feet stay flat and planted; the ankle follows the shank; the trunk leans forward.
  - The chair is placed from the seated pose (seat about 400 mm high).
- **Stumble recovery.** Trip now, or random trips (about 1 in 3 strides):
  - **Early swing (elevating strategy):** C2 bumps add knee +28°, hip +10° and ankle +6° over 0.32 s to lift the foot over the obstacle.
  - **Late swing (lowering strategy):** the swing finishes 1.6× faster with a shorter, straighter step.
  - A red obstacle is drawn on the belt at the toe.
- **Tuning.** Swing clearance, push-off strength and stance knee flexion are sliders in the Walk panel. Keyframes are in `PROFILES` in `gait.js`.

### Motor limits

Hip commands stay inside the bridge's configured range, currently −30…30°. Stair ascent (hip up to about 62°) and sitting (about 85°) go beyond it.

- The 3D model shows the full target pose.
- The bridge receives the command clamped to the limit, with zero velocity feed-forward at the stop.
- The viewport shows a **MOTOR LIMIT** badge, and the Joints tab plots target (dashed) and command (solid).
- On the bench, these activities therefore move the hip only to 30°.

Widening the hip range is a hardware and safety decision, not a UI one.

### Fixed hip mount (bench view)

Holds the pelvis still, like the bench rig. The joint commands are unchanged; only the visualisation stops tracking the ground. Use it to compare the simulator with the physical bench (handoff issue 4).

## Bridge: velocity extrapolation (`knee_bridge.py`)

Before this change, if a browser packet arrived more than 40 ms late, the bridge kept the old position target and set velocity to zero. The motor braked, and then the next packet jumped the target by several degrees (handoff issue 3).

Now, in auto mode while running:

1. **Up to 40 ms after the last packet:** the reference continues at the browser's velocity.
2. **From 40 ms:** the velocity decays with a 40 ms time constant, so the reference eases to a stop. Total extrapolation is at most v × 80 ms.
3. **From 250 ms:** the reference is held with zero velocity.
4. **At any time:** extrapolated targets are clamped to the signed command limits, and velocity is zeroed at a limit.

The 500 ms stale-update watchdog, fault handling and arming checks are unchanged. Manual mode and Stop never extrapolate.

New tests are in `test_knee_bridge.py`: smooth late-packet extrapolation, the limit clamp, and no extrapolation when stopped or in manual. Three existing assertions now allow for the extrapolated position.

Generating the gait inside the bridge on a fixed clock is still the long-term fix.

## Energy panel (`dist/energy.js`)

- Mechanical power per output, as reported by the bridge console: rotary torque × angular speed, and actuator force × speed (ankle = sum of nodes 1–3).
- Positive work = delivered by the motor; negative = absorbed.
- Shows live W, cumulative +J / −J, and the average per stride over the last 5 strides.
- **This is not electrical consumption.** The drives report no DC bus current. Electrical power and Wh appear only for devices with an external DC sensor posting to `/api/power` (see `docs/Motors/online_configuration.md`).
- Stale or missing feedback shows "—" and is never counted as zero.

## Running without hardware

```bash
python drivers/sim_bridge.py            # real bridge code, simulated knee + hip (no ankle)
# open http://127.0.0.1:8000/, pick an activity, Start, then Arm
```

Feedback from `sim_bridge.py` comes from a simple joint model (inertia, friction, gravity-like load). Never use it to tune real hardware.

## Checks

```bash
cd CYBER_LEG_V01_Developer_Handoff/CYBER_LEG_V01
node scripts/check_activities.mjs   # gait/stairs/sit/stumble geometry, slip, clearance, limits (real CAD sole)
node scripts/check_energy.mjs       # power integration and bookkeeping
cd ../../drivers && python -m unittest discover -s . -p test_knee_bridge.py   # + the other hardware-free suites
```

All pass on this branch. `drivers/test_motor_console_ui.cjs` times out in a headless software-GL browser both on this branch and on `smooth-gait` (rendering is about 2 fps, so updates go stale). Run it on a machine with a GPU.

## Not yet verified

- None of this has run on the real leg. Test disarmed first, then armed at 0.3 m/s, unloaded, with the bench clear, for each activity.
- Stair and sit hip targets exceed the current ±30° hip range (see Motor limits).
- `gait.js` and Nir's unpublished gait v2 solve the same problem. Choose one before merging.
