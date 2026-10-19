"""Single supervised bench trial; never run alongside the browser bridge."""
import argparse
import csv
import json
import math
import time
from pathlib import Path
from knee_control_cli import min_jerk
from motorevo_driver import MotorevoMotor
from knee_trajectory import KneeTrajectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ki', type=float, default=0.001)
    parser.add_argument('--kd', type=float, default=5)
    parser.add_argument('--output', required=True)
    parser.add_argument('--target', type=float, default=5)
    parser.add_argument('--duration', type=float, default=3)
    parser.add_argument('--standalone', action='store_true', help='unattached motor, no mechanical stops')
    parser.add_argument('--continuous', action='store_true')
    parser.add_argument('--kp', type=float, default=40)
    parser.add_argument('--velocity-kp', type=float, default=50)
    args = parser.parse_args()
    if not (0 <= args.target <= 70 if args.standalone else 3 <= args.target <= 7) or not 0.6 <= args.duration <= 3:
        parser.error('Target or ramp duration outside trial range')
    if not 0 <= args.ki <= 0.005 or not 0 <= args.kd <= 10:
        parser.error('Trial gain outside restricted tuning range')
    if not 5 <= args.kp <= 60 or not 10 <= args.velocity_kp <= 80:
        parser.error('Gain outside trial range')
    rows = []
    motor = None
    error = None
    target = math.radians(args.target)
    try:
        motor = MotorevoMotor()
        fb = motor.enter_rest_mode()
        if not fb or fb['status'] != 0 or fb['fault']:
            raise RuntimeError(f'Rest check failed: {fb}')
        start = fb['position_rad']
        if not (-10 <= math.degrees(start) <= 80 if args.standalone else 0 <= math.degrees(start) <= 8):
            raise RuntimeError(f'Start angle outside trial range: {math.degrees(start):.3f}')
        fb = motor.enter_motor_mode()
        if not fb or fb['status'] != 1 or fb['fault']:
            raise RuntimeError(f'Arm failed: {fb}')
        t0 = time.monotonic()
        deadline = t0
        previous = t0
        planner = KneeTrajectory(start)
        while True:
            now = time.monotonic()
            elapsed = now - t0
            if elapsed >= args.duration + 5:
                break
            if now - previous > 0.1:
                raise RuntimeError('Control scheduling gap exceeded 100 ms')
            dt = min(max(now - previous, 0.0001), 0.02)
            previous = now
            command = start + (target - start) * min_jerk(elapsed / args.duration)
            if args.continuous:
                command = planner.advance(target, dt)
            fb = motor.send_servo_command(command, vel_limit_rad_s=2.5,
                                          kp_pos=args.kp, kd_pos=args.kd, kp_vel=args.velocity_kp,
                                          kd_vel=0, ki_vel=args.ki, timeout=0.01)
            if not fb:
                raise RuntimeError('Feedback timeout')
            row = dict(t=elapsed, command_deg=math.degrees(command),
                       actual_deg=math.degrees(fb['position_rad']),
                       velocity_deg_s=math.degrees(fb['velocity_rad_s']),
                       torque_nm=fb['torque_nm'], fault=fb['fault'], status=fb['status'])
            rows.append(row)
            if fb['fault'] or fb['status'] != 1:
                raise RuntimeError(f'Motor fault/state changed: {fb}')
            if not (-15 <= row['actual_deg'] <= 85 if args.standalone else 0 <= row['actual_deg'] <= 9):
                raise RuntimeError(f'Position guard reached: {row}')
            if abs(row['velocity_deg_s']) > (160 if args.standalone else 20) or abs(row['torque_nm']) > 4:
                raise RuntimeError(f'Velocity/torque guard reached: {row}')
            deadline += 0.01
            time.sleep(max(0, deadline - time.monotonic()))
    except Exception as exc:
        error = str(exc)
    finally:
        if motor:
            try:
                print('Final Rest:', motor.enter_rest_mode(), flush=True)
            finally:
                motor.close()
        if rows:
            with Path(args.output).open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
    hold = [r for r in rows if r['t'] >= args.duration]
    summary = dict(error=error, samples=len(rows), ki=args.ki, kd=args.kd)
    if hold:
        summary.update(hold_min_deg=min(r['actual_deg'] for r in hold),
                       hold_max_deg=max(r['actual_deg'] for r in hold),
                       final_deg=hold[-1]['actual_deg'],
                       peak_abs_torque_nm=max(abs(r['torque_nm']) for r in rows))
        last_bad = max((i for i, r in enumerate(hold) if abs(r['actual_deg'] - args.target) > 0.2), default=-1)
        settled = hold[last_bad + 1:] if last_bad + 1 < len(hold) else []
        summary['settling_seconds_after_ramp'] = settled[0]['t'] - args.duration if settled and settled[-1]['t'] - settled[0]['t'] >= 1 else None
    print(json.dumps(summary, indent=2), flush=True)
    if error:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
