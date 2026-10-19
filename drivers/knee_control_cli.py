"""
Interactive CLI for Cyber-Leg Knee Motor Control (Motorevo + PEAK-CAN)

While armed, a background thread streams the servo command at 100 Hz.
Moves follow a minimum-jerk trajectory so the setpoint changes smoothly
instead of stepping to the final angle.
The driver's CAN watchdog (CAN COM Timeout = 1000 ms) latches Fault 16 and
drops to Rest State if command frames stop, so the stream must never pause
while the motor is in Motor State.
"""

import sys
import time
import math
import threading
from motorevo_driver import MotorevoMotor

LOOP_DT = 0.01
MAX_TRAJ_VEL = 1.0      # peak output-shaft speed of a planned move [rad/s]
MIN_MOVE_TIME = 0.6     # [s]
# Servo-loop velocity limit must stay above MAX_TRAJ_VEL so it never clips the profile.
VEL_LIMIT = 2.5
KP_POS, KD_POS = 40.0, 5.0
KP_VEL, KD_VEL, KI_VEL = 50.0, 0.0, 0.005


def min_jerk(tau: float) -> float:
    tau = min(max(tau, 0.0), 1.0)
    return tau ** 3 * (10.0 - 15.0 * tau + 6.0 * tau ** 2)


def print_telemetry(telem: dict):
    if not telem:
        print("[!] No response from motor.")
        return
    pos_rad = telem['position_rad']
    pos_deg = math.degrees(pos_rad)
    vel = telem['velocity_rad_s']
    tau = telem['torque_nm']
    temp = telem['temp_c']
    status = telem['status_str']
    fault = telem['fault_desc']
    print(f"[{status}] Pos: {pos_deg:+6.1f}° ({pos_rad:+6.3f} rad) | Vel: {vel:+5.2f} r/s | Tau: {tau:+5.2f} Nm | T: {temp}°C | Fault: {fault}")


class KneeController:
    def __init__(self, motor: MotorevoMotor):
        self.motor = motor
        self.lock = threading.Lock()
        self.armed = False
        self.target_pos = 0.0       # final goal of the current move
        self.setpoint = 0.0         # instantaneous command sent to the drive
        self._traj = None           # (start_pos, end_pos, start_time, duration)
        self.telem = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._thread.start()

    def _update_setpoint(self, now: float):
        if self._traj is None:
            return
        p0, p1, t0, dur = self._traj
        s = min_jerk((now - t0) / dur)
        self.setpoint = p0 + (p1 - p0) * s
        if s >= 1.0:
            self._traj = None

    def _stream_loop(self):
        next_t = time.perf_counter()
        while not self._stop.is_set():
            with self.lock:
                if self.armed:
                    self._update_setpoint(time.perf_counter())
                    t = self.motor.send_servo_command(
                        target_pos_rad=self.setpoint, vel_limit_rad_s=VEL_LIMIT,
                        kp_pos=KP_POS, kd_pos=KD_POS,
                        kp_vel=KP_VEL, kd_vel=KD_VEL, ki_vel=KI_VEL,
                        timeout=0.01)
                    if t:
                        self.telem = t
                        if t['status'] == 0:
                            self.armed = False
                            self._traj = None
                            print(f"\n[!] Motor dropped to Rest State (fault {t['fault']}: {t['fault_desc']}). Press Enter.")
            next_t += LOOP_DT
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.perf_counter()

    def _cmd(self, fn):
        with self.lock:
            t = fn()
            if t:
                self.telem = t
            return t

    def read(self):
        if self.armed:
            return self.telem
        return self._cmd(self.motor.enter_rest_mode)

    def clear_faults(self):
        return self._cmd(self.motor.enter_rest_mode)

    def arm(self):
        t = self.clear_faults()
        if not t:
            return None
        if t['fault'] != 0:
            # Rest command did not clear the fault; the driver will refuse Motor State.
            return t
        with self.lock:
            self.target_pos = self.setpoint = t['position_rad']
            self._traj = None
            t = self.motor.enter_motor_mode()
            if t:
                self.telem = t
                self.armed = t['status'] != 0
        return t

    def disarm(self):
        with self.lock:
            self.armed = False
            t = self.motor.enter_rest_mode()
            if t:
                self.telem = t
            return t

    def zero(self):
        if self.armed:
            print("[!] Disarm before zeroing.")
            return None
        return self._cmd(self.motor.set_zero_position)

    def move_to(self, pos_rad: float):
        if not self.armed:
            print("[!] Motor is not armed. Use [m] first.")
            return None
        with self.lock:
            start = self.setpoint
            dist = abs(pos_rad - start)
            # Peak velocity of a min-jerk profile is 1.875 * dist / duration.
            dur = max(MIN_MOVE_TIME, 1.875 * dist / MAX_TRAJ_VEL)
            self.target_pos = pos_rad
            self._traj = (start, pos_rad, time.perf_counter(), dur)
        deadline = time.time() + dur + 1.0
        while time.time() < deadline and self.armed:
            t = self.telem
            if self._traj is None and t and abs(t['position_rad'] - pos_rad) < math.radians(1.0) \
                    and abs(t['velocity_rad_s']) < 0.05:
                break
            time.sleep(0.05)
        return self.telem

    def close(self):
        self.disarm()
        self._stop.set()
        self._thread.join(timeout=0.5)
        self.motor.close()


def main():
    print("=" * 60)
    print(" Cyber-Leg Knee Actuator Controller (Peak-CAN @ 1 Mbps)")
    print("=" * 60)

    try:
        motor = MotorevoMotor(motor_id=1, interface="pcan", channel="PCAN_USBBUS1", bitrate=1000000)
    except Exception as e:
        print(f"[ERROR] Failed to open Peak-CAN: {e}")
        sys.exit(1)

    ctl = KneeController(motor)

    telem = ctl.clear_faults()
    print("Initial Motor Status:")
    print_telemetry(telem)
    if telem and telem['fault'] != 0:
        print(f"[!] Fault {telem['fault']} is latched and was not cleared by Enter Rest State.")
        print("    The motor will refuse to arm. Power-cycle the 48 V supply, then restart this CLI.")

    menu = """
Commands:
  [r] Read live telemetry
  [c] Clear faults (Enter Rest State)
  [z] Zero current position (set current angle to 0.0 rad, disarmed only)
  [m] Arm motor (Enter Motor Mode, holds current angle)
  [s] Safe stop / Disarm (Enter Rest Mode)
  [g] Go to target angle (in degrees)
  [+] Nudge +10 degrees
  [-] Nudge -10 degrees
  [0] Return to 0 degrees
  [q] Quit
"""
    print(menu)

    try:
        while True:
            cmd = input("cmd> ").strip().lower()
            if not cmd:
                continue

            if cmd == 'r':
                print_telemetry(ctl.read())

            elif cmd == 'c':
                print("[*] Sending Enter Rest State to clear faults...")
                print_telemetry(ctl.clear_faults())

            elif cmd == 'z':
                print("[*] Setting current position to 0.0 rad...")
                print_telemetry(ctl.zero())

            elif cmd == 'm':
                print("[*] Arming motor (Enter Motor Mode)...")
                t = ctl.arm()
                print_telemetry(t)
                if t and not ctl.armed:
                    print(f"[!] Motor did not arm (status {t['status']}, fault {t['fault']}: {t['fault_desc']}).")
                    if t['fault'] != 0:
                        print("    Fault could not be cleared over CAN. Power-cycle the 48 V supply.")
                elif ctl.armed:
                    print("[*] Armed. Holding current angle; blue LED should be on.")

            elif cmd == 's':
                print("[*] Disarming motor (Enter Rest Mode)...")
                print_telemetry(ctl.disarm())

            elif cmd == '0':
                print("[*] Moving to 0.0°...")
                print_telemetry(ctl.move_to(0.0))

            elif cmd in ('+', '-'):
                step = math.radians(10.0) * (1 if cmd == '+' else -1)
                target = ctl.target_pos + step
                print(f"[*] Moving to {math.degrees(target):.1f}°...")
                print_telemetry(ctl.move_to(target))

            elif cmd == 'g':
                val_str = input("Enter target angle in degrees: ").strip()
                try:
                    deg = float(val_str)
                except ValueError:
                    print("[!] Invalid angle value.")
                    continue
                print(f"[*] Moving to {deg:.1f}° ({math.radians(deg):.3f} rad)...")
                print_telemetry(ctl.move_to(math.radians(deg)))

            elif cmd == 'q':
                print("Exiting...")
                break

            else:
                print(menu)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        print("[*] Ensuring motor is disarmed...")
        ctl.close()
        print("Done.")


if __name__ == "__main__":
    main()
