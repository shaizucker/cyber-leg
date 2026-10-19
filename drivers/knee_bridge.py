"""Local simulator-to-knee bridge. Run with --help for options."""
import argparse
import json
import math
import queue
import secrets
import socket
import threading
import time
from collections import deque
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit
from knee_control_cli import min_jerk, MIN_MOVE_TIME, MAX_TRAJ_VEL
from knee_trajectory import KneeTrajectory

DT = 0.01
STALE_SECONDS = 0.5
MAX_SPEED = 2.0
SIM_MAX = math.radians(70)
SERVO_BASELINE = dict(kp_pos=40.0, kd_pos=5.0, kp_vel=50.0, kd_vel=0.0, ki_vel=0.005)
SERVO_DEFAULTS = dict(kp_pos=40.0, kd_pos=10.0, kp_vel=50.0, kd_vel=0.0, ki_vel=0.001)
SERVO_MAX = dict(kp_pos=250.0, kd_pos=50.0, kp_vel=250.0, kd_vel=50.0, ki_vel=0.05)


def validate_gains(gains):
    if set(gains) != set(SERVO_DEFAULTS):
        raise ValueError('Provide all five servo gains')
    for name, value in gains.items():
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or not 0 <= value <= SERVO_MAX[name]:
            raise ValueError(f'{name} must be finite and between 0 and {SERVO_MAX[name]}')
    return dict(gains)


DIST = Path(__file__).resolve().parents[1] / "CYBER_LEG_V01_Developer_Handoff" / "CYBER_LEG_V01" / "dist"


class CanTrace:
    """Write diagnostics off the control thread; drop log lines if output stalls."""
    def __init__(self):
        self.queue = queue.Queue(maxsize=4096)
        self.dropped = 0
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()

    def __call__(self, event, arbitration_id, data, detail):
        stamp = time.strftime('%H:%M:%S')
        raw = data.hex(' ').upper()
        line = f'{stamp} {event:8} ID=0x{arbitration_id:03X} DLC={len(data)} {raw} {detail}'.rstrip()
        self.message(line)

    def message(self, line):
        try:
            self.queue.put_nowait(line)
        except queue.Full:
            self.dropped += 1

    def _write(self):
        while True:
            line = self.queue.get()
            if self.dropped:
                count = self.dropped
                self.dropped = 0
                print(f'CAN log: {count} lines dropped because terminal output was slow', flush=True)
            print(line, flush=True)


class KneeBridge:
    def __init__(self, motor_factory, sign=1, offset=0, log=None, gains=None):
        self.gains = validate_gains(SERVO_DEFAULTS if gains is None else gains)
        self.motor_factory, self.sign, self.offset = motor_factory, sign, offset
        self.log = log or (lambda message: None)
        self.lock = threading.RLock()
        self.motor = None
        self.armed = False
        self.goal = None
        self.running = False
        self.mode = 'manual'
        self._traj = None
        self.planner = None
        self._last_diagnostic = 0.0
        self.setpoint = None
        self.feedback = None
        self.samples = deque(maxlen=6000)
        self.last_update = 0.0
        self.error = None
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def update(self, angle_deg, running, mode='manual'):
        if isinstance(angle_deg, bool) or not isinstance(angle_deg, (int, float)) or not math.isfinite(angle_deg) or not 0 <= angle_deg <= 70:
            raise ValueError("Knee angle must be between 0 and 70 degrees")
        if not isinstance(running, bool):
            raise ValueError("Running state must be a boolean")
        if mode not in ('manual', 'auto'):
            raise ValueError('Unknown simulator mode')
        with self.lock:
            old_goal, old_mode = self.goal, self.mode
            self.mode = mode
            self.running = running
            self.goal = (self.sign * math.radians(angle_deg) + self.offset
                         if running or not self.armed else self.setpoint)
            self.last_update = time.monotonic()
            if not running:
                self._traj = None
                if self.planner and self.setpoint is not None:
                    self.planner.reset(self.setpoint)
            elif self.armed and (self.goal != old_goal or mode != old_mode):
                self._plan_manual_move()

    def _plan_manual_move(self):
        # Retain position and velocity history when targets change in either mode.
        self._traj = None
        if self.planner is None:
            self.planner = KneeTrajectory(self.setpoint, max_speed=MAX_SPEED)

    def _advance_setpoint(self, now, elapsed):
        if self.planner is None:
            self._plan_manual_move()
        self.setpoint = self.planner.advance(self.goal, min(max(elapsed, 0.0001), DT * 2))

    def arm(self):
        with self.lock:
            if self.armed:
                return
            if self.goal is None or time.monotonic() - self.last_update > STALE_SECONDS:
                raise ValueError("Simulator updates are not current")
            if not -12.5 <= self.goal <= 12.5:
                raise ValueError("Mapped target is outside the motor range")
            try:
                self.motor = self.motor_factory()
                t = self.motor.enter_rest_mode()
                if not t or t['fault'] != 0 or t['status'] != 0:
                    raise RuntimeError("Rest state failed or a fault is present")
                lo, hi = sorted((self.offset, self.offset + self.sign * SIM_MAX))
                if not lo - math.radians(1) <= t['position_rad'] <= hi + math.radians(1):
                    raise RuntimeError(f"Measured position {math.degrees(t['position_rad']):.2f} degrees is outside {math.degrees(lo):.1f} to {math.degrees(hi):.1f} degrees; disarm and zero at the reference pose")
                self.feedback = t
                self.setpoint = t['position_rad']
                self.planner = KneeTrajectory(self.setpoint, max_speed=MAX_SPEED)
                if not self.running:
                    self.goal = self.setpoint
                t = self.motor.enter_motor_mode()
                if not t or t['status'] != 1 or t['fault'] != 0:
                    raise RuntimeError("Motor did not enter servo mode without a fault")
                self.feedback = t
                self.armed = True
                if self.running:
                    self._plan_manual_move()
                self.error = None
            except Exception as exc:
                self.error = str(exc)
                self.log(f'Arm failed: {exc}')
                self._disarm_locked()
                raise

    def _disarm_locked(self):
        self.armed = False
        self._traj = None
        if self.motor:
            try:
                self.motor.close()
            finally:
                self.motor = None

    def zero(self):
        """Zero the current physical reference pose while confirmed disarmed."""
        with self.lock:
            if self.armed:
                raise ValueError('Disarm before zeroing')
            if self.running:
                raise ValueError('Stop the simulator before zeroing')
            if self.offset != 0:
                raise ValueError('Hardware zero requires --offset-deg 0')
            try:
                self.motor = self.motor_factory()
                t = self.motor.enter_rest_mode()
                if not t or t['status'] != 0 or t['fault'] != 0:
                    raise RuntimeError('Cannot zero: motor did not confirm fault-free Rest')
                self.motor.set_zero_position()
                # The zero-command reply can contain the previous position.
                time.sleep(0.05)
                t = self.motor.enter_rest_mode()
                self.feedback = t
                if not t or t['status'] != 0 or t['fault'] != 0 or abs(t['position_rad']) > math.radians(0.5):
                    raise RuntimeError(f'Zero verification failed: {t}')
                self.setpoint = self.goal = t['position_rad']
                self.error = None
                self.log(f"Zero verified: {math.degrees(t['position_rad']):.3f} degrees; motor remains disarmed")
            except Exception as exc:
                self.error = str(exc)
                self.log(f'Zero failed: {exc}')
                raise
            finally:
                self._disarm_locked()

    def disarm(self, reason=None):
        with self.lock:
            try:
                self._disarm_locked()
            finally:
                if reason:
                    self.error = reason

    def status(self):
        with self.lock:
            return {"armed": self.armed, "error": self.error, "feedback": self.feedback,
                    "setpoint_rad": self.setpoint, "goal_rad": self.goal,
                    "sign": self.sign, "offset_rad": self.offset, "gains": self.gains.copy()}

    def _loop(self):
        next_t = time.monotonic()
        last_t = next_t
        while not self.stop_event.is_set():
            now = time.monotonic()
            with self.lock:
                if self.armed:
                    try:
                        if now - self.last_update > STALE_SECONDS:
                            raise RuntimeError("Simulator connection lost; motor disarmed")
                        # Limit elapsed time as well as the rate after scheduling pauses.
                        self._advance_setpoint(now, now - last_t)
                        t = self.motor.send_servo_command(
                            self.setpoint, vel_limit_rad_s=2.5, **self.gains, timeout=0.01)
                        if not t:
                            raise RuntimeError("Motor feedback timed out")
                        self.feedback = t
                        self.samples.append(dict(t=time.monotonic(), goal_rad=self.goal,
                                                 command_rad=self.setpoint, **t))
                        if now - self._last_diagnostic >= 0.25:
                            self._last_diagnostic = now
                            self.log(f"TRACK t={now:.3f} goal={math.degrees(self.goal):+.2f} deg command={math.degrees(self.setpoint):+.2f} deg actual={math.degrees(t['position_rad']):+.2f} deg velocity={math.degrees(t['velocity_rad_s']):+.2f} deg/s torque={t['torque_nm']:+.3f} Nm fault={t['fault']}")
                        if t['status'] != 1 or t['fault'] != 0:
                            raise RuntimeError(f"Motor stopped or faulted: {t['fault_desc']}")
                    except Exception as exc:
                        try:
                            self._disarm_locked()
                        except Exception:
                            pass
                        self.error = str(exc)
                        self.log(f'Motor stopped: {exc}')
            last_t = now
            next_t += DT
            delay = next_t - time.monotonic()
            if delay <= 0:
                next_t = time.monotonic()
            else:
                self.stop_event.wait(delay)

    def close(self):
        self.stop_event.set()
        self.thread.join()
        self.disarm()


class BridgeServer(ThreadingHTTPServer):
    allow_reuse_address = False

    def server_bind(self):
        if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def make_handler(bridge, token, port):
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(DIST), **kwargs)

        def log_message(self, format, *args):
            pass

        def reply(self, code, data):
            body = json.dumps(data).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.headers.get('Host') != f'127.0.0.1:{port}':
                self.send_error(403)
            elif urlsplit(self.path).path == '/api/session':
                self.reply(200, {'token': token, **bridge.status()})
            elif urlsplit(self.path).path == '/api/telemetry':
                with bridge.lock:
                    samples = list(bridge.samples)
                self.reply(200, {'samples': samples})
            else:
                super().do_GET()

        def do_POST(self):
            if (self.headers.get('Host') != f'127.0.0.1:{port}' or
                self.headers.get('Origin') != f'http://127.0.0.1:{port}' or
                self.headers.get('X-Bridge-Token') != token or
                self.headers.get_content_type() != 'application/json'):
                self.send_error(403)
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 1024:
                    raise ValueError('Invalid request size')
                payload = json.loads(self.rfile.read(size))
                route = urlsplit(self.path).path
                if route == '/api/update':
                    bridge.update(payload['angle_deg'], payload['running'], payload.get('mode', 'manual'))
                elif route == '/api/arm':
                    bridge.arm()
                elif route == '/api/disarm':
                    bridge.disarm()
                elif route == '/api/zero':
                    bridge.zero()
                else:
                    self.send_error(404)
                    return
                self.reply(200, bridge.status())
            except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
                self.reply(400, {'error': str(exc)})
            except Exception as exc:
                self.reply(500, {'error': str(exc)})
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--sign', type=int, choices=(-1, 1), default=1, help='motor direction relative to simulator')
    parser.add_argument('--offset-deg', type=float, default=0, help='motor angle at zero simulator flexion')
    for name, default in SERVO_DEFAULTS.items():
        parser.add_argument('--' + name.replace('_', '-'), type=float, default=default,
                            help=f'servo gain, default {default}; protocol range 0..{SERVO_MAX[name]}')
    args = parser.parse_args()
    try:
        gains = validate_gains({name: getattr(args, name) for name in SERVO_DEFAULTS})
    except ValueError as exc:
        parser.error(str(exc))
    if not math.isfinite(args.offset_deg) or any(not -12.5 <= p <= 12.5 for p in (math.radians(args.offset_deg), math.radians(args.offset_deg) + args.sign * SIM_MAX)):
        parser.error('mapped knee range must fit within the motor position range')
    from motorevo_driver import MotorevoMotor
    trace = CanTrace()
    bridge = KneeBridge(partial(MotorevoMotor, motor_id=1, interface='pcan', channel='PCAN_USBBUS1', bitrate=1000000, trace=trace), args.sign, math.radians(args.offset_deg), log=trace.message, gains=gains)
    server = None
    try:
        try:
            server = BridgeServer(('127.0.0.1', args.port), make_handler(bridge, secrets.token_urlsafe(32), args.port))
        except OSError as exc:
            if getattr(exc, 'winerror', None) in (10048, 10013) or exc.errno in (98, 48):
                print(f'Cannot open port {args.port}: another bridge or application may already be using it. '
                      'Stop the existing bridge with Ctrl+C in its terminal, then retry. '
                      f'Alternatively, open http://127.0.0.1:{args.port}/ to use the running bridge.', flush=True)
                raise SystemExit(1) from None
            raise
        print(f'Open http://127.0.0.1:{args.port}/  (Ctrl+C to stop)', flush=True)
        print('CAN trace: TX = accepted by adapter; RX = received frame; TIMEOUT = no reply.', flush=True)
        print(f'Servo gains (requested, quantized to 8-bit CAN fields): {gains}', flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if server:
            server.server_close()
        bridge.close()


if __name__ == '__main__':
    main()
