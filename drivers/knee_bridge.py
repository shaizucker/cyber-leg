"""Local simulator-to-knee-and-ankle bridge. Run with --help for options."""
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
from knee_safety import read_configuration, validate_configuration

DT = 0.02
STALE_SECONDS = 0.5
MAX_VELOCITY = 10.0  # Configured P-T-M velocity encoding range, rad/s.
SIM_MAX_DEG = 105.0
# Match gemini-code-gui.py and the current MParameter Studio configuration.
THETA_MIN, THETA_MAX = 0.0, 1.83
SIM_MAX = THETA_MAX  # 105 degrees saturates at 1.83 rad, as in the working GUI.
PTM_DEFAULTS = dict(kp_pos=20.0, kd_pos=4.0)
PTM_MAX = dict(kp_pos=250.0, kd_pos=50.0)


def validate_mapping(sign, offset):
    if sign not in (-1, 1) or not math.isfinite(offset):
        raise ValueError('Invalid motor direction or offset')
    if any(not THETA_MIN <= p <= THETA_MAX for p in (offset, offset + sign * SIM_MAX)):
        raise ValueError('Mapped knee range must fit within the configured motor range 0..1.83 rad')


def validate_gains(gains):
    if set(gains) != set(PTM_DEFAULTS):
        raise ValueError('Provide both P-T-M position gains')
    for name, value in gains.items():
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or not 0 <= value <= PTM_MAX[name]:
            raise ValueError(f'{name} must be finite and between 0 and {PTM_MAX[name]}')
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
    def __init__(self, motor_factory, sign=1, offset=0, log=None, gains=None, ankle=None):
        validate_mapping(sign, offset)
        self.gains = validate_gains(PTM_DEFAULTS if gains is None else gains)
        self.motor_factory, self.sign, self.offset = motor_factory, sign, offset
        self.log = log or (lambda message: None)
        self.lock = threading.RLock()
        self.ankle = ankle
        self.motor = None
        self.armed = False
        self.goal = None
        self.running = False
        self.mode = 'manual'
        self.velocity = 0.0
        self._last_diagnostic = 0.0
        self.setpoint = None
        self.feedback = None
        self.samples = deque(maxlen=6000)
        self.last_update = 0.0
        self.error = None
        self.configuration = None
        self.zero_verification = None
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def update(self, angle_deg, running, mode='manual', velocity_deg_s=0.0, extensions_mm=None):
        if isinstance(angle_deg, bool) or not isinstance(angle_deg, (int, float)) or not math.isfinite(angle_deg) or not 0 <= angle_deg <= SIM_MAX_DEG:
            raise ValueError("Knee angle must be between 0 and 105 degrees")
        if not isinstance(running, bool):
            raise ValueError("Running state must be a boolean")
        if mode not in ('manual', 'auto'):
            raise ValueError('Unknown simulator mode')
        if isinstance(velocity_deg_s, bool) or not isinstance(velocity_deg_s, (int, float)) or not math.isfinite(velocity_deg_s):
            raise ValueError('Knee velocity must be finite degrees per second')
        with self.lock:
            if self.ankle:
                self.ankle.update(extensions_mm, running)
            self.mode = mode
            self.running = running
            self.goal = (self.sign * min(math.radians(angle_deg), SIM_MAX) + self.offset
                         if running or not self.armed else self.setpoint)
            self.last_update = time.monotonic()
            # Browser targets already contain the simulator's motion profile.
            # Sending them directly avoids adding a second, slower trajectory.
            velocity = self.sign * math.radians(velocity_deg_s) if running and mode == 'auto' else 0.0
            if ((angle_deg >= math.degrees(THETA_MAX) and velocity_deg_s > 0) or
                    (angle_deg == 0 and velocity_deg_s < 0)):
                velocity = 0.0
            self.velocity = max(-MAX_VELOCITY, min(MAX_VELOCITY, velocity))

    def _advance_setpoint(self, now, elapsed):
        if self.ankle and not self.ankle.status()['armed']:
            return  # Hold measured knee position until the ankle is ready too.
        self.setpoint = self.goal

    def _command_velocity(self, now):
        # A delayed browser update must not keep requesting motion indefinitely.
        if self.ankle and not self.ankle.status()['armed']:
            return 0.0
        return self.velocity if self.running and now - self.last_update <= DT * 2 else 0.0

    def _verify_motor_configuration(self):
        self.configuration = read_configuration(self.motor)
        validate_configuration(self.configuration, THETA_MIN, THETA_MAX)
        return self.configuration

    def arm(self):
        with self.lock:
            if self.armed:
                return
            if self.goal is None or time.monotonic() - self.last_update > STALE_SECONDS:
                raise ValueError("Simulator updates are not current")
            if not THETA_MIN <= self.goal <= THETA_MAX:
                raise ValueError("Mapped target is outside the motor range")
            if self.ankle:
                self.ankle.check_ready()
            try:
                self.motor = self.motor_factory()
                config = self._verify_motor_configuration()
                if self.zero_verification is not None:
                    if not self.zero_verification.get('command_confirmed'):
                        raise RuntimeError('Previous knee zero verification failed; verify zero before arming')
                    if abs(config['machine_offset_rad'] - self.zero_verification['machine_offset_after_rad']) > 1e-6:
                        raise RuntimeError('Knee machine zero changed since verification; verify the reference again')
                t = self.motor.enter_rest_mode()
                if not t or t['fault'] != 0 or t['status'] != 0:
                    raise RuntimeError("Rest state failed or a fault is present")
                lo, hi = sorted((self.offset, self.offset + self.sign * SIM_MAX))
                if not lo - math.radians(1) <= t['position_rad'] <= hi + math.radians(1):
                    raise RuntimeError(f"Measured position {math.degrees(t['position_rad']):.2f} degrees is outside {math.degrees(lo):.1f} to {math.degrees(hi):.1f} degrees; disarm and zero at the reference pose")
                self.feedback = t
                self.setpoint = t['position_rad']
                if not self.running:
                    self.goal = self.setpoint
                time.sleep(0.05)  # Match the working GUI's Rest-to-Motor delay.
                if time.monotonic() - self.last_update > STALE_SECONDS:
                    raise RuntimeError('Simulator updates became stale during knee configuration checks')
                t = self.motor.enter_motor_mode()
                if not t or t['status'] != 2 or t['fault'] != 0:
                    raise RuntimeError("Motor did not enter P-T-M mode (status 2) without a fault; check MParameter Studio control mode")
                self.feedback = t
                self.armed = True
                if self.ankle:
                    self.ankle.arm()
                self.error = None
            except Exception as exc:
                self.error = str(exc)
                self.log(f'Arm failed: {exc}')
                self._disarm_locked()
                raise

    def _disarm_locked(self):
        self.armed = False
        if self.ankle:
            self.ankle.disarm()
        self.velocity = 0.0
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
            self.zero_verification = dict(command_confirmed=False, physical_reference_verified=False,
                                          power_cycle_persistence_verified=False)
            try:
                self.motor = self.motor_factory()
                before = self._verify_motor_configuration()
                t = self.motor.enter_rest_mode()
                if (not t or t['status'] != 0 or t['fault'] != 0 or
                        not math.isfinite(t['position_rad']) or not math.isfinite(t['velocity_rad_s']) or
                        abs(t['velocity_rad_s']) > 0.05):
                    raise RuntimeError('Cannot zero: motor must confirm fault-free stationary Rest')
                previous_position = t['position_rad']
                acknowledgement = self.motor.set_zero_position()
                if not acknowledgement or acknowledgement['status'] != 0 or acknowledgement['fault'] != 0:
                    raise RuntimeError(f'Zero command was not acknowledged in fault-free Rest: {acknowledgement}')
                # Never trust the zero reply alone; it can contain old feedback.
                time.sleep(0.15)
                for _ in range(3):
                    t = self.motor.enter_rest_mode()
                    self.feedback = t
                    if (not t or t['status'] != 0 or t['fault'] != 0 or
                            not math.isfinite(t['position_rad']) or
                            abs(t['position_rad']) > math.radians(0.1) or
                            not math.isfinite(t['velocity_rad_s']) or abs(t['velocity_rad_s']) > 0.05):
                        raise RuntimeError(f'Zero verification failed: {t}')
                    time.sleep(0.05)
                after = self._verify_motor_configuration()
                if (abs(previous_position) > math.radians(0.1) and
                        abs(after['machine_offset_rad'] - before['machine_offset_rad']) <= 1e-6):
                    raise RuntimeError('Zero feedback is near zero but Machine Angle Offset did not change')
                self.setpoint = self.goal = t['position_rad']
                self.zero_verification.update(command_confirmed=True,
                    machine_offset_before_rad=before['machine_offset_rad'],
                    machine_offset_after_rad=after['machine_offset_rad'],
                    position_after_rad=t['position_rad'])
                self.error = None
                self.log(f"Zero command/readback confirmed: {math.degrees(t['position_rad']):.3f} degrees; "
                         'physical reference and power-cycle persistence still require verification; remains disarmed')
            except Exception as exc:
                self.error = str(exc)
                self.zero_verification['error'] = str(exc)
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
                    "motor_configuration": self.configuration, "zero_verification": self.zero_verification,
                    "setpoint_rad": self.setpoint, "goal_rad": self.goal,
                    "command_velocity_rad_s": self._command_velocity(time.monotonic()),
                    "control_mode": "P-T-M", "max_angle_deg": SIM_MAX_DEG,
                    "ankle": self.ankle.status() if self.ankle else {"enabled": False},
                    "sign": self.sign, "offset_rad": self.offset, "gains": self.gains.copy()}

    def _loop(self):
        next_t = time.monotonic()
        last_t = next_t
        while not self.stop_event.is_set():
            now = time.monotonic()
            with self.lock:
                if self.armed:
                    try:
                        if self.ankle and self.ankle.status()['error']:
                            raise RuntimeError(self.ankle.status()['error'])
                        if now - self.last_update > STALE_SECONDS:
                            raise RuntimeError("Simulator connection lost; motor disarmed")
                        self._advance_setpoint(now, now - last_t)
                        if not math.isfinite(self.setpoint) or not THETA_MIN <= self.setpoint <= THETA_MAX:
                            raise RuntimeError('Knee command outside the verified position range')
                        t = self.motor.send_ptm_command(
                            self.setpoint, vel_rad_s=self._command_velocity(now), kp=self.gains['kp_pos'],
                            kd=self.gains['kd_pos'], torque_ff_nm=0.0, timeout=0.01)
                        if not t:
                            raise RuntimeError("Motor feedback timed out")
                        self.feedback = t
                        self.samples.append(dict(t=time.monotonic(), goal_rad=self.goal,
                                                 command_rad=self.setpoint,
                                                 command_velocity_rad_s=self._command_velocity(now), **t))
                        if now - self._last_diagnostic >= 0.25:
                            self._last_diagnostic = now
                            self.log(f"TRACK t={now:.3f} goal={math.degrees(self.goal):+.2f} deg command={math.degrees(self.setpoint):+.2f} deg actual={math.degrees(t['position_rad']):+.2f} deg velocity={math.degrees(t['velocity_rad_s']):+.2f} deg/s torque={t['torque_nm']:+.3f} Nm fault={t['fault']}")
                        if t['status'] != 2 or t['fault'] != 0:
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
        try:
            self.disarm()
        finally:
            if self.ankle:
                self.ankle.close()


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
                    bridge.update(payload['angle_deg'], payload['running'], payload.get('mode', 'manual'), payload.get('velocity_deg_s', 0.0), payload.get('extensions_mm'))
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
    parser.add_argument('--ankle-speed-mm-s', type=float, default=20.0, help='ankle command speed limit, mm/s')
    parser.add_argument('--ankle-response-time', type=float, default=0.10, help='ankle smoothing time per stage, seconds; higher is softer')
    parser.add_argument('--ankle-port', default='COM7', help='Seenpin serial CAN-FD adapter')
    parser.add_argument('--no-ankle', action='store_true', help='Run knee only')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--sign', type=int, choices=(-1, 1), default=1, help='motor direction relative to simulator')
    parser.add_argument('--offset-deg', type=float, default=0, help='motor angle at zero simulator flexion')
    for name, default in PTM_DEFAULTS.items():
        parser.add_argument('--' + name.replace('_', '-'), type=float, default=default,
                            help=f'P-T-M position gain, default {default}; protocol range 0..{PTM_MAX[name]}')
    args = parser.parse_args()
    try:
        gains = validate_gains({name: getattr(args, name) for name in PTM_DEFAULTS})
        validate_mapping(args.sign, math.radians(args.offset_deg))
        if not all(math.isfinite(x) and x > 0 for x in (args.ankle_speed_mm_s, args.ankle_response_time)):
            raise ValueError('Ankle speed and smoothing time must be positive and finite')
    except ValueError as exc:
        parser.error(str(exc))
    from motorevo_driver import MotorevoMotor
    trace = CanTrace()
    ankle = None
    if not args.no_ankle:
        from ankle_bridge import AnkleBridge
        from seenpin_driver import SeenpinActuators
        ankle = AnkleBridge(partial(SeenpinActuators, port=args.ankle_port, log=trace.message), log=trace.message, max_speed=args.ankle_speed_mm_s, response_time=args.ankle_response_time)
    bridge = KneeBridge(partial(MotorevoMotor, motor_id=1, interface='pcan', channel='PCAN_USBBUS1', bitrate=1000000, theta_min=THETA_MIN, theta_max=THETA_MAX, trace=trace), args.sign, math.radians(args.offset_deg), log=trace.message, gains=gains, ankle=ankle)
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
        print(f'P-T-M control (status 2), position scale 0..1.83 rad, 50 Hz direct simulator tracking; gains (12-bit): {gains}', flush=True)
        print('Ankle disabled (--no-ankle)' if args.no_ankle else f'Ankle: {args.ankle_port}, CAN-FD+BRS 1M/5M, nodes 1/2/3; soft motion {args.ankle_speed_mm_s:g} mm/s, filter {args.ankle_response_time:g} s; opens on Arm knee + ankle', flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if server:
            server.server_close()
        bridge.close()


if __name__ == '__main__':
    main()
