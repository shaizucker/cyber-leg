"""Local simulator-to-hip-knee-and-ankle bridge. Run with --help for options."""
import argparse
import json
import math
import queue
import secrets
import socket
import sys
import threading
import time
from collections import deque
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit
from knee_safety import read_configuration, validate_configuration, verify_command_scaling, POSITION_GUARD_RAD

DT = 0.02
STALE_SECONDS = 0.5
MAX_VELOCITY = 10.0  # Configured P-T-M velocity encoding range, rad/s.
SIM_MAX_DEG = 105.0
# Match gemini-code-gui.py and the current MParameter Studio configuration.
THETA_MIN, THETA_MAX = 0.0, 1.83
SIM_MAX = THETA_MAX  # 105 degrees saturates at 1.83 rad, as in the working GUI.
KNEE_ANGLE_SCALE = 0.5  # Motor rotation per simulator knee angle.
PTM_DEFAULTS = dict(kp_pos=20.0, kd_pos=4.0)
PTM_MAX = dict(kp_pos=250.0, kd_pos=50.0)
HIP_MIN_DEG, HIP_MAX_DEG = -30.0, 30.0
HIP_THETA_MIN, HIP_THETA_MAX = math.radians(HIP_MIN_DEG), math.radians(HIP_MAX_DEG)
# Position gains provide tracking stiffness; [21] in the motor is the torque cap.
HIP_PTM_DEFAULTS = dict(kp_pos=20.0, kd_pos=4.0)


def validate_mapping(sign, offset, theta_min=THETA_MIN, theta_max=THETA_MAX, sim_max=SIM_MAX):
    if sign not in (-1, 1) or not math.isfinite(offset):
        raise ValueError('Invalid motor direction or offset')
    if any(not theta_min <= p <= theta_max for p in (offset, offset + sign * sim_max)):
        raise ValueError(f'Mapped joint range must fit within the configured motor range {theta_min:g}..{theta_max:g} rad')


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


def select_outputs(only='all', no_knee=False, no_hip=False, no_ankle=False):
    """Resolve startup selection without constructing any hardware drivers."""
    if only not in ('all', 'knee', 'hip', 'ankle'):
        raise ValueError('Unknown physical output selection')
    outputs = {'knee', 'hip', 'ankle'} if only == 'all' else {only}
    outputs -= {name for name, disabled in
                (('knee', no_knee), ('hip', no_hip), ('ankle', no_ankle)) if disabled}
    if not outputs:
        raise ValueError('Select at least one physical output')
    return outputs


class KneeBridge:
    def __init__(self, motor_factory, sign=1, offset=0, log=None, gains=None, ankle=None, hip=None, start_worker=True, theta_min=THETA_MIN, theta_max=THETA_MAX, sim_max_deg=SIM_MAX_DEG, joint_min_deg=0.0, angle_scale=1.0):
        self.theta_min, self.theta_max = theta_min, theta_max
        self.can_theta_min, self.can_theta_max = theta_min, theta_max
        self.sim_max_deg = sim_max_deg
        self.sim_max = min(math.radians(sim_max_deg), theta_max - theta_min)
        if not math.isfinite(angle_scale) or angle_scale <= 0:
            raise ValueError('Angle scale must be finite and positive')
        self.angle_scale = angle_scale
        validate_mapping(sign, offset, theta_min, theta_max, self.sim_max*self.angle_scale)
        self.gains = validate_gains(PTM_DEFAULTS if gains is None else gains)
        self.motor_factory, self.sign, self.offset = motor_factory, sign, offset
        self.knee_enabled = motor_factory is not None
        self.log = log or (lambda message: None)
        self.lock = threading.RLock()
        self.ankle = ankle
        self.hip = hip
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
        self.reference_rad = 0.0
        self.joint_min_deg = joint_min_deg
        self.soft_limits_deg = [self.joint_min_deg, self.joint_min_deg+sim_max_deg]
        self.ki_pos = 0.0
        self.integral_limit_nm = 0.5
        self.integral_nm = 0.0
        self.torque_limit_nm = None
        self.expected_torque_limit_value = None
        self.configuration_error = None
        self.reference_binding = None
        self.feedback_at = 0.0
        self.console = None
        self.verify_online_scaling = False
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        if start_worker:
            self.thread.start()

    def update(self, angle_deg, running, mode='manual', velocity_deg_s=0.0, extensions_mm=None, hip_angle_deg=None, hip_velocity_deg_s=0.0):
        if self.knee_enabled and (isinstance(angle_deg, bool) or not isinstance(angle_deg, (int, float)) or not math.isfinite(angle_deg) ):
            raise ValueError('Joint angle must be finite')
        if not isinstance(running, bool):
            raise ValueError("Running state must be a boolean")
        if mode not in ('manual', 'auto'):
            raise ValueError('Unknown simulator mode')
        if self.knee_enabled and (isinstance(velocity_deg_s, bool) or not isinstance(velocity_deg_s, (int, float)) or not math.isfinite(velocity_deg_s)):
            raise ValueError('Knee velocity must be finite degrees per second')
        if self.hip:
            self.hip.validate_target(hip_angle_deg, hip_velocity_deg_s)
            self.hip.mapped_target(hip_angle_deg-self.hip.MIN_DEG)
        if self.knee_enabled:
            mapped = self.mapped_target(angle_deg)
        with self.lock:
            if self.ankle:
                self.ankle.update(extensions_mm, running)
            if self.hip:
                self.hip.update(hip_angle_deg, running, mode, hip_velocity_deg_s)
            self.mode = mode
            self.running = running
            self.last_update = time.monotonic()
            if self.knee_enabled:
                self.goal = (mapped
                             if running or not self.armed else self.setpoint)
                # Browser targets already contain the simulator's motion profile.
                # Sending them directly avoids adding a second, slower trajectory.
                velocity = self.sign * self.angle_scale * math.radians(velocity_deg_s) if running and mode == 'auto' else 0.0
                if ((angle_deg >= math.degrees(self.sim_max) and velocity_deg_s > 0) or
                        (angle_deg == 0 and velocity_deg_s < 0)):
                    velocity = 0.0
                self.velocity = max(-MAX_VELOCITY, min(MAX_VELOCITY, velocity))

    def mapped_target(self, normalized_angle):
        angle = normalized_angle+self.joint_min_deg
        if not self.soft_limits_deg[0] <= angle <= self.soft_limits_deg[1]:
            raise ValueError('Target outside configured limits relative to zero')
        mapped = self.sign*self.angle_scale*min(math.radians(normalized_angle),self.sim_max)+self.offset+self.reference_rad
        if not self.theta_min-1e-9 <= mapped <= self.theta_max+1e-9:
            raise ValueError('Target exceeds hardware angle range from the selected zero')
        return max(self.theta_min,min(self.theta_max,mapped))

    def open_motor(self):
        motor = self.motor_factory()
        motor.theta_min, motor.theta_max = self.can_theta_min, self.can_theta_max
        return motor

    def read_motor_configuration(self, motor):
        config = read_configuration(motor)
        validate_configuration(config, self.theta_min, self.theta_max,
                               guard_band=POSITION_GUARD_RAD, allow_wide_range=True)
        # Theta parameters define wire conversion AND drive protection. Keep
        # targets in the original operating envelope, but decode/encode using
        # the exact reported scale, including any permitted protection margin.
        self.can_theta_min = motor.theta_min = config['theta_min_rad']
        self.can_theta_max = motor.theta_max = config['theta_max_rad']
        return config

    def feedback_description(self, feedback):
        if not feedback:
            return 'no motor reply'
        detail = f"status {feedback['status']}, fault {feedback['fault']} ({feedback['fault_desc']})"
        if feedback['fault'] == 13:
            detail += (f'; position is outside the motor range {self.can_theta_min:g}..{self.can_theta_max:g} rad'
                       '; reported angle may be clamped at the boundary; resolve the physical position/reference before retrying')
        return detail

    def _advance_setpoint(self, now, elapsed):
        if self.ankle and not self.ankle.status()['armed']:
            return  # Hold measured knee position until the ankle is ready too.
        self.setpoint = self.goal

    def _command_velocity(self, now):
        # A delayed browser update must not keep requesting motion indefinitely.
        if self.ankle and not self.ankle.status()['armed']:
            return 0.0
        return self.velocity if self.running and now - self.last_update <= DT * 2 else 0.0

    def _verify_motor_configuration(self, check_reference=True):
        self.configuration = self.read_motor_configuration(self.motor)
        if self.verify_online_scaling:
            verify_command_scaling(self.motor)
        if check_reference and self.reference_binding is not None and abs(self.configuration['machine_offset_rad']-self.reference_binding)>1e-6:
            raise RuntimeError('Motor machine zero changed; capture a new signed reference before arming')
        if self.ki_pos or self.expected_torque_limit_value is not None:
            limit = self.motor.read_parameter(21)
            if not math.isfinite(limit) or limit <= 0:
                raise RuntimeError('Motor torque limit is invalid for integral control')
            if self.expected_torque_limit_value is not None and not math.isclose(limit,self.expected_torque_limit_value,rel_tol=1e-6,abs_tol=1e-6):
                raise RuntimeError('Motor torque limit differs from saved settings; read/apply configuration before arming')
            if self.ki_pos:
                constant = self.motor.read_parameter(46)
                if not math.isfinite(constant) or constant <= 0:
                    raise RuntimeError('Torque constant is invalid for integral control')
                self.torque_limit_nm = min(limit,limit*constant)
            else:
                self.torque_limit_nm = limit
        return self.configuration

    def arm(self):
        with self.lock:
            if self.configuration_error:
                raise ValueError(self.configuration_error)
            if self.armed:
                return
            if not self.last_update or (self.knee_enabled and self.goal is None) or time.monotonic() - self.last_update > STALE_SECONDS:
                raise ValueError("Simulator updates are not current")
            if self.knee_enabled and not self.theta_min <= self.goal <= self.theta_max:
                raise ValueError("Mapped target is outside the motor range")
            if self.ankle:
                self.ankle.check_ready()
            try:
                if not self.knee_enabled:
                    if not self.hip and not self.ankle:
                        raise ValueError('Select at least one physical output')
                    if self.hip:
                        self.hip.arm()
                    if time.monotonic() - self.last_update > STALE_SECONDS:
                        raise RuntimeError('Simulator updates became stale during configuration checks')
                    if self.ankle:
                        self.ankle.arm()
                    self.armed = True
                    self.error = None
                    return
                self.motor = self.open_motor()
                config = self._verify_motor_configuration()
                if self.zero_verification is not None:
                    if not self.zero_verification.get('command_confirmed'):
                        cause = self.zero_verification.get('error', 'no confirmed zero readback')
                        raise RuntimeError(f'Previous knee zero verification failed: {cause}. '
                                           'Stop and disarm, resolve the cause, then repeat machine-zero verification before arming')
                    if abs(config['machine_offset_rad'] - self.zero_verification['machine_offset_after_rad']) > 1e-6:
                        raise RuntimeError('Knee machine zero changed since verification; verify the reference again')
                t = self.motor.enter_rest_mode()
                self.feedback = t
                if not t or t['fault'] != 0 or t['status'] != 0:
                    raise RuntimeError(f'Rest state failed or a fault is present: {self.feedback_description(t)}')
                lo, hi = self.theta_min, self.theta_max
                if not lo - math.radians(1) <= t['position_rad'] <= hi + math.radians(1):
                    raise RuntimeError(f"Measured position {math.degrees(t['position_rad']):.2f} degrees is outside {math.degrees(lo):.1f} to {math.degrees(hi):.1f} degrees; disarm and zero at the reference pose")
                self.feedback = t
                if not self.soft_limits_deg[0]-0.1 <= self.relative_degrees(t['position_rad']) <= self.soft_limits_deg[1]+0.1:
                    raise RuntimeError('Measured position is outside configured signed command limits')
                # A boundary measurement can be a fraction of an encoding step
                # outside the operating interval when its CAN scale has a margin.
                self.setpoint = max(lo, min(hi, t['position_rad']))
                if not self.running:
                    self.goal = self.setpoint
                time.sleep(0.05)  # Match the working GUI's Rest-to-Motor delay.
                if time.monotonic() - self.last_update > STALE_SECONDS:
                    raise RuntimeError('Simulator updates became stale during knee configuration checks')
                if self.hip:
                    try:
                        self.hip.arm()
                    except Exception as exc:
                        raise RuntimeError(f'Hip: {exc}') from exc
                if time.monotonic() - self.last_update > STALE_SECONDS:
                    raise RuntimeError('Simulator updates became stale during rotary configuration checks')
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
        self.integral_nm = 0.0
        if self.hip:
            self.hip.disarm()
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
        if not self.knee_enabled:
            raise ValueError('Knee output is disabled')
        with self.lock:
            if self.armed:
                raise ValueError('Disarm before zeroing')
            if self.running:
                raise ValueError('Stop the simulator before zeroing')
            if self.offset != self.theta_min or self.sign != 1:
                raise ValueError('Hardware zero requires the default positive direction and reference offset')
            self.zero_verification = dict(command_confirmed=False, physical_reference_verified=False,
                                          power_cycle_persistence_verified=False)
            try:
                self.motor = self.open_motor()
                before = self._verify_motor_configuration()
                t = self.motor.enter_rest_mode()
                self.feedback = t
                if (not t or t['status'] != 0 or t['fault'] != 0 or
                        not math.isfinite(t['position_rad']) or not math.isfinite(t['velocity_rad_s']) or
                        abs(t['velocity_rad_s']) > 0.05):
                    raise RuntimeError('Cannot zero: motor must confirm fault-free stationary Rest; '
                                       + self.feedback_description(t))
                previous_position = t['position_rad']
                encoding_step = (before['theta_max_rad']-before['theta_min_rad'])/65535
                previous_position_clipped = any(abs(previous_position-edge) <= encoding_step
                    for edge in (before['theta_min_rad'],before['theta_max_rad']))
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
                after = self._verify_motor_configuration(check_reference=False)
                if (abs(previous_position) > math.radians(0.1) and
                        not previous_position_clipped and
                        abs(after['machine_offset_rad'] - before['machine_offset_rad']) <= 1e-6):
                    raise RuntimeError('Zero feedback is near zero but Machine Angle Offset did not change')
                self.setpoint = self.goal = t['position_rad']
                self.reference_rad = 0.0
                self.reference_binding = None
                low = max(self.joint_min_deg,self.soft_limits_deg[0])
                high = min(self.joint_min_deg+self.sim_max_deg,self.soft_limits_deg[1])
                if not low < high:
                    raise RuntimeError('Hardware zero leaves no valid command interval; configure limits before arming')
                self.soft_limits_deg = [low,high]
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
            return {"knee_enabled": self.knee_enabled,
                    "outputs": [name for name, enabled in (("knee", self.knee_enabled), ("hip", self.hip is not None), ("ankle", self.ankle is not None)) if enabled],
                    "armed": self.armed, "error": self.error or self.configuration_error, "feedback": self.feedback,
                    "motor_configuration": self.configuration, "zero_verification": self.zero_verification,
                    "setpoint_rad": self.setpoint, "goal_rad": self.goal,
                    "command_velocity_rad_s": self._command_velocity(time.monotonic()),
                    "control_mode": "P-T-M", "max_angle_deg": self.sim_max_deg,
                    "hip": self.hip.status() if self.hip else {"enabled": False},
                    "ankle": self.ankle.status() if self.ankle else {"enabled": False},
                    "sign": self.sign, "offset_rad": self.offset, "gains": self.gains.copy(),
                    "angle_scale": self.angle_scale,
                    "reference_rad": self.reference_rad, "soft_limits_deg": list(self.soft_limits_deg),
                    "can_theta_min_rad": self.can_theta_min, "can_theta_max_rad": self.can_theta_max,
                    "console": self.console.status() if self.console else None}

    def relative_degrees(self, position):
        return math.degrees((position-self.offset-self.reference_rad)/(self.sign*self.angle_scale))+self.joint_min_deg

    def _integral_command(self, elapsed):
        if not self.running or not self.ki_pos or not self.feedback:
            self.integral_nm = 0.0
            return 0.0
        if not self.torque_limit_nm or not math.isfinite(self.torque_limit_nm):
            raise RuntimeError('Read and verify motor torque limit before enabling position I')
        error = self.setpoint-self.feedback['position_rad']
        cap = min(self.integral_limit_nm, self.torque_limit_nm)
        candidate = max(-cap, min(cap, self.integral_nm+self.ki_pos*error*max(0,min(elapsed,DT*2))))
        pd = self.gains['kp_pos']*error + self.gains['kd_pos']*(self._command_velocity(time.monotonic())-self.feedback['velocity_rad_s'])
        # Conditional integration: do not wind up further into output saturation.
        if not ((pd+candidate > self.torque_limit_nm and error > 0) or
                (pd+candidate < -self.torque_limit_nm and error < 0)):
            self.integral_nm = candidate
        return self.integral_nm

    def _control_step(self, now, elapsed):
        self._advance_setpoint(now, elapsed)
        if not math.isfinite(self.setpoint) or not self.theta_min <= self.setpoint <= self.theta_max:
            raise RuntimeError('Joint command outside the verified position range')
        t = self.motor.send_ptm_command(
            self.setpoint, vel_rad_s=self._command_velocity(now), kp=self.gains['kp_pos'],
            kd=self.gains['kd_pos'], torque_ff_nm=self._integral_command(elapsed), timeout=0.01)
        if not t:
            raise RuntimeError("Motor feedback timed out")
        self.feedback = t
        self.feedback_at = time.monotonic()
        self.samples.append(dict(t=time.monotonic(), goal_rad=self.goal,
                                 command_rad=self.setpoint,
                                 command_velocity_rad_s=self._command_velocity(now),
                                 relative_angle_deg=self.relative_degrees(t['position_rad']),
                                 relative_command_deg=self.relative_degrees(self.setpoint), **t))
        if now - self._last_diagnostic >= 0.25:
            self._last_diagnostic = now
            self.log(f"TRACK t={now:.3f} goal={math.degrees(self.goal):+.2f} deg command={math.degrees(self.setpoint):+.2f} deg actual={math.degrees(t['position_rad']):+.2f} deg velocity={math.degrees(t['velocity_rad_s']):+.2f} deg/s torque={t['torque_nm']:+.3f} Nm fault={t['fault']}")
        if t['status'] != 2 or t['fault'] != 0:
            raise RuntimeError(f"Motor stopped or faulted: {t['fault_desc']}")

    def _hip_control_step(self, now, elapsed):
        try:
            self.hip._control_step(now, elapsed)
        except Exception as exc:
            self.hip.error = str(exc)
            raise RuntimeError(f'Hip: {exc}') from exc

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
                        if self.knee_enabled:
                            self._control_step(now, now - last_t)
                        if self.hip:
                            if not self.hip.armed:
                                raise RuntimeError('Hip is not armed')
                            # Both joints hold measured position during ankle startup.
                            self.hip.running = self.running
                            if self.ankle and not self.ankle.status()['armed']:
                                goal, velocity = self.hip.goal, self.hip.velocity
                                try:
                                    self.hip.goal, self.hip.velocity = self.hip.setpoint, 0.0
                                    self._hip_control_step(now, now - last_t)
                                finally:
                                    self.hip.goal, self.hip.velocity = goal, velocity
                            else:
                                self._hip_control_step(now, now - last_t)
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
        if self.thread.ident is not None:
            self.thread.join()
        try:
            self.disarm()
        finally:
            if self.hip:
                self.hip.close()
            if self.ankle:
                self.ankle.close()


class HipBridge(KneeBridge):
    """Hip has its own signed encoding and physical zero at neutral (0 degrees)."""
    MIN_DEG = HIP_MIN_DEG
    MAX_DEG = HIP_MAX_DEG

    def __init__(self, motor_factory, **kwargs):
        kwargs.setdefault('offset', HIP_THETA_MIN)
        kwargs.setdefault('gains', HIP_PTM_DEFAULTS)
        super().__init__(motor_factory, start_worker=False, theta_min=HIP_THETA_MIN,
                         theta_max=HIP_THETA_MAX, sim_max_deg=self.MAX_DEG-self.MIN_DEG, joint_min_deg=self.MIN_DEG, **kwargs)

    @classmethod
    def validate_target(cls, angle, velocity):
        if (isinstance(angle, bool) or not isinstance(angle, (int, float)) or
                not math.isfinite(angle) or not cls.MIN_DEG <= angle <= cls.MAX_DEG):
            raise ValueError('Hip angle must be between -30 and 30 degrees')
        if (isinstance(velocity, bool) or not isinstance(velocity, (int, float)) or
                not math.isfinite(velocity)):
            raise ValueError('Hip velocity must be finite degrees per second')

    def update(self, angle_deg, running, mode='manual', velocity_deg_s=0.0):
        self.validate_target(angle_deg, velocity_deg_s)
        super().update(angle_deg - self.MIN_DEG, running, mode, velocity_deg_s)

    def status(self):
        return {**super().status(), 'enabled': True, 'motor_id': 2,
                'min_angle_deg': self.MIN_DEG, 'max_angle_deg': self.MAX_DEG,
                'reference_angle_deg': self.MIN_DEG, 'zero_angle_deg': 0.0,
                'theta_min_rad': self.theta_min, 'theta_max_rad': self.theta_max}


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

        def handle_one_request(self):
            try:
                super().handle_one_request()
            except (ConnectionAbortedError, ConnectionResetError, BrokenPipeError):
                # Includes request-header reads and static-file responses. A
                # cancelled browser request is not a motor/control failure.
                self.close_connection = True

        def reply(self, code, data):
            body = json.dumps(data).encode()
            try:
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except (ConnectionAbortedError, ConnectionResetError, BrokenPipeError):
                # Never attempt a second HTTP response on a dead connection.
                self.close_connection = True
                return False
            return True

        def do_GET(self):
            if self.headers.get('Host') != f'127.0.0.1:{port}':
                self.send_error(403)
            elif urlsplit(self.path).path == '/api/session':
                self.reply(200, {'token': token, **bridge.status()})
            elif urlsplit(self.path).path == '/api/configuration':
                self.reply(200, bridge.console.status() if bridge.console else {'available': False})
            elif urlsplit(self.path).path == '/api/telemetry':
                with bridge.lock:
                    samples = list(bridge.samples)
                    hip_samples = list(bridge.hip.samples) if bridge.hip else []
                self.reply(200, {'samples': samples, 'hip_samples': hip_samples})
            else:
                super().do_GET()

        def do_POST(self):
            if (self.headers.get('Host') != f'127.0.0.1:{port}' or
                self.headers.get('Origin') != f'http://127.0.0.1:{port}' or
                self.headers.get('X-Bridge-Token') != token or
                self.headers.get_content_type() != 'application/json'):
                # Drain a small, already-sent body before closing. Otherwise
                # Windows can reset the TCP connection before delivering 403.
                self.close_connection = True
                previous_timeout = self.connection.gettimeout()
                try:
                    size = int(self.headers.get('Content-Length', '0'))
                    if 0 < size <= 1024 and not self.headers.get('Transfer-Encoding'):
                        self.connection.settimeout(0.2)
                        self.rfile.read(size)
                except (ValueError, OSError):
                    pass
                finally:
                    self.connection.settimeout(previous_timeout)
                self.send_error(403)
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 1024:
                    raise ValueError('Invalid request size')
                try:
                    raw = self.rfile.read(size)
                except (ConnectionAbortedError, ConnectionResetError, BrokenPipeError):
                    self.close_connection = True
                    return
                payload = json.loads(raw)
                route = urlsplit(self.path).path
                if route == '/api/update':
                    bridge.update(payload.get('angle_deg'), payload['running'], payload.get('mode', 'manual'), payload.get('velocity_deg_s', 0.0), payload.get('extensions_mm'), payload.get('hip_angle_deg'), payload.get('hip_velocity_deg_s', 0.0))
                elif route in ('/api/configure', '/api/read-hardware', '/api/reference', '/api/power'):
                    if not bridge.console:
                        raise ValueError('Motor configuration console is unavailable')
                    if route == '/api/configure':
                        bridge.console.apply(payload['target'], payload['settings'])
                    elif route == '/api/read-hardware':
                        bridge.console.read(payload['target'])
                    elif route == '/api/reference':
                        bridge.console.zero(payload.get('target', 'all'))
                    else:
                        bridge.console.ingest_power(payload['device'],payload['voltage_v'],payload['bus_current_a'])
                elif route == '/api/arm':
                    bridge.arm()
                elif route == '/api/disarm':
                    bridge.disarm()
                elif route == '/api/zero':
                    if payload.get('joint', 'knee') == 'hip':
                        with bridge.lock:
                            if bridge.armed or bridge.running or not bridge.hip:
                                raise ValueError('Stop and disarm before zeroing an enabled hip')
                            bridge.hip.zero()
                    elif payload.get('joint', 'knee') == 'knee':
                        bridge.zero()
                    else:
                        raise ValueError('Unknown joint')
                    if bridge.console:
                        bridge.console._save()
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
    parser.add_argument('--ankle-port', default='/dev/ttyACM0' if sys.platform.startswith('linux') else 'COM7', help='Seenpin serial CAN-FD adapter; accepts /dev/serial/by-id paths')
    parser.add_argument('--can-interface', choices=('pcan', 'socketcan'), default='socketcan' if sys.platform.startswith('linux') else 'pcan', help='rotary CAN backend (Linux: socketcan; Windows: pcan)')
    parser.add_argument('--can-channel', help='rotary CAN channel (socketcan: can1; pcan: PCAN_USBBUS1)')
    parser.add_argument('--only', choices=('all', 'knee', 'hip', 'ankle'), default='all', help='physical outputs to operate (default: all)')
    parser.add_argument('--no-knee', action='store_true', help='Disable knee output')
    parser.add_argument('--no-ankle', action='store_true', help='Disable ankle outputs')
    parser.add_argument('--no-hip', action='store_true', help='Disable hip output for knee-only hardware')
    parser.add_argument('--hip-sign', type=int, choices=(-1, 1), default=1)
    parser.add_argument('--hip-offset-deg', type=float, default=HIP_MIN_DEG, help='motor position at hip -30 degree reference; default -30 gives neutral zero')
    parser.add_argument('--hip-kp-pos', type=float, default=HIP_PTM_DEFAULTS['kp_pos'], help='hip P-T-M position gain (default 20); motor Torque Limit controls the output cap')
    parser.add_argument('--hip-kd-pos', type=float, default=HIP_PTM_DEFAULTS['kd_pos'], help='hip P-T-M damping gain (default 4)')
    parser.add_argument('--config-file', default=str(Path(__file__).with_name('bridge_settings.json')), help='persistent online settings and zero-reference profile')
    parser.add_argument('--no-config-save', action='store_true', help='keep online settings for this session only')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--sign', type=int, choices=(-1, 1), default=1, help='motor direction relative to simulator')
    parser.add_argument('--offset-deg', type=float, default=0, help='motor angle at zero simulator flexion')
    for name, default in PTM_DEFAULTS.items():
        parser.add_argument('--' + name.replace('_', '-'), type=float, default=default,
                            help=f'P-T-M position gain, default {default}; protocol range 0..{PTM_MAX[name]}')
    args = parser.parse_args()
    if args.can_channel is None:
        args.can_channel = 'can1' if args.can_interface == 'socketcan' else 'PCAN_USBBUS1'
    try:
        outputs = select_outputs(args.only, args.no_knee, args.no_hip, args.no_ankle)
        gains = validate_gains({name: getattr(args, name) for name in PTM_DEFAULTS})
        validate_mapping(args.sign, math.radians(args.offset_deg))
        hip_gains = validate_gains(dict(kp_pos=args.hip_kp_pos, kd_pos=args.hip_kd_pos))
        validate_mapping(args.hip_sign, math.radians(args.hip_offset_deg), HIP_THETA_MIN, HIP_THETA_MAX, HIP_THETA_MAX-HIP_THETA_MIN)
        if not all(math.isfinite(x) and x > 0 for x in (args.ankle_speed_mm_s, args.ankle_response_time)):
            raise ValueError('Ankle speed and smoothing time must be positive and finite')
    except ValueError as exc:
        parser.error(str(exc))
    from motorevo_driver import SharedMotorNetwork
    trace = CanTrace()
    ankle = None
    if 'ankle' in outputs:
        from ankle_bridge import AnkleBridge
        from seenpin_driver import SeenpinActuators
        ankle = AnkleBridge(partial(SeenpinActuators, port=args.ankle_port, log=trace.message), log=trace.message, max_speed=args.ankle_speed_mm_s, response_time=args.ankle_response_time)
    network = SharedMotorNetwork(interface=args.can_interface, channel=args.can_channel, bitrate=1000000,
                                 theta_min=THETA_MIN, theta_max=THETA_MAX, trace=trace)
    hip = None if 'hip' not in outputs else HipBridge(partial(network.motor, 2, theta_min=HIP_THETA_MIN, theta_max=HIP_THETA_MAX), sign=args.hip_sign,
                    offset=math.radians(args.hip_offset_deg), log=trace.message, gains=hip_gains)
    bridge = KneeBridge(partial(network.motor, 1) if 'knee' in outputs else None, args.sign, math.radians(args.offset_deg),
                        log=trace.message, gains=gains, ankle=ankle, hip=hip, angle_scale=KNEE_ANGLE_SCALE)
    from motor_console import MotorConsole
    try:
        MotorConsole(bridge, None if args.no_config_save else args.config_file)
    except Exception:
        bridge.close()
        network.close()
        raise
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
        if 'knee' in outputs:
            print(f'P-T-M control (status 2), knee angle scale {KNEE_ANGLE_SCALE:g} (simulator angle divided by 2); CAN scale read from motor (joint range with up to 1 degree margin, or -12..12 rad); 50 Hz tracking; gains (12-bit): {gains}', flush=True)
        print('Physical outputs: ' + ', '.join(bridge.status()['outputs']), flush=True)
        print(f'Knee: classic CAN node 1 on {args.can_channel} ({args.can_interface})' if 'knee' in outputs else 'Knee disabled (node 1 is not contacted)', flush=True)
        print('Hip disabled' if 'hip' not in outputs else f'Hip: classic CAN node 2 on {args.can_channel} ({args.can_interface}); range -30..30 degrees, neutral = motor zero; P/D {hip_gains}', flush=True)
        print('Ankle disabled' if 'ankle' not in outputs else f'Ankle: {args.ankle_port}, CAN-FD+BRS 1M/5M, nodes 1/2/3; soft motion {args.ankle_speed_mm_s:g} mm/s, filter {args.ankle_response_time:g} s; opens when arming all enabled joints', flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if server:
            server.server_close()
        try:
            bridge.close()
        finally:
            network.close()


if __name__ == '__main__':
    main()
