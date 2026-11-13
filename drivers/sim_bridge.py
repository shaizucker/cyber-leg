"""Hardware-free bridge for UI development: the real bridge code driving simulated motors.

Runs the same HTTP server, KneeBridge/HipBridge control loops, safety checks and
online console as knee_bridge.py, but the knee (CAN ID 1) and hip (ID 2) are
simulated rigid joints under P-T-M control (inertia, viscous friction and a
gravity-like load). Use it to try the simulator, arming, joint tracking and the
energy panel without a Jetson or motors. The ankle output is disabled.

    python drivers/sim_bridge.py            # then open http://127.0.0.1:8000/

Feedback is a model, not a measurement: never use it to tune real hardware.
"""
import argparse
import math
import secrets
import threading
import time

from knee_bridge import (BridgeServer, HipBridge, KneeBridge, make_handler, HIP_THETA_MIN, HIP_THETA_MAX,
                         KNEE_ANGLE_SCALE)
from motor_console import MotorConsole

TORQUE_LIMIT = 15.0


class SimMotor:
    """P-T-M joint model: J*a = clamp(kp*(p*-p) + kd*(v*-v) + tff) - b*v - load(p)."""

    def __init__(self, theta_min, theta_max, inertia, damping, load):
        self.parameters = {11: 2, 23: 0.0, 24: theta_min, 25: theta_max, 37: 1000, 47: 1, 56: 0xFFFFFFFF,
                           21: TORQUE_LIMIT, 46: 1.0,
                           26: -10.0, 27: 10.0, 28: 0.0, 29: 250.0, 30: 0.0, 31: 50.0, 34: -50.0, 35: 50.0}
        self.theta_min, self.theta_max = theta_min, theta_max
        self.vel_min, self.vel_max, self.kp_min, self.kp_max = -10.0, 10.0, 0.0, 250.0
        self.kd_min, self.kd_max, self.torque_min, self.torque_max = 0.0, 50.0, -50.0, 50.0
        self.inertia, self.damping, self.load = inertia, damping, load
        self.position, self.velocity, self.torque, self.mode = 0.0, 0.0, 0.0, 0
        self.last = time.monotonic()
        self.lock = threading.Lock()

    def read_parameter(self, index, data_type='float', timeout=0.25):
        return self.parameters[index]

    def write_parameter(self, index, value, data_type='float', timeout=0.25):
        self.parameters[index] = value
        return value

    def _feedback(self):
        return {'status': self.mode, 'fault': 0, 'fault_desc': 'No fault', 'position_rad': self.position,
                'velocity_rad_s': self.velocity, 'torque_nm': self.torque, 'temp_c': 30}

    def enter_rest_mode(self):
        with self.lock:
            # Rest: torque off, the mechanism is assumed supported (no fall).
            self.mode, self.velocity, self.torque, self.last = 0, 0.0, 0.0, time.monotonic()
            return self._feedback()

    def enter_motor_mode(self):
        with self.lock:
            self.mode, self.last = 2, time.monotonic()
            return self._feedback()

    def set_zero_position(self):
        with self.lock:
            self.parameters[23] += self.position
            self.position = 0.0
            return self._feedback()

    def send_ptm_command(self, pos_rad, vel_rad_s=0.0, kp=10.0, kd=1.0, torque_ff_nm=0.0, timeout=0.02):
        with self.lock:
            now = time.monotonic()
            elapsed, self.last = min(now - self.last, 0.1), now
            steps = max(1, math.ceil(elapsed / 0.001))
            h = elapsed / steps
            for _ in range(steps):
                drive = kp * (pos_rad - self.position) + kd * (vel_rad_s - self.velocity) + torque_ff_nm
                self.torque = max(-TORQUE_LIMIT, min(TORQUE_LIMIT, drive))
                accel = (self.torque - self.damping * self.velocity - self.load(self.position)) / self.inertia
                self.velocity += accel * h
                self.position += self.velocity * h
            return self._feedback()

    def close(self):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--bind', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8000)
    args = parser.parse_args()
    # Knee: motor turns half the joint angle; light shank load. Hip: whole-leg load.
    knee_motor = SimMotor(-12.0, 12.0, inertia=0.05, damping=0.3, load=lambda p: 1.5 * math.sin(2 * p))
    hip_motor = SimMotor(HIP_THETA_MIN, HIP_THETA_MAX, inertia=0.25, damping=0.5, load=lambda p: 4.0 * math.sin(p))
    log = lambda message: print(message, flush=True)
    hip = HipBridge(lambda: hip_motor, offset=HIP_THETA_MIN, log=log)
    bridge = KneeBridge(lambda: knee_motor, log=log, hip=hip, angle_scale=KNEE_ANGLE_SCALE)
    MotorConsole(bridge, None)
    server = BridgeServer((args.bind, args.port), make_handler(bridge, secrets.token_urlsafe(32), args.port, args.bind))
    print(f'SIMULATED MOTORS — open http://{args.bind}:{args.port}/  (Ctrl+C to stop). No hardware is contacted.', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        bridge.close()


if __name__ == '__main__':
    main()
