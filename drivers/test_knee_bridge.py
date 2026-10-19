import math
import time
import unittest
import json
import threading
from unittest.mock import patch
from http.server import ThreadingHTTPServer
from urllib.request import Request, urlopen
from urllib.error import HTTPError
from knee_bridge import KneeBridge, STALE_SECONDS, make_handler, validate_gains, SERVO_DEFAULTS, BridgeServer
from motorevo_driver import MotorevoMotor
import can


class FakeMotor:
    def __init__(self):
        self.calls = []
        self.commands = []
        self.fault = 0

    def feedback(self, status):
        return {'status': status, 'fault': self.fault, 'fault_desc': 'fake fault',
                'position_rad': 0.2, 'velocity_rad_s': 0, 'torque_nm': 0, 'temp_c': 20}

    def enter_rest_mode(self):
        self.calls.append('rest')
        return self.feedback(0)

    def enter_motor_mode(self):
        self.calls.append('arm')
        return self.feedback(1)

    def send_servo_command(self, pos, **kwargs):
        self.commands.append((time.monotonic(), pos, kwargs))
        return self.feedback(1)

    def close(self):
        self.calls.append('close')


class BridgeTests(unittest.TestCase):
    def test_exclusive_bridge_port(self):
        server = BridgeServer(('127.0.0.1', 0), make_handler(self.bridge, 'test', 0))
        try:
            with self.assertRaises(OSError):
                BridgeServer(server.server_address, make_handler(self.bridge, 'other', 0))
        finally:
            server.server_close()

    def test_gain_validation(self):
        for value in (math.nan, math.inf, -1, 251, True):
            with self.assertRaises(ValueError):
                validate_gains({**SERVO_DEFAULTS, 'kp_pos': value})
        self.assertEqual(validate_gains(SERVO_DEFAULTS), SERVO_DEFAULTS)

    def test_selected_gains_reach_motor_command(self):
        self.bridge.gains = validate_gains({**SERVO_DEFAULTS, 'kp_pos': 30})
        self.bridge.update(20, True)
        self.bridge.arm()
        time.sleep(0.05)
        self.assertEqual(self.motor.commands[-1][2]['kp_pos'], 30)
        self.assertTrue(self.bridge.samples)
        self.assertIn('command_rad', self.bridge.samples[-1])

    def setUp(self):
        self.motor = FakeMotor()
        self.bridge = KneeBridge(lambda: self.motor)

    def tearDown(self):
        self.bridge.close()

    def test_sequence_and_slew(self):
        self.bridge.update(90, True)
        self.bridge.arm()
        time.sleep(0.09)
        self.assertEqual(self.motor.calls[:2], ['rest', 'arm'])
        self.assertTrue(self.motor.commands)
        self.assertGreaterEqual(self.motor.commands[0][1], 0.2)
        self.assertLess(self.motor.commands[-1][1], 0.32)
        self.assertEqual(self.motor.commands[-1][2]['kp_pos'], 40)
        self.bridge.disarm()
        self.assertEqual(self.motor.calls[-1], 'close')

    def test_lost_client_and_motor_fault_disarm(self):
        self.bridge.update(20, True)
        self.bridge.arm()
        time.sleep(STALE_SECONDS + 0.08)
        self.assertFalse(self.bridge.status()['armed'])
        self.assertIn('connection lost', self.bridge.status()['error'])
        self.bridge.update(20, True)
        self.bridge.arm()
        self.motor.fault = 2
        time.sleep(0.05)
        self.assertFalse(self.bridge.status()['armed'])
        self.assertIn('fault', self.bridge.status()['error'])

    def test_rejects_invalid_input_and_fault_at_start(self):
        for value in (-1, 111, math.nan, True):
            with self.assertRaises(ValueError):
                self.bridge.update(value, True)
        self.bridge.update(20, True)
        self.motor.fault = 2
        with self.assertRaises(RuntimeError):
            self.bridge.arm()
        self.assertEqual(self.motor.calls, ['rest', 'close'])
        self.assertFalse(self.bridge.status()['armed'])

    def test_stop_holds_current_setpoint(self):
        self.bridge.update(90, True)
        self.bridge.arm()
        time.sleep(0.05)
        self.bridge.update(90, False)
        held = self.bridge.status()['setpoint_rad']
        time.sleep(0.05)
        self.assertAlmostEqual(self.bridge.status()['setpoint_rad'], held)

    def test_manual_profile_is_monotonic_and_finishes_without_overshoot(self):
        self.bridge.stop_event.set()
        self.bridge.thread.join()
        self.bridge.setpoint = 0.2
        self.bridge.goal = 0.7
        self.bridge._plan_manual_move()
        start, end, t0, duration = self.bridge._traj
        positions = []
        for i in range(101):
            self.bridge._advance_setpoint(t0 + duration * i / 100 + 1e-9, duration / 100)
            positions.append(self.bridge.setpoint)
        self.assertTrue(all(a <= b for a, b in zip(positions, positions[1:])))
        self.assertAlmostEqual(positions[-1], end)
        self.assertLess(positions[1] - start, 0.0001)
        self.assertLess(end - positions[-2], 0.0001)
        self.assertLessEqual(max(b - a for a, b in zip(positions, positions[1:])) / (duration / 100), 1.001)

    def test_heartbeat_does_not_restart_manual_move(self):
        self.bridge.update(30, True)
        self.bridge.arm()
        trajectory = self.bridge._traj
        self.bridge.update(30, True)
        self.assertEqual(self.bridge._traj, trajectory)

    def test_zero_checks_following_rest_reply(self):
        old = self.motor.feedback(0)
        zeroed = {**old, 'position_rad': -0.00019}
        with patch.object(self.motor, 'enter_rest_mode', side_effect=[old, zeroed]), patch.object(self.motor, 'set_zero_position', return_value=old, create=True) as zero:
            self.bridge.zero()
            zero.assert_called_once()
        self.assertEqual(self.bridge.status()['feedback'], zeroed)
        self.assertFalse(self.bridge.status()['armed'])
        self.assertEqual(self.motor.calls[-1], 'close')

    def test_zero_refuses_running_simulator(self):
        self.bridge.update(0, True)
        with self.assertRaises(ValueError):
            self.bridge.zero()
        self.assertEqual(self.motor.calls, [])

    def test_http_requires_local_origin_and_token(self):
        server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(self.bridge, 'secret', 0))
        port = server.server_address[1]
        server.RequestHandlerClass = make_handler(self.bridge, 'secret', port)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        base = f'http://127.0.0.1:{port}'
        try:
            with urlopen(base + '/api/session') as response:
                self.assertEqual(json.load(response)['token'], 'secret')
            body = json.dumps({'angle_deg': 30, 'running': True}).encode()
            headers = {'Content-Type': 'application/json', 'Origin': base, 'X-Bridge-Token': 'secret'}
            with urlopen(Request(base + '/api/update', data=body, headers=headers)) as response:
                self.assertAlmostEqual(json.load(response)['goal_rad'], math.radians(30))
            headers['Origin'] = 'https://example.com'
            with self.assertRaises(HTTPError) as error:
                urlopen(Request(base + '/api/arm', data=b'{}', headers=headers))
            self.assertEqual(error.exception.code, 403)
            self.assertEqual(self.motor.calls, [])
        finally:
            server.shutdown()
            server.server_close()
            worker.join()


class DriverTraceTests(unittest.TestCase):
    def test_trace_reports_sent_and_received_raw_frames(self):
        events = []
        response = can.Message(arbitration_id=1, data=[0] * 8, is_extended_id=False)
        with patch('motorevo_driver.can.Bus') as bus_factory:
            bus_factory.return_value.recv.return_value = response
            motor = MotorevoMotor(trace=lambda *event: events.append(event))
            motor.enter_rest_mode()
        self.assertEqual(events[0], ('TX', 1, bytes([255] * 7 + [253]), ''))
        self.assertEqual(events[1], ('RX', 1, bytes(8), ''))

    def test_trace_distinguishes_send_failure_and_timeout(self):
        events = []
        with patch('motorevo_driver.can.Bus') as bus_factory:
            bus_factory.return_value.send.side_effect = RuntimeError('send failed')
            motor = MotorevoMotor(trace=lambda *event: events.append(event))
            with self.assertRaises(RuntimeError):
                motor.enter_rest_mode()
            self.assertEqual(events[0][0], 'TX ERROR')
            bus_factory.return_value.send.side_effect = None
            bus_factory.return_value.recv.return_value = None
            motor._send_and_recv([0] * 8, timeout=0.001)
            self.assertEqual([event[0] for event in events[1:]], ['TX', 'TIMEOUT'])


if __name__ == '__main__':
    unittest.main()
