import math
import time
import unittest
import json
import threading
from unittest.mock import patch
from http.server import ThreadingHTTPServer
from urllib.request import Request, urlopen
from urllib.error import HTTPError
from knee_bridge import KneeBridge, STALE_SECONDS, make_handler, validate_gains, PTM_DEFAULTS, BridgeServer, DT, MAX_VELOCITY, THETA_MAX, validate_mapping
from motorevo_driver import MotorevoMotor
import can


class FakeMotor:
    def __init__(self):
        self.calls = []
        self.commands = []
        self.fault = 0
        self.parameters = {11: 2, 23: 0.0, 24: 0.0, 25: 1.83, 37: 1000, 47: 1, 56: 0xFFFFFFFF}
        self.parameters.update({26:-10.,27:10.,28:0.,29:250.,30:0.,31:50.,34:-50.,35:50.})
        self.vel_min,self.vel_max=-10.,10.
        self.kp_min,self.kp_max=0.,250.
        self.kd_min,self.kd_max=0.,50.
        self.torque_min,self.torque_max=-50.,50.

    def read_parameter(self, index, data_type="float"):
        return self.parameters[index]

    def feedback(self, status):
        return {'status': status, 'fault': self.fault, 'fault_desc': 'fake fault',
                'position_rad': 0.2, 'velocity_rad_s': 0, 'torque_nm': 0, 'temp_c': 20}

    def enter_rest_mode(self):
        self.calls.append('rest')
        return self.feedback(0)

    def enter_motor_mode(self):
        self.calls.append('arm')
        return self.feedback(2)

    def send_ptm_command(self, pos, **kwargs):
        self.commands.append((time.monotonic(), pos, kwargs))
        return self.feedback(2)

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
                validate_gains({**PTM_DEFAULTS, 'kp_pos': value})
        self.assertEqual(validate_gains(PTM_DEFAULTS), PTM_DEFAULTS)

    def test_selected_gains_reach_motor_command(self):
        self.bridge.gains = validate_gains({**PTM_DEFAULTS, 'kp_pos': 30})
        self.bridge.update(20, True)
        self.bridge.arm()
        time.sleep(0.05)
        self.assertEqual(self.motor.commands[-1][2]['kp'], 30)
        self.assertTrue(self.bridge.samples)
        self.assertIn('command_rad', self.bridge.samples[-1])

    def setUp(self):
        self.motor = FakeMotor()
        self.bridge = KneeBridge(lambda: self.motor)

    def tearDown(self):
        self.bridge.close()

    def test_sequence_and_direct_target(self):
        self.bridge.update(90, True)
        self.bridge.arm()
        time.sleep(0.09)
        self.assertEqual(self.motor.calls[:2], ['rest', 'arm'])
        self.assertTrue(self.motor.commands)
        self.assertGreaterEqual(self.motor.commands[0][1], 0.2)
        self.assertAlmostEqual(self.motor.commands[-1][1], math.radians(90))
        self.assertEqual(self.motor.commands[-1][2]['kp'], 20)
        self.assertEqual(self.motor.commands[-1][2]['kd'], 4)
        self.assertEqual(self.motor.commands[-1][2]['vel_rad_s'], 0)
        self.assertEqual(self.motor.commands[-1][2]['torque_ff_nm'], 0)
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
        for value in (-1, 105.01, 110, math.nan, math.inf, True):
            with self.assertRaises(ValueError):
                self.bridge.update(value, True)
        self.bridge.update(20, True)
        self.motor.fault = 2
        with self.assertRaises(RuntimeError):
            self.bridge.arm()
        self.assertEqual(self.motor.calls, ['rest', 'close'])
        self.assertFalse(self.bridge.status()['armed'])

    def test_full_range_mapping(self):
        for angle in (0, 30, 70, 90, 105):
            self.bridge.update(angle, True)
            self.assertAlmostEqual(self.bridge.goal, min(math.radians(angle), THETA_MAX))
        validate_mapping(-1, THETA_MAX)
        reverse = KneeBridge(lambda: self.motor, sign=-1, offset=THETA_MAX)
        try:
            reverse.update(105, True)
            self.assertAlmostEqual(reverse.goal, 0)
            reverse.update(0, True)
            self.assertAlmostEqual(reverse.goal, THETA_MAX)
        finally:
            reverse.close()
        for sign, offset in ((-1, 0), (1, 0.1), (1, math.nan)):
            with self.assertRaises(ValueError):
                KneeBridge(lambda: self.motor, sign=sign, offset=offset)

    def test_servo_mode_is_rejected_before_sending_ptm(self):
        self.bridge.update(30, True)
        with patch.object(self.motor, 'enter_motor_mode', return_value=self.motor.feedback(1)):
            with self.assertRaisesRegex(RuntimeError, 'P-T-M'):
                self.bridge.arm()
        self.assertFalse(self.bridge.armed)
        self.assertFalse(self.motor.commands)
        self.assertEqual(self.motor.calls[-1], 'close')

    def test_feedback_loss_disarms(self):
        self.bridge.update(30, True)
        self.bridge.arm()
        with patch.object(self.motor, 'send_ptm_command', return_value=None):
            time.sleep(DT * 3)
        self.assertFalse(self.bridge.armed)
        self.assertIn('timed out', self.bridge.error)

    def test_stop_holds_current_setpoint(self):
        self.bridge.update(90, True)
        self.bridge.arm()
        time.sleep(0.05)
        self.bridge.update(90, False)
        held = self.bridge.status()['setpoint_rad']
        time.sleep(0.05)
        self.assertAlmostEqual(self.bridge.status()['setpoint_rad'], held)

    def test_targets_follow_simulator_without_filter_or_speed_cap(self):
        self.bridge.stop_event.set()
        self.bridge.thread.join()
        self.bridge.setpoint = 0.2
        for mode in ('manual', 'auto'):
            for angle in (0, 105, 30, 90, 0):
                self.bridge.update(angle, True, mode)
                self.bridge._advance_setpoint(time.monotonic(), DT)
                self.assertAlmostEqual(self.bridge.setpoint, min(math.radians(angle), THETA_MAX))

    def test_velocity_mapping_stop_and_stale_reference(self):
        self.bridge.update(30, True, 'auto', 180)
        now = self.bridge.last_update
        self.assertAlmostEqual(self.bridge._command_velocity(now), math.pi)
        self.assertEqual(self.bridge._command_velocity(now + DT * 3), 0)
        self.bridge.update(30, False, 'auto', 180)
        self.assertEqual(self.bridge._command_velocity(self.bridge.last_update), 0)
        self.bridge.update(30, True, 'manual', 180)
        self.assertEqual(self.bridge.velocity, 0)
        for velocity in (-2000, 2000):
            self.bridge.update(30, True, 'auto', velocity)
            self.assertEqual(abs(self.bridge.velocity), MAX_VELOCITY)
        for angle, velocity in ((105, 180), (0, -180)):
            self.bridge.update(angle, True, 'auto', velocity)
            self.assertEqual(self.bridge.velocity, 0)
        for velocity in (True, math.nan, math.inf, 'fast'):
            with self.assertRaises(ValueError):
                self.bridge.update(30, True, 'auto', velocity)
        reverse = KneeBridge(lambda: self.motor, sign=-1, offset=THETA_MAX)
        try:
            reverse.update(30, True, 'auto', 180)
            self.assertAlmostEqual(reverse.velocity, -math.pi)
        finally:
            reverse.close()

    def test_auto_velocity_reaches_motor(self):
        self.bridge.update(30, True, 'auto', 180)
        self.bridge.arm()
        for _ in range(8):
            self.bridge.update(30, True, 'auto', 180)
            time.sleep(0.01)
        moving = [c for c in self.motor.commands if c[2]['vel_rad_s'] != 0]
        self.assertTrue(moving)
        self.assertAlmostEqual(moving[-1][2]['vel_rad_s'], math.pi)
        self.assertAlmostEqual(moving[-1][1], math.radians(30))

    def test_gait_command_timing_at_simulator_speeds(self):
        self.bridge.stop_event.set()
        self.bridge.thread.join()
        # Same knee profile and phase rates as motion.js and app.js.
        for speed in (0.1, 0.3, 0.8):
            previous = 3.0
            for i in range(1, 151):
                phase = (i * DT * speed / 0.35) % 1
                if phase < 0.62:
                    angle = 3 + 10 * math.sin(math.pi * phase / 0.62) ** 2
                else:
                    angle = 3 + 62 * math.sin(math.pi * (phase - 0.62) / 0.38) ** 1.4
                self.bridge.update(angle, True, 'auto', (angle - previous) / DT)
                self.bridge._advance_setpoint(self.bridge.last_update, DT)
                self.assertAlmostEqual(self.bridge.setpoint, math.radians(angle))
                self.assertLessEqual(abs(self.bridge.velocity), MAX_VELOCITY)
                previous = angle

    def test_zero_checks_following_rest_reply(self):
        old = self.motor.feedback(0)
        zeroed = {**old, 'position_rad': -0.00019}
        def accept_zero():
            self.motor.parameters[23] = 0.2
            return old
        with patch.object(self.motor, 'enter_rest_mode', side_effect=[old, zeroed, zeroed, zeroed]), patch.object(self.motor, 'set_zero_position', side_effect=accept_zero, create=True) as zero:
            self.bridge.zero()
            zero.assert_called_once()
        self.assertEqual(self.bridge.status()['feedback'], zeroed)
        self.assertFalse(self.bridge.status()['armed'])
        self.assertEqual(self.motor.calls[-1], 'close')

    def test_disabled_protections_and_wrong_scale_block_arming(self):
        self.bridge.update(20, True)
        for index, value in ((56, 0xFFFFFF7C), (56, 0xFFFFFF7F), (24, -12.5), (25, 12.5), (23, math.nan)):
            defaults = dict(self.motor.parameters)
            self.motor.parameters[index] = value
            with self.assertRaises(RuntimeError):
                self.bridge.arm()
            self.assertFalse(self.bridge.armed)
            self.assertNotIn('arm', self.motor.calls)
            self.assertFalse(self.motor.commands)
            self.motor.parameters = defaults

    def test_zero_without_offset_change_is_rejected_and_blocks_rearm(self):
        old = self.motor.feedback(0)
        zeroed = {**old, 'position_rad': 0.0}
        with patch.object(self.motor, 'enter_rest_mode', side_effect=[old, zeroed, zeroed, zeroed]), patch.object(self.motor, 'set_zero_position', return_value=old, create=True):
            with self.assertRaisesRegex(RuntimeError, 'Offset did not change'):
                self.bridge.zero()
        self.assertFalse(self.bridge.zero_verification['command_confirmed'])
        self.bridge.update(0, True)
        with self.assertRaisesRegex(RuntimeError, 'Previous knee zero verification failed'):
            self.bridge.arm()

    def test_zero_missing_acknowledgement_is_rejected(self):
        with patch.object(self.motor, 'set_zero_position', return_value=None, create=True):
            with self.assertRaisesRegex(RuntimeError, 'not acknowledged'):
                self.bridge.zero()
        self.assertFalse(self.bridge.armed)

    def test_position_fault_blocks_zero_and_preserves_cause_on_rearm(self):
        feedback = {**self.motor.feedback(0), 'position_rad': 0.0,
                    'fault': 13, 'fault_desc': 'Position Out of Range'}
        with patch.object(self.motor, 'enter_rest_mode', return_value=feedback), patch.object(self.motor, 'set_zero_position', create=True) as zero:
            with self.assertRaisesRegex(RuntimeError, 'fault 13 .*Position Out of Range'):
                self.bridge.zero()
            zero.assert_not_called()
        self.assertEqual(self.bridge.feedback, feedback)
        self.bridge.update(0, True)
        with self.assertRaisesRegex(RuntimeError, 'Previous knee zero verification failed:.*fault 13'):
            self.bridge.arm()
        self.assertNotIn('arm', self.motor.calls)
        self.assertFalse(self.motor.commands)

    def test_successful_zero_retry_clears_failed_verification(self):
        self.bridge.zero_verification = dict(command_confirmed=False, error='old failure')
        feedback = {**self.motor.feedback(0), 'position_rad': 0.0}
        with patch.object(self.motor, 'enter_rest_mode', return_value=feedback), patch.object(self.motor, 'set_zero_position', return_value=feedback, create=True):
            self.bridge.zero()
        self.assertTrue(self.bridge.zero_verification['command_confirmed'])
        self.bridge.update(0, False)
        self.bridge.arm()
        self.assertTrue(self.bridge.armed)

    def test_machine_offset_changed_externally_blocks_arming(self):
        self.bridge.zero_verification = dict(command_confirmed=True, machine_offset_after_rad=0.0)
        self.motor.parameters[23] = 1.0
        self.bridge.update(0, True)
        with self.assertRaisesRegex(RuntimeError, 'zero changed'):
            self.bridge.arm()
        self.assertNotIn('arm', self.motor.calls)

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
            body = json.dumps({'angle_deg': 30, 'running': True, 'mode': 'auto', 'velocity_deg_s': 180}).encode()
            headers = {'Content-Type': 'application/json', 'Origin': base, 'X-Bridge-Token': 'secret'}
            with urlopen(Request(base + '/api/update', data=body, headers=headers)) as response:
                data = json.load(response)
                self.assertAlmostEqual(data['goal_rad'], math.radians(30))
                self.assertAlmostEqual(data['command_velocity_rad_s'], math.pi)
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
    def test_ptm_frames_match_working_gui_and_feedback_scale(self):
        # Fixed vectors from the GUI's 16/12/12/12/12-bit encoding, Kp=20, Kd=4.
        for angle, expected_position in ((0, 0), (30, 18750), (70, 43751), (105, 65535)):
            with self.subTest(angle=angle), patch('motorevo_driver.can.Bus') as factory:
                motor = MotorevoMotor(theta_min=0, theta_max=THETA_MAX)
                reply = [2, expected_position >> 8, expected_position & 255, 0x7f, 0xf7, 0xff, 0, 27]
                factory.return_value.recv.return_value = can.Message(arbitration_id=1, data=reply, is_extended_id=False)
                feedback = motor.send_ptm_command(math.radians(angle), kp=20, kd=4)
                sent = factory.return_value.send.call_args.args[0]
                self.assertEqual(sent.arbitration_id, 1)
                self.assertFalse(sent.is_extended_id)
                self.assertEqual(bytes(sent.data), bytes([
                    expected_position >> 8, expected_position & 255,
                    0x7f, 0xf1, 0x47, 0x14, 0x77, 0xff]))
                self.assertAlmostEqual(feedback['position_rad'], min(math.radians(angle), THETA_MAX), delta=THETA_MAX / 65535)
                self.assertEqual(feedback['status'], 2)
                self.assertEqual(feedback['fault'], 0)
                self.assertEqual(feedback['temp_c'], 27)

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
