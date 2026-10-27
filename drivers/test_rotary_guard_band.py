"""Protection clearance must not widen operating targets or corrupt CAN scaling."""
import math
import struct
import unittest
from unittest.mock import patch

import can
from knee_bridge import KneeBridge, HipBridge, THETA_MAX, KNEE_ANGLE_SCALE
from knee_safety import POSITION_GUARD_RAD
from motor_console import MotorConsole
from motorevo_driver import MotorevoMotor
from test_motor_console import ConsoleMotor


def f32(value):
    return struct.unpack('<f', struct.pack('<f', value))[0]


class GuardBandTests(unittest.TestCase):
    def setUp(self):
        self.motor = ConsoleMotor()
        self.motor.parameters[24] = f32(-POSITION_GUARD_RAD)
        self.motor.parameters[25] = f32(THETA_MAX+POSITION_GUARD_RAD)
        self.bridge = KneeBridge(lambda: self.motor, start_worker=False)

    def tearDown(self):
        self.bridge.close()

    def test_actual_scale_is_read_before_feedback_and_targets_stay_restricted(self):
        self.bridge.update(0, True)
        self.bridge.arm()
        self.assertEqual(self.motor.theta_min, self.motor.parameters[24])
        self.assertEqual(self.motor.theta_max, self.motor.parameters[25])
        self.assertEqual(self.bridge.soft_limits_deg, [0, 105])
        self.assertAlmostEqual(self.bridge.mapped_target(105), THETA_MAX)
        for angle in (-.5, 105.5):
            with self.assertRaises(ValueError):
                self.bridge.update(angle, True)
        self.bridge._control_step(self.bridge.last_update, .02)
        self.assertEqual(self.motor.commands[-1][1], 0)

    def test_zero_hold_tolerates_one_step_quantization_and_commands_exact_zero(self):
        lo, hi = self.motor.parameters[24], self.motor.parameters[25]
        code = MotorevoMotor._float_to_uint(0, lo, hi, 16)
        measured = MotorevoMotor._uint_to_float(code, lo, hi, 16)
        self.assertLess(measured, 0)
        self.assertLess(abs(measured), (hi-lo)/65535)
        feedback = {**self.motor.feedback(0), 'position_rad': measured}
        with patch.object(self.motor, 'enter_rest_mode', return_value=feedback):
            self.bridge.update(0, False)
            self.bridge.arm()
        self.assertEqual(self.bridge.setpoint, 0)
        self.bridge._control_step(self.bridge.last_update, .02)
        self.assertEqual(self.motor.commands[-1][1], 0)

    def test_excessive_margin_or_narrowed_motor_range_blocks_arm(self):
        for index, value in ((24,-math.radians(1.1)), (24,.01),
                             (25,THETA_MAX+math.radians(1.1)), (25,THETA_MAX-.01)):
            original = self.motor.parameters[index]
            self.motor.parameters[index] = value
            self.bridge.update(0, False)
            with self.assertRaisesRegex(RuntimeError, 'does not match bridge scale'):
                self.bridge.arm()
            self.assertNotIn('arm', self.motor.calls)
            self.motor.parameters[index] = original

    def test_margin_does_not_allow_arming_outside_signed_operating_limits(self):
        feedback = {**self.motor.feedback(0), 'position_rad': -math.radians(.5)}
        with patch.object(self.motor, 'enter_rest_mode', return_value=feedback):
            self.bridge.update(0, False)
            with self.assertRaisesRegex(RuntimeError, 'outside configured signed command limits'):
                self.bridge.arm()
        self.assertNotIn('arm', self.motor.calls)

    def test_console_reads_guarded_scale_without_widening_profile(self):
        console = MotorConsole(self.bridge, None)
        console.read('knee')
        status = console.status()['rotary']['knee']
        self.assertAlmostEqual(status['hardware_limits_deg'][0], -1, places=5)
        self.assertAlmostEqual(status['hardware_limits_deg'][1], math.degrees(THETA_MAX)+1, places=5)
        self.assertEqual(status['limits_deg'], [0,105])
        self.assertEqual(console.profile()['rotary']['knee']['limits_deg'], [0,105])
        with self.assertRaises(ValueError):
            console.apply('knee', {'limits_deg': [-.5,105]})

    def test_driver_encodes_zero_inside_guarded_range_and_decodes_same_scale(self):
        lo, hi = self.motor.parameters[24], self.motor.parameters[25]
        raw = MotorevoMotor._float_to_uint(0, lo, hi, 16)
        with patch('motorevo_driver.can.Bus') as factory:
            driver = MotorevoMotor(theta_min=lo, theta_max=hi)
            factory.return_value.recv.return_value = can.Message(arbitration_id=1, is_extended_id=False,
                data=bytes([2,raw>>8,raw&255,0x7f,0xf7,0xff,0,23]))
            feedback = driver.send_ptm_command(0, kp=20, kd=4)
            frame = factory.return_value.send.call_args.args[0]
            self.assertEqual(int.from_bytes(frame.data[:2], 'big'), raw)
            self.assertGreater(raw, 0)
            self.assertLess(abs(feedback['position_rad']), (hi-lo)/65535)

    def test_clipped_pre_zero_feedback_cannot_require_an_offset_change(self):
        before = {**self.motor.feedback(0), 'position_rad': self.motor.parameters[24]}
        after = {**self.motor.feedback(0), 'position_rad': 0}
        with patch.object(self.motor,'enter_rest_mode',side_effect=[before,after,after,after]), patch.object(self.motor,'set_zero_position',return_value=after,create=True):
            self.bridge.zero()
        self.assertTrue(self.bridge.zero_verification['command_confirmed'])
        self.assertEqual(self.motor.parameters[23], 0)
        self.assertFalse(self.bridge.armed)

    def test_wide_servo_range_scales_both_joints_without_widening_targets(self):
        for bridge_type, limits in ((KneeBridge, [0,105]), (HipBridge, [-30,30])):
            with self.subTest(joint=bridge_type.__name__):
                motor = ConsoleMotor()
                motor.parameters[24], motor.parameters[25] = -12.0, 12.0
                joint = bridge_type(lambda: motor)
                try:
                    with joint.lock:
                        joint.update(0, False)
                        joint.arm()
                        self.assertEqual((motor.theta_min, motor.theta_max), (-12,12))
                        self.assertEqual(joint.soft_limits_deg, limits)
                        joint._control_step(joint.last_update, .02)
                        self.assertAlmostEqual(motor.commands[-1][1], .2)
                        for angle in (limits[0]-.5, limits[1]+.5):
                            with self.assertRaises(ValueError):
                                joint.update(angle, False)
                        joint.disarm()
                        console = MotorConsole(joint, None)
                        console.read('knee')
                        self.assertEqual(console.status()['rotary']['knee']['limits_deg'], limits)
                        self.assertAlmostEqual(console.status()['rotary']['knee']['hardware_limits_deg'][1], math.degrees(12))
                finally:
                    joint.close()

    def test_partial_wide_range_is_rejected(self):
        self.motor.parameters[24], self.motor.parameters[25] = -12, 1.83
        self.bridge.update(0, False)
        with self.assertRaisesRegex(RuntimeError, 'does not match bridge scale'):
            self.bridge.arm()
        self.assertNotIn('arm', self.motor.calls)

    def test_driver_uses_wide_range_for_command_and_feedback(self):
        lo, hi, target = -12, 12, math.radians(5)
        raw = MotorevoMotor._float_to_uint(target, lo, hi, 16)
        with patch('motorevo_driver.can.Bus') as factory:
            driver = MotorevoMotor(theta_min=lo, theta_max=hi)
            factory.return_value.recv.return_value = can.Message(arbitration_id=1, is_extended_id=False,
                data=bytes([2,raw>>8,raw&255,0x7f,0xf7,0xff,0,23]))
            feedback = driver.send_ptm_command(target, kp=20, kd=4)
            self.assertEqual(int.from_bytes(factory.return_value.send.call_args.args[0].data[:2], 'big'), raw)
            self.assertLess(abs(feedback['position_rad']-target), (hi-lo)/65535)

    def test_knee_half_angle_tracks_position_velocity_and_feedback(self):
        self.motor.parameters[24], self.motor.parameters[25] = -12, 12
        joint = KneeBridge(lambda: self.motor, angle_scale=KNEE_ANGLE_SCALE, start_worker=False)
        try:
            joint.update(50, True, mode='auto', velocity_deg_s=20)
            joint.arm()
            joint._control_step(joint.last_update, .02)
            command = self.motor.commands[-1]
            self.assertAlmostEqual(command[1], math.radians(25))
            self.assertAlmostEqual(command[2]['vel_rad_s'], math.radians(10))
            self.assertAlmostEqual(joint.relative_degrees(command[1]), 50)
            self.assertAlmostEqual(joint.mapped_target(105), THETA_MAX/2)
            self.assertEqual(joint.status()['angle_scale'], .5)
            for angle in (-1,106):
                with self.assertRaises(ValueError):
                    joint.update(angle, True)
            joint.disarm()
            joint.reference_rad = .1
            self.assertAlmostEqual(joint.relative_degrees(joint.mapped_target(50)), 50)
            MotorConsole._check_rotary_limits(joint, [0,105], .1)
        finally:
            joint.close()

    def test_hip_retains_one_to_one_mapping(self):
        joint = HipBridge(lambda: self.motor)
        try:
            self.assertEqual(joint.angle_scale, 1)
            joint.update(20, True, mode='auto', velocity_deg_s=12)
            self.assertAlmostEqual(joint.goal, math.radians(20))
            self.assertAlmostEqual(joint.velocity, math.radians(12))
            self.assertAlmostEqual(joint.relative_degrees(joint.goal), 20)
        finally:
            joint.close()


if __name__ == '__main__':
    unittest.main()
