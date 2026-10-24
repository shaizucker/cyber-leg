import struct
import unittest
from unittest.mock import patch
import can
from motorevo_driver import MotorevoMotor
from knee_safety import validate_configuration


class KneeSafetyTests(unittest.TestCase):
    def test_read_parameter_packet_and_filtered_float_response(self):
        wrong = can.Message(arbitration_id=0x601, is_extended_id=False, data=bytes([1, 24])+struct.pack('<f', 99)+bytes([0,255]))
        right = can.Message(arbitration_id=0x601, is_extended_id=False, data=bytes([1, 25])+struct.pack('<f', 1.83)+bytes([0,255]))
        with patch('motorevo_driver.can.Bus') as factory:
            factory.return_value.recv.side_effect = [wrong, right]
            motor = MotorevoMotor()
            self.assertAlmostEqual(motor.read_parameter(25), 1.83, places=6)
            sent = factory.return_value.send.call_args.args[0]
            self.assertEqual(sent.arbitration_id, 0x601)
            self.assertEqual(bytes(sent.data), bytes.fromhex('67 19 00 00 00 00 04 76'))

    def test_unsigned_protection_mask_is_not_decoded_as_float(self):
        response = can.Message(arbitration_id=0x601, is_extended_id=False, data=bytes([1,56])+struct.pack('<I',0xFFFFFF7C)+bytes([0,255]))
        with patch('motorevo_driver.can.Bus') as factory:
            factory.return_value.recv.return_value = response
            self.assertEqual(MotorevoMotor().read_parameter(56, 'uint'), 0xFFFFFF7C)

    def test_read_timeout_never_sends_zero_enable_or_write(self):
        with patch('motorevo_driver.can.Bus') as factory:
            factory.return_value.recv.return_value = None
            motor = MotorevoMotor()
            with self.assertRaises(RuntimeError):
                motor.read_parameter(23, timeout=0.001)
            for call in factory.return_value.send.call_args_list:
                frame = call.args[0]
                self.assertEqual(frame.arbitration_id, 0x601)
                self.assertEqual(frame.data[6], 0x04)

    def test_actual_readback_mask_has_three_disabled_protections(self):
        mask = 0xFFFFFF7C
        self.assertEqual(mask & 0x83, 0)
        self.assertEqual(mask | 0x83, 0xFFFFFFFF)
        config = dict(control_mode=2, machine_offset_rad=4.095345497,
                      theta_min_rad=0, theta_max_rad=1.8300000429,
                      rotation_direction=1, protection_mask=mask, can_timeout_ms=1000)
        with self.assertRaisesRegex(RuntimeError, 'position-limit protection is DISABLED'):
            validate_configuration(config)
        validate_configuration({**config, 'protection_mask': mask | 0x83})


if __name__ == '__main__':
    unittest.main()
