"""Linux/Windows transport selection, without opening physical hardware."""
import sys
import unittest
from unittest.mock import patch
from knee_bridge import main
from motorevo_driver import SharedMotorNetwork


class PlatformTransportTests(unittest.TestCase):
    def run_bridge(self, platform, arguments):
        with patch.object(sys, 'platform', platform), patch.object(sys, 'argv', ['knee_bridge.py', '--no-config-save', *arguments]), patch('motorevo_driver.SharedMotorNetwork') as network, patch('knee_bridge.CanTrace'), patch('knee_bridge.BridgeServer'), patch('knee_bridge.make_handler') as handler, patch('builtins.print') as output:
            main()
            bridge = handler.call_args.args[0]
            self.last_bridge = bridge
            ankle_port = bridge.ankle.factory.keywords['port'] if bridge.ankle else None
            return network.call_args.kwargs, ankle_port, '\n'.join(str(c.args[0]) for c in output.call_args_list if c.args)

    def test_linux_defaults_and_log(self):
        options, ankle_port, output = self.run_bridge('linux', [])
        self.assertEqual(options['interface'], 'socketcan')
        self.assertEqual(options['channel'], 'can1')
        self.assertEqual(options['bitrate'], 1000000)
        self.assertEqual(ankle_port, '/dev/ttyACM0')
        self.assertIn('node 1 on can1 (socketcan)', output)
        self.assertIn('node 2 on can1 (socketcan)', output)
        self.assertNotIn('PCAN_USBBUS1', output)

    def test_windows_defaults(self):
        options, ankle_port, output = self.run_bridge('win32', [])
        self.assertEqual(options['interface'], 'pcan')
        self.assertEqual(options['channel'], 'PCAN_USBBUS1')
        self.assertEqual(ankle_port, 'COM7')
        self.assertIn('node 2 on PCAN_USBBUS1 (pcan)', output)

    def test_explicit_socketcan_and_persistent_serial_path(self):
        path = '/dev/serial/by-id/usb-ankle-controller'
        options, port, output = self.run_bridge('win32', ['--can-interface', 'socketcan', '--can-channel', 'can7', '--ankle-port', path])
        self.assertEqual(options['interface'], 'socketcan')
        self.assertEqual(options['channel'], 'can7')
        self.assertEqual(port, path)
        self.assertIn('node 2 on can7 (socketcan)', output)

    def test_backend_override_selects_appropriate_default_channel(self):
        options, _, _ = self.run_bridge('linux', ['--can-interface', 'pcan', '--only', 'hip'])
        self.assertEqual(options['channel'], 'PCAN_USBBUS1')
        options, _, _ = self.run_bridge('win32', ['--can-interface', 'socketcan', '--only', 'hip'])
        self.assertEqual(options['channel'], 'can1')

    def test_teensy_slcan_requires_explicit_port_and_preserves_ankle_port(self):
        path = '/dev/serial/by-id/usb-Teensyduino_USB_Serial_TEENSY-if00'
        options, ankle_port, output = self.run_bridge('linux', ['--can-interface','slcan','--can-channel',path])
        self.assertEqual(options['interface'],'slcan')
        self.assertEqual(options['channel'],path)
        self.assertEqual(options['bitrate'],1000000)
        self.assertEqual(ankle_port,'/dev/ttyACM0')
        self.assertIn('(slcan)',output)
        with self.assertRaises(SystemExit), patch('sys.stderr'):
            self.run_bridge('linux',['--can-interface','slcan'])

    def test_teensy_uses_one_shared_bus_without_open_delay(self):
        with patch('motorevo_driver.can.Bus') as factory:
            network=SharedMotorNetwork(interface='slcan',channel='COM16',bitrate=1000000)
            knee,hip=network.motor(1),network.motor(2)
            self.assertIs(knee.bus,hip.bus)
            factory.assert_called_once_with(interface='slcan',channel='COM16',bitrate=1000000,
                                            sleep_after_open=0,tty_baudrate=115200)
            network.close()

    def test_feedback_timeout_is_applied_to_both_joints_and_bounded(self):
        options,_,output = self.run_bridge('linux',['--feedback-timeout-ms','40'])
        self.assertEqual(self.last_bridge.feedback_timeout,.04)
        self.assertEqual(self.last_bridge.hip.feedback_timeout,.04)
        self.assertIn('reply deadline 40 ms per joint',output)
        for value in ('9','51','nan'):
            with self.subTest(value=value), self.assertRaises(SystemExit), patch('sys.stderr'):
                self.run_bridge('linux',['--feedback-timeout-ms',value])


if __name__ == '__main__':
    unittest.main()
