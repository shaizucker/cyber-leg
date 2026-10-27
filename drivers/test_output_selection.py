"""Independent physical-system selections, using fake hardware only."""
import math
import sys
import time
import unittest
from unittest.mock import patch

from knee_bridge import KneeBridge, HipBridge, select_outputs, main
from ankle_bridge import AnkleBridge
from test_knee_bridge import FakeMotor
from test_ankle_bridge import FakeActuators
from test_hip_bridge import wait_for, FakeHipMotor


class OutputSelectionTests(unittest.TestCase):
    def test_selection_flags_and_empty_selection(self):
        self.assertEqual(select_outputs(), {'knee', 'hip', 'ankle'})
        for name in ('knee', 'hip', 'ankle'):
            self.assertEqual(select_outputs(name), {name})
        self.assertEqual(select_outputs(no_knee=True), {'hip', 'ankle'})
        self.assertEqual(select_outputs(no_hip=True, no_ankle=True), {'knee'})
        for kwargs in (dict(only='other'), dict(only='knee', no_knee=True),
                       dict(no_knee=True, no_hip=True, no_ankle=True)):
            with self.assertRaises(ValueError):
                select_outputs(**kwargs)

    def test_knee_only_tracks_without_other_outputs(self):
        motor = FakeMotor()
        bridge = KneeBridge(lambda: motor)
        try:
            bridge.update(45, True)
            bridge.arm()
            wait_for(lambda: motor.commands)
            self.assertAlmostEqual(motor.commands[-1][1], math.pi/4)
            self.assertEqual(bridge.status()['outputs'], ['knee'])
            self.assertFalse(bridge.status()['hip']['enabled'])
            self.assertFalse(bridge.status()['ankle']['enabled'])
        finally:
            bridge.close()

    def test_hip_only_tracks_holds_and_faults_without_knee_targets(self):
        motor = FakeHipMotor()
        hip = HipBridge(lambda: motor)
        bridge = KneeBridge(None, hip=hip)
        try:
            bridge.update(None, True, hip_angle_deg=20)
            bridge.arm()
            wait_for(lambda: motor.commands)
            self.assertAlmostEqual(motor.commands[-1][1], math.radians(20))
            self.assertEqual(bridge.status()['outputs'], ['hip'])
            self.assertIsNone(bridge.motor)
            self.assertFalse(bridge.status()['knee_enabled'])
            with self.assertRaisesRegex(ValueError, 'disabled'):
                bridge.zero()
            bridge.update(None, False, hip_angle_deg=-10)
            time.sleep(0.04)
            self.assertAlmostEqual(motor.commands[-1][1], math.radians(20))
            motor.fault = 2
            wait_for(lambda: not bridge.armed)
            self.assertFalse(hip.armed)
        finally:
            bridge.close()

    def test_hip_only_stale_client_disarms(self):
        motor = FakeHipMotor()
        bridge = KneeBridge(None, hip=HipBridge(lambda: motor))
        try:
            bridge.update(None, True, hip_angle_deg=0)
            bridge.arm()
            wait_for(lambda: not bridge.armed)
            self.assertEqual(motor.calls[-1], 'close')
            self.assertIn('connection lost', bridge.error)
        finally:
            bridge.close()

    def test_ankle_only_tracks_holds_and_faults_without_rotary_hardware(self):
        driver = FakeActuators()
        ankle = AnkleBridge(lambda: driver)
        bridge = KneeBridge(None, ankle=ankle)
        try:
            bridge.update(None, True, extensions_mm=[25, 26, 27])
            bridge.arm()
            wait_for(lambda: ankle.status()['armed'])
            wait_for(lambda: driver.moves[-1][0] > 20)
            self.assertEqual(bridge.status()['outputs'], ['ankle'])
            self.assertIsNone(bridge.motor)
            bridge.update(None, False, extensions_mm=[10, 10, 10])
            time.sleep(0.04)
            held = list(driver.moves[-1])
            time.sleep(0.04)
            self.assertEqual(driver.moves[-1], held)
            driver.fault = True
            wait_for(lambda: not bridge.armed)
            wait_for(lambda: driver.closed)
        finally:
            bridge.close()

    def test_ankle_only_disconnect_closes_adapter(self):
        driver = FakeActuators()
        bridge = KneeBridge(None, ankle=AnkleBridge(lambda: driver))
        try:
            bridge.update(None, True, extensions_mm=[20,20,20])
            bridge.arm()
            wait_for(lambda: driver.closed)
            wait_for(lambda: not bridge.armed)
        finally:
            bridge.close()

    def test_no_outputs_cannot_arm(self):
        bridge = KneeBridge(None)
        try:
            bridge.update(None, True)
            with self.assertRaisesRegex(ValueError, 'at least one'):
                bridge.arm()
        finally:
            bridge.close()

    def test_hip_only_can_frames_never_address_knee(self):
        import struct
        import can
        from motorevo_driver import SharedMotorNetwork

        class Adapter:
            def __init__(self):
                self.sent = []

            def send(self, message):
                self.sent.append(message)

            def recv(self, timeout):
                request = self.sent[-1]
                node = request.arbitration_id
                if node == 0x602:
                    index = request.data[1]
                    value = FakeHipMotor().parameters[index]
                    kind = '<I' if index == 56 else '<i' if index in (11,37,47) else '<f'
                    raw = bytes([2,index]) + struct.pack(kind,value) + bytes([0,255])
                else:
                    status = 0 if request.data[-1] == 0xFD else 2
                    raw = bytes([status,0,0,0,0,0,0,20])
                return can.Message(arbitration_id=node, data=raw, is_extended_id=False)

            def shutdown(self):
                pass

        adapter = Adapter()
        with patch('motorevo_driver.can.Bus', return_value=adapter):
            network = SharedMotorNetwork(interface='pcan', channel='PCAN_USBBUS1',
                                         bitrate=1000000, theta_min=0, theta_max=1.83)
            bridge = KneeBridge(None, hip=HipBridge(lambda: network.motor(2, theta_min=-math.pi/6, theta_max=math.pi/6)))
            try:
                bridge.update(None, True, hip_angle_deg=20)
                bridge.arm()
                wait_for(lambda: bridge.hip.samples)
                bridge.disarm()
                self.assertEqual({frame.arbitration_id for frame in adapter.sent}, {2,0x602})
                self.assertTrue(all(not frame.is_extended_id for frame in adapter.sent))
                self.assertFalse(bridge.status()['knee_enabled'])
            finally:
                bridge.close()
                network.close()

    def test_cli_only_constructs_selected_hardware(self):
        for selected in ('knee', 'hip', 'ankle'):
            with self.subTest(selected=selected), patch.object(sys, 'argv', ['knee_bridge.py', '--no-config-save', '--only', selected]), patch('motorevo_driver.SharedMotorNetwork') as network_factory, patch('seenpin_driver.SeenpinActuators', return_value=FakeActuators()) as ankle_factory, patch('knee_bridge.CanTrace'), patch('knee_bridge.BridgeServer') as server_factory, patch('knee_bridge.make_handler') as handler, patch('builtins.print') as output:
                network_factory.return_value.motor.side_effect = lambda node, **kwargs: FakeHipMotor() if node == 2 else FakeMotor()
                def run_once():
                    bridge = handler.call_args.args[0]
                    self.assertEqual(bridge.status()['outputs'], [selected])
                    bridge.update(30, True, hip_angle_deg=0, extensions_mm=[20,20,20])
                    bridge.arm()
                    if bridge.ankle:
                        wait_for(lambda: bridge.ankle.status()['armed'])
                server_factory.return_value.serve_forever.side_effect = run_once
                main()
                startup = '\n'.join(str(call.args[0]) for call in output.call_args_list if call.args)
                if selected != 'knee':
                    self.assertIn('Knee disabled (node 1 is not contacted)', startup)
                if selected == 'hip':
                    self.assertIn('Hip: classic CAN node 2 on ', startup)
                    self.assertNotIn('with knee node 1', startup)
                expected_nodes = {'knee': [1], 'hip': [2], 'ankle': []}[selected]
                self.assertEqual([call.args[0] for call in network_factory.return_value.motor.call_args_list], expected_nodes)
                self.assertEqual(ankle_factory.call_count, int(selected == 'ankle'))
                if selected == 'hip':
                    options = network_factory.return_value.motor.call_args.kwargs
                    self.assertAlmostEqual(options['theta_min'], -math.pi/6)
                    self.assertAlmostEqual(options['theta_max'], math.pi/6)


if __name__ == '__main__':
    unittest.main()
