"""Hip routing and coordinated fail-safe tests; no physical CAN hardware."""
import json
import math
import threading
import time
import unittest
from unittest.mock import patch
from urllib.request import Request, urlopen

import can
from knee_bridge import KneeBridge, HipBridge, BridgeServer, make_handler, THETA_MAX, HIP_THETA_MIN, HIP_THETA_MAX
from motorevo_driver import SharedMotorNetwork
from test_knee_bridge import FakeMotor


def wait_for(predicate, timeout=0.8):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError('Timed out waiting for bridge state')


class FakeHipMotor(FakeMotor):
    def __init__(self):
        super().__init__()
        self.parameters[24] = HIP_THETA_MIN
        self.parameters[25] = HIP_THETA_MAX


class HipTests(unittest.TestCase):
    def setUp(self):
        self.knee_motor, self.hip_motor = FakeMotor(), FakeHipMotor()
        self.hip = HipBridge(lambda: self.hip_motor)
        self.bridge = KneeBridge(lambda: self.knee_motor, hip=self.hip)

    def tearDown(self):
        self.bridge.close()

    def update(self, knee=30, hip=-12, running=True, **kwargs):
        self.bridge.update(knee, running, hip_angle_deg=hip, **kwargs)

    def test_independent_signed_targets_and_velocity(self):
        self.update(mode='auto', velocity_deg_s=12, hip_velocity_deg_s=-8)
        self.bridge.arm()
        self.update(mode='auto', velocity_deg_s=12, hip_velocity_deg_s=-8)
        wait_for(lambda: self.hip_motor.commands and self.hip_motor.commands[-1][2]['vel_rad_s'] < 0)
        self.assertAlmostEqual(self.knee_motor.commands[-1][1], math.radians(30))
        self.assertAlmostEqual(self.hip_motor.commands[-1][1], math.radians(-12))
        self.assertAlmostEqual(self.hip_motor.commands[-1][2]['vel_rad_s'], math.radians(-8))
        self.assertEqual(self.bridge.status()['hip']['motor_id'], 2)
        self.assertTrue(self.hip.samples)

    def test_hip_full_range_and_rejected_targets_do_not_mutate_knee(self):
        for angle, expected in ((-30, -math.pi/6), (0, 0), (30, math.pi/6)):
            self.update(hip=angle)
            self.assertAlmostEqual(self.hip.goal, expected)
        previous = self.bridge.goal
        for angle in (None, True, -30.1, 30.1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                self.update(knee=90, hip=angle)
            self.assertEqual(self.bridge.goal, previous)
        with self.assertRaises(ValueError):
            self.update(hip_velocity_deg_s=True)

    def test_old_hip_scale_is_rejected_before_arming(self):
        self.hip_motor.parameters[24] = 0.0
        self.hip_motor.parameters[25] = 1.83
        self.update()
        with self.assertRaisesRegex(RuntimeError, 'does not match bridge scale'):
            self.bridge.arm()
        self.assertFalse(self.hip_motor.commands)
        self.assertNotIn('arm', self.hip_motor.calls)
        self.assertFalse(self.bridge.armed)

    def test_optional_soft_hip_gains_are_independent_of_knee_gains(self):
        self.hip.gains = dict(kp_pos=2.0, kd_pos=0.5)
        self.update()
        self.bridge.arm()
        wait_for(lambda: self.hip_motor.commands)
        self.assertEqual(self.hip_motor.commands[-1][2]['kp'], 2.0)
        self.assertEqual(self.hip_motor.commands[-1][2]['kd'], 0.5)
        self.assertEqual(self.knee_motor.commands[-1][2]['kp'], 20.0)
        self.assertEqual(self.knee_motor.commands[-1][2]['kd'], 4.0)

    def test_default_tracking_gains_and_intermediate_manual_targets(self):
        self.update(hip=-25)
        self.bridge.arm()
        # Step deterministically through the actual control path at non-endpoints.
        self.bridge.stop_event.set()
        self.bridge.thread.join()
        for angle in (-25, -17.5, -10, -2.5, 2.5, 10, 17.5, 25):
            self.update(hip=angle)
            self.hip._control_step(time.monotonic(), 0.02)
            command = self.hip_motor.commands[-1]
            self.assertAlmostEqual(command[1], math.radians(angle))
            self.assertEqual(command[2]['kp'], 20.0)
            self.assertEqual(command[2]['kd'], 4.0)

    def test_hip_zero_uses_neutral_and_signed_configuration(self):
        old = self.hip_motor.feedback(0)
        zeroed = {**old, 'position_rad': 0.0}
        def accept_zero():
            self.hip_motor.parameters[23] = 0.2
            return old
        with patch.object(self.hip_motor, 'enter_rest_mode', side_effect=[old,zeroed,zeroed,zeroed]), patch.object(self.hip_motor, 'set_zero_position', side_effect=accept_zero, create=True):
            self.hip.zero()
        self.assertTrue(self.hip.zero_verification['command_confirmed'])
        self.assertEqual(self.hip.goal, 0.0)
        self.assertEqual(self.hip.status()['zero_angle_deg'], 0.0)
        self.assertFalse(self.hip.armed)

    def test_stop_holds_and_disarm_closes_both(self):
        self.update()
        self.bridge.arm()
        wait_for(lambda: self.hip_motor.commands)
        held = self.hip.setpoint
        self.update(hip=25, running=False)
        time.sleep(0.04)
        self.assertEqual(self.hip_motor.commands[-1][1], held)
        self.assertEqual(self.hip_motor.commands[-1][2]['vel_rad_s'], 0)
        self.bridge.disarm()
        self.assertFalse(self.hip.armed)
        self.assertEqual(self.knee_motor.calls[-1], 'close')
        self.assertEqual(self.hip_motor.calls[-1], 'close')

    def test_hip_fault_disarms_both(self):
        self.update()
        self.bridge.arm()
        self.hip_motor.fault = 2
        wait_for(lambda: not self.bridge.armed)
        self.assertFalse(self.hip.armed)
        self.assertIn('Hip:', self.bridge.error)
        self.assertIn('fault', self.hip.error)
        self.assertEqual(self.knee_motor.calls[-1], 'close')

    def test_hip_timeout_disarms_both(self):
        self.update()
        self.bridge.arm()
        with patch.object(self.hip_motor, 'send_ptm_command', return_value=None):
            wait_for(lambda: not self.bridge.armed)
        self.assertFalse(self.hip.armed)
        self.assertIn('Hip: Motor feedback timed out', self.bridge.error)

    def test_knee_fault_and_browser_loss_disarm_hip(self):
        self.update()
        self.bridge.arm()
        self.knee_motor.fault = 2
        wait_for(lambda: not self.hip.armed)
        self.knee_motor.fault = 0
        self.update()
        self.bridge.arm()
        wait_for(lambda: not self.hip.armed)
        self.assertIn('connection lost', self.bridge.error)

    def test_failed_hip_configuration_rolls_back_both(self):
        self.hip_motor.parameters[11] = 1
        self.update()
        with self.assertRaises(Exception):
            self.bridge.arm()
        self.assertFalse(self.bridge.armed)
        self.assertFalse(self.hip.armed)
        self.assertEqual(self.knee_motor.calls[-1], 'close')
        self.assertEqual(self.hip_motor.calls[-1], 'close')
        self.assertFalse(self.knee_motor.commands)

    def test_knee_arm_failure_after_hip_arm_rolls_back_hip(self):
        self.update()
        with patch.object(self.knee_motor, 'enter_motor_mode', return_value=self.knee_motor.feedback(1)):
            with self.assertRaisesRegex(RuntimeError, 'P-T-M'):
                self.bridge.arm()
        self.assertIn('arm', self.hip_motor.calls)
        self.assertEqual(self.hip_motor.calls[-1], 'close')
        self.assertFalse(self.hip.armed)
        self.assertFalse(self.bridge.armed)

    def test_http_hip_target_and_telemetry(self):
        server = BridgeServer(('127.0.0.1', 0), make_handler(self.bridge, 'test', 0))
        port = server.server_address[1]
        server.RequestHandlerClass = make_handler(self.bridge, 'test', port)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        base = f'http://127.0.0.1:{port}'
        try:
            payload = dict(angle_deg=45, hip_angle_deg=-10, running=True, mode='auto', hip_velocity_deg_s=5)
            headers = {'Content-Type': 'application/json', 'Origin': base, 'X-Bridge-Token': 'test'}
            with urlopen(Request(base+'/api/update', data=json.dumps(payload).encode(), headers=headers)) as response:
                status = json.load(response)
            self.assertAlmostEqual(status['hip']['goal_rad'], math.radians(-10))
            self.assertAlmostEqual(status['goal_rad'], math.radians(45))
            with urlopen(base+'/api/telemetry') as response:
                self.assertIn('hip_samples', json.load(response))
        finally:
            server.shutdown()
            server.server_close()
            worker.join()


class HipAnkleTests(unittest.TestCase):
    def test_hold_during_ankle_startup_and_ankle_fault_disarms_hip(self):
        from ankle_bridge import AnkleBridge
        from test_ankle_bridge import FakeActuators
        driver = FakeActuators()
        driver.verify_gate = threading.Event()
        ankle = AnkleBridge(lambda: driver)
        hip_motor = FakeHipMotor()
        hip = HipBridge(lambda: hip_motor)
        bridge = KneeBridge(FakeMotor, hip=hip, ankle=ankle)
        try:
            bridge.update(60, True, extensions_mm=[30,30,30], hip_angle_deg=20)
            bridge.arm()
            wait_for(lambda: len(driver.moves) >= 3 and hip_motor.commands)
            self.assertFalse(ankle.status()['armed'])
            self.assertTrue(all(abs(command[1]-0.2)<1e-9 for command in hip_motor.commands))
            bridge.update(60, True, extensions_mm=[30,30,30], hip_angle_deg=20)
            driver.verify_gate.set()
            wait_for(lambda: ankle.status()['armed'])
            wait_for(lambda: abs(hip_motor.commands[-1][1]-math.radians(20))<1e-9)
            driver.fault = True
            wait_for(lambda: not hip.armed)
            self.assertFalse(bridge.armed)
        finally:
            driver.verify_gate.set()
            bridge.close()


class SharedNetworkTests(unittest.TestCase):
    def test_shared_bus_uses_different_position_encoding_for_each_joint(self):
        with patch('motorevo_driver.can.Bus') as factory:
            network = SharedMotorNetwork(interface='pcan', channel='PCAN_USBBUS1',
                                         bitrate=1000000, theta_min=0, theta_max=1.83)
            try:
                knee = network.motor(1)
                hip = network.motor(2, theta_min=-math.pi/6, theta_max=math.pi/6)
                self.assertIs(knee.bus, hip.bus)
                bus = factory.return_value
                for angle, raw in ((-30,0), (0,32767), (30,65535)):
                    bus.recv.return_value = can.Message(arbitration_id=2,
                        data=[2,raw>>8,raw&255,0,0,0,0,20], is_extended_id=False)
                    reply = hip.send_ptm_command(math.radians(angle))
                    sent = bus.send.call_args.args[0]
                    self.assertEqual((sent.data[0]<<8)|sent.data[1], raw)
                    self.assertAlmostEqual(reply['position_rad'], math.radians(angle), delta=math.pi/3/65535)
                bus.recv.return_value = can.Message(arbitration_id=1,data=[2]*8,is_extended_id=False)
                knee.send_ptm_command(math.radians(30))
                sent = bus.send.call_args.args[0]
                self.assertEqual((sent.data[0]<<8)|sent.data[1],18750)
                factory.assert_called_once()
            finally:
                network.close()

    def test_concurrent_motor_transactions_cannot_steal_replies(self):
        from concurrent.futures import ThreadPoolExecutor
        class Adapter:
            def send(self, message):
                self.pending_id = message.arbitration_id
                time.sleep(0.003)

            def recv(self, timeout):
                return can.Message(arbitration_id=self.pending_id,
                    data=[2,0,0,0,0,0,0,self.pending_id], is_extended_id=False)

            def shutdown(self):
                pass

        with patch('motorevo_driver.can.Bus', return_value=Adapter()):
            network = SharedMotorNetwork(interface='pcan', channel='PCAN_USBBUS1', bitrate=1000000)
            try:
                motors = [network.motor(1), network.motor(2)]
                def commands(motor):
                    for _ in range(10):
                        reply = motor.send_ptm_command(0)
                        self.assertIsNotNone(reply)
                        self.assertEqual(reply['temp_c'], motor.motor_id)
                with ThreadPoolExecutor(max_workers=2) as executor:
                    list(executor.map(commands, motors))
            finally:
                network.close()

    def test_single_bus_node_routing_and_shutdown_ownership(self):
        with patch('motorevo_driver.can.Bus') as factory:
            network = SharedMotorNetwork(interface='pcan', channel='PCAN_USBBUS1', bitrate=1000000)
            self.assertEqual(factory.call_count, 0)
            knee, hip = network.motor(1), network.motor(2)
            self.assertIs(knee.bus, hip.bus)
            self.assertIs(knee.bus_lock, hip.bus_lock)
            self.assertEqual(factory.call_count, 1)
            bus = factory.return_value
            # A frame for knee must never become hip feedback.
            bus.recv.side_effect = [can.Message(arbitration_id=1, data=[0]*8, is_extended_id=False),
                                    can.Message(arbitration_id=2, data=[2,0,0,0,0,0,0,20], is_extended_id=False)]
            feedback = hip.enter_motor_mode()
            self.assertEqual(feedback['status'], 2)
            self.assertEqual(bus.send.call_args.args[0].arbitration_id, 2)
            bus.recv.side_effect = None
            bus.recv.return_value = can.Message(arbitration_id=1, data=[0]*8, is_extended_id=False)
            knee.close()
            bus.shutdown.assert_not_called()
            network.close()
            network.close()
            bus.shutdown.assert_called_once()


if __name__ == '__main__':
    unittest.main()
