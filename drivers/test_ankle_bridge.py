import json
import math
import struct
import threading
import time
import unittest
from unittest.mock import patch
from ankle_bridge import AnkleBridge
from seenpin_driver import SeenpinActuators, position_payload, parse_frame
from http.server import ThreadingHTTPServer
from urllib.request import Request, urlopen
from knee_bridge import KneeBridge, make_handler
from test_knee_bridge import FakeMotor


def wait_for(predicate, timeout=0.5):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError('Condition timed out')


class SerialPort:
    def __init__(self, *args, **kwargs):
        self.sent = []
        self.pending = bytearray()
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.pending)

    def write(self, raw):
        self.sent.append(raw)
        frame = parse_frame(raw.decode().strip())
        if frame:
            can_id, data, kind = frame
            if 0x601 <= can_id <= 0x603 and data[0] == 0x40:
                index, sub = struct.unpack('<HB', data[1:4])
                value = {(0x2010, 5): 0, (0x2010, 2): 2000,
                         (0x607D, 1): 0, (0x607D, 2): 3900}[index, sub]
                response = bytes([0x4B]) + data[1:4] + struct.pack('<h', value)
                self.pending.extend(f'b{can_id-0x80:03X}6{response.hex()}\r'.encode())
        return len(raw)

    def read(self, count):
        result = bytes(self.pending[:count])
        del self.pending[:count]
        if not result:
            time.sleep(0.001)
        return result

    def reset_input_buffer(self):
        self.pending.clear()

    def close(self):
        self.closed = True


class LossySerialPort(SerialPort):
    def __init__(self, drop=0, response=None):
        super().__init__()
        self.drop, self.response = drop, response

    def write(self, raw):
        frame = parse_frame(raw.decode().strip())
        if frame and frame[0] == 0x603 and frame[1][:4] == bytes.fromhex('40 10 20 05'):
            if self.drop:
                self.drop -= 1
                self.sent.append(raw)
                # Traffic from other nodes must not count as node 3's reply.
                self.pending.extend(b'b58164B1020050000\r')
                return len(raw)
            if self.response is not None:
                self.sent.append(raw)
                self.pending.extend(self.response)
                return len(raw)
        return super().write(raw)


class FakeActuators:
    def __init__(self):
        self.limits = [[0, 39]] * 3
        self.positions = [20, 20, 20]
        self.moves = []
        self.refresh = None
        self.enabled = self.closed = self.fault = False
        self.gate = None
        self.verify_gate = None
        self.startup_error = None

    def initialize(self):
        if self.gate:
            self.gate.wait(0.3)

    def validate_targets(self, targets):
        if any(not 0 <= p <= 39 for p in targets):
            raise ValueError('drive limits')

    def enable(self):
        self.enabled = True

    def verify_startup(self):
        if self.verify_gate:
            while not self.verify_gate.wait(0.002):
                if self.refresh:
                    self.refresh()
        if self.startup_error:
            raise RuntimeError(self.startup_error)

    def move(self, targets):
        self.validate_targets(targets)
        self.moves.append(list(targets))
        self.positions = list(targets)

    def poll(self, index):
        if self.fault:
            raise RuntimeError('Ankle node fault')

    def listen(self, seconds):
        if self.refresh:
            self.refresh()
        time.sleep(0.002)

    def close(self):
        self.closed = True


class TransportTests(unittest.TestCase):
    def test_packets_limits_group_validation_and_shutdown(self):
        port = SerialPort()
        driver = SeenpinActuators(serial_factory=lambda *a, **k: port)
        with patch('seenpin_driver.time.sleep', return_value=None):
            driver.initialize()
        self.assertEqual(driver.limits, [[0, 39]] * 3)
        self.assertEqual(driver.positions, [20, 20, 20])
        self.assertEqual(port.sent[:7], [x.encode() + b'\r' for x in ('C','V','S8','Y5','M0','A0','O')])
        driver.enable()
        driver.move([21, 22, 23])
        for node, target, raw in zip((1, 2, 3), (2100, 2200, 2300), port.sent[-3:]):
            can_id, data, kind = parse_frame(raw.decode().strip())
            self.assertEqual((can_id, kind), (0x200 + node, 'b'))
            self.assertEqual(struct.unpack('<BhHH', data), (1, target, 100, 0xF))
        count = len(port.sent)
        with self.assertRaises(ValueError):
            driver.move([20, 40, 20])
        self.assertEqual(len(port.sent), count)
        driver.close()
        self.assertTrue(port.closed)
        for node, raw in zip((1, 2, 3), port.sent[-4:-1]):
            can_id, data, kind = parse_frame(raw.decode().strip())
            self.assertEqual(can_id, 0x200 + node)
            self.assertEqual(data, bytes.fromhex('00 00 00 00 00 04 00'))

    def test_latched_pdo_interruption_can_initialize_and_recover(self):
        port = SerialPort()
        logs = []
        driver = SeenpinActuators(serial_factory=lambda *a, **k: port, log=logs.append)
        def read_initial(node, index, sub, signed=False):
            return {(0x2010,5): 0x8002 if node == 3 else 0,
                    (0x2010,2): 2000, (0x607D,1): 0, (0x607D,2): 3900}[index,sub]
        with patch.object(driver, 'read', side_effect=read_initial), patch.object(driver, 'listen'), patch('seenpin_driver.time.sleep'):
            driver.initialize()
        self.assertEqual(driver.startup_faults, {3: 0x8002})
        driver.enable()
        controls = [parse_frame(raw.decode().strip()) for raw in port.sent[-4:]]
        self.assertEqual(controls[0][0], 0x203)
        self.assertEqual(controls[0][1], bytes.fromhex('00 00 00 00 00 04 00'))
        self.assertEqual([f[0] for f in controls[1:]], [0x201,0x202,0x203])
        with patch.object(driver, 'read', side_effect=[0,0,0x8002,0,0,0]), patch.object(driver, 'listen'):
            driver.verify_startup()
        self.assertFalse(driver.startup_faults)
        self.assertTrue(any('fault-free startup confirmed' in line for line in logs))

    def test_other_faults_block_initialization_and_persistent_pdo_fault_stops(self):
        port = SerialPort()
        driver = SeenpinActuators(serial_factory=lambda *a, **k: port)
        with patch.object(driver, 'read', return_value=0x1002), patch.object(driver, 'listen'), patch('seenpin_driver.time.sleep'):
            with self.assertRaisesRegex(RuntimeError, 'fault 0x1002'):
                driver.initialize()
        self.assertFalse(any(parse_frame(raw.decode().strip()) for raw in port.sent))
        with patch.object(driver, 'read', side_effect=[0,0,0x8002]*3), patch.object(driver, 'listen'):
            with self.assertRaisesRegex(RuntimeError, 'persists after re-enable'):
                driver.verify_startup()
        with patch.object(driver, 'read', side_effect=[0,0,0x7001]):
            with self.assertRaisesRegex(RuntimeError, 'fault 0x7001'):
                driver.verify_startup()

    def test_payload_signed_position_and_no_zero_duration(self):
        self.assertEqual(position_payload(25.67), bytes.fromhex('01 07 0A 64 00 0F 00'))
        for target, duration in ((math.nan, 20), (400, 20), (20, 0)):
            with self.assertRaises(ValueError):
                position_payload(target, duration)

    def test_node_three_recovers_after_two_missing_replies(self):
        port = LossySerialPort(drop=2)
        logs = []
        driver = SeenpinActuators(serial_factory=lambda *a, **k: port, log=logs.append)
        refreshes = []
        driver.refresh = lambda: refreshes.append(True)
        with patch('seenpin_driver.SDO_TIMEOUT', 0.015):
            self.assertEqual(driver.read(3, 0x2010, 5), 0)
        self.assertEqual(len(port.sent), 3)
        self.assertGreater(len(refreshes), 3)
        self.assertIn('retry 2/3', logs[0])
        self.assertIn('retry 3/3', logs[1])
        self.assertIn('recovered on attempt 3', logs[2])

    def test_persistent_missing_node_three_still_stops(self):
        port = LossySerialPort(drop=10)
        driver = SeenpinActuators(serial_factory=lambda *a, **k: port)
        with patch('seenpin_driver.SDO_TIMEOUT', 0.01):
            with self.assertRaisesRegex(RuntimeError, 'after 3 attempts; RX IDs: 0x581'):
                driver.read(3, 0x2010, 5)
        self.assertEqual(len(port.sent), 3)

    def test_actual_fault_and_sdo_abort_do_not_retry(self):
        for response, expected in (
                (b'b58364B1020050200\r', 'fault 0x0002'),
                (b'b58388010200500000000\r', 'SDO abort')):
            port = LossySerialPort(response=response)
            driver = SeenpinActuators(serial_factory=lambda *a, **k: port)
            with self.assertRaisesRegex(RuntimeError, expected):
                driver.poll(2)
            self.assertEqual(len(port.sent), 1)

    def test_disarm_interrupts_retry_wait_immediately(self):
        port = LossySerialPort(drop=10)
        driver = SeenpinActuators(serial_factory=lambda *a, **k: port)
        def cancelled():
            raise InterruptedError('Ankle disarmed')
        driver.refresh = cancelled
        with self.assertRaises(InterruptedError):
            driver.read(3, 0x2010, 5)
        self.assertEqual(len(port.sent), 1)

    def test_adapter_errors_and_feedback_timeout(self):
        port = SerialPort()
        driver = SeenpinActuators(serial_factory=lambda *a, **k: port)
        port.pending.extend(b'\a')
        with self.assertRaisesRegex(RuntimeError, 'BEL'):
            driver.listen(0.01)
        with self.assertRaisesRegex(RuntimeError, 'timeout'):
            driver.read(4, 0x2010, 5)


class ParallelBridgeTests(unittest.TestCase):
    def setUp(self):
        self.driver = FakeActuators()
        self.ankle = AnkleBridge(lambda: self.driver)
        self.motor = FakeMotor()
        self.bridge = KneeBridge(lambda: self.motor, ankle=self.ankle)

    def tearDown(self):
        self.bridge.close()

    def update(self, extensions, running=True):
        self.bridge.update(30, running, extensions_mm=extensions)

    def test_parallel_targets_stop_and_disarm(self):
        self.update([21, 22, 23])
        self.bridge.arm()
        wait_for(lambda: self.ankle.status()['armed'])
        self.assertEqual(self.driver.moves[0], [20, 20, 20])
        self.assertTrue(self.motor.commands)
        self.update([25, 26, 27])
        wait_for(lambda: self.driver.moves[-1][0] > 20)
        self.assertLess(self.driver.moves[-1][0], 25)
        self.update([10, 10, 10], False)
        time.sleep(0.03)
        held = list(self.driver.moves[-1])
        time.sleep(0.06)
        self.assertEqual(self.driver.moves[-1], held)
        self.bridge.disarm()
        wait_for(lambda: self.driver.closed)
        self.assertFalse(self.ankle.status()['armed'])

    def test_serial_initialization_does_not_block_knee_or_updates(self):
        self.driver.gate = threading.Event()
        self.update([20, 20, 20])
        self.bridge.arm()
        time.sleep(0.06)
        self.assertTrue(self.motor.commands)
        self.assertTrue(self.ankle.status()['arming'])
        self.update([22, 23, 24])
        self.driver.gate.set()
        wait_for(lambda: self.ankle.status()['armed'])
        self.assertEqual(self.driver.moves[0], [20, 20, 20])
        self.assertEqual(self.ankle.targets, [22, 23, 24])
        wait_for(lambda: self.driver.moves[-1][0] > 20)

    def test_both_joints_hold_until_ankle_startup_is_verified(self):
        self.driver.verify_gate = threading.Event()
        self.update([30, 30, 30])
        self.bridge.arm()
        wait_for(lambda: len(self.driver.moves) >= 3)
        self.assertFalse(self.ankle.status()['armed'])
        self.assertTrue(all(p == [20,20,20] for p in self.driver.moves))
        self.assertTrue(self.motor.commands)
        self.assertTrue(all(abs(c[1] - 0.2) < 1e-9 for c in self.motor.commands))
        self.update([30,30,30])
        self.driver.verify_gate.set()
        wait_for(lambda: self.ankle.status()['armed'])
        wait_for(lambda: self.driver.moves[-1][0] > 20)

    def test_failed_startup_verification_stops_both_without_tracking(self):
        self.driver.startup_error = 'PDO interruption persists'
        self.update([30,30,30])
        self.bridge.arm()
        wait_for(lambda: self.driver.closed)
        wait_for(lambda: not self.bridge.status()['armed'])
        self.assertTrue(all(p == [20,20,20] for p in self.driver.moves))
        self.assertIn('PDO interruption persists', self.bridge.error)

    def test_ankle_fault_stops_knee_and_all_actuators(self):
        self.update([20, 20, 20])
        self.bridge.arm()
        wait_for(lambda: self.ankle.status()['armed'])
        self.driver.fault = True
        wait_for(lambda: not self.bridge.status()['armed'])
        wait_for(lambda: self.driver.closed)
        self.assertIn('Ankle node fault', self.bridge.error)

    def test_knee_fault_stops_ankle(self):
        self.update([20, 20, 20])
        self.bridge.arm()
        wait_for(lambda: self.ankle.status()['armed'])
        self.motor.fault = 2
        wait_for(lambda: self.driver.closed)
        self.assertFalse(self.bridge.status()['armed'])

    def test_missing_invalid_and_out_of_range_targets(self):
        for targets in (None, [20, 20], [20, True, 20], [math.nan, 20, 20], [51, 20, 20]):
            with self.assertRaises(ValueError):
                self.update(targets)
        with self.assertRaises(ValueError):
            self.bridge.arm()
        self.update([20, 20, 20])
        self.bridge.arm()
        wait_for(lambda: self.ankle.status()['armed'])
        count = len(self.driver.moves)
        with self.assertRaises(ValueError):
            self.update([20, 40, 20])
        self.assertNotIn([20, 40, 20], self.driver.moves[count:])

    def test_http_routes_knee_and_three_ankle_targets_together(self):
        server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(self.bridge, 'test', 0))
        port = server.server_address[1]
        server.RequestHandlerClass = make_handler(self.bridge, 'test', port)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        base = f'http://127.0.0.1:{port}'
        try:
            body = json.dumps(dict(angle_deg=45, running=True, extensions_mm=[25, 26, 27])).encode()
            headers = {'Content-Type': 'application/json', 'Origin': base, 'X-Bridge-Token': 'test'}
            with urlopen(Request(base + '/api/update', data=body, headers=headers)) as response:
                data = json.load(response)
            self.assertAlmostEqual(data['goal_rad'], math.radians(45))
            self.assertEqual(self.ankle.targets, [25, 26, 27])
            self.assertTrue(data['ankle']['enabled'])
        finally:
            server.shutdown()
            server.server_close()
            worker.join()

    def test_cancel_during_initialization_never_enables(self):
        self.driver.gate = threading.Event()
        self.update([20, 20, 20])
        self.bridge.arm()
        time.sleep(0.02)
        self.bridge.disarm()
        self.driver.gate.set()
        wait_for(lambda: self.driver.closed)
        self.assertFalse(self.driver.enabled)

    def test_browser_disconnect_stops_all(self):
        self.update([20, 20, 20])
        self.bridge.arm()
        wait_for(lambda: self.ankle.status()['armed'])
        wait_for(lambda: self.driver.closed, timeout=0.7)
        # Independent workers can finish on adjacent control ticks.
        wait_for(lambda: not self.bridge.status()['armed'])


if __name__ == '__main__':
    unittest.main()
