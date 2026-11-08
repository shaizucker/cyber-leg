"""Deterministic delayed-USB tests; no real CAN devices or wall-clock sleeps."""
import unittest
from unittest.mock import patch, Mock
import can
from knee_bridge import KneeBridge, HipBridge, FEEDBACK_TIMEOUT, CONTROL_OVERRUN_YIELD, DT
from motorevo_driver import MotorevoMotor


class Clock:
    now = 100.0
    def __call__(self):
        return self.now


class DelayedBus:
    def __init__(self, clock, delay, fault=0):
        self.clock, self.delay, self.fault = clock, delay, fault
        self.sent = []
        self.pending = None

    def send(self, frame):
        self.sent.append(frame)
        rest = bytes(frame.data) == bytes.fromhex('FF FF FF FF FF FF FF FD')
        data = bytes([0 if rest else 2,0x20,0,0x7f,0xf7,0xff,0 if rest else self.fault,23])
        self.pending = (self.clock.now+(0 if rest else self.delay),
                        can.Message(arbitration_id=frame.arbitration_id,is_extended_id=False,data=data))

    def recv(self, timeout):
        if self.pending and self.pending[0] <= self.clock.now+timeout:
            self.clock.now = max(self.clock.now,self.pending[0])
            _,frame = self.pending
            self.pending = None
            return frame
        self.clock.now += timeout
        return None


class RotaryTimingTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.patch = patch('time.monotonic', self.clock)
        self.patch.start()

    def tearDown(self):
        self.patch.stop()

    def joint(self, timeout=FEEDBACK_TIMEOUT, delay=.015, fault=0):
        bus = DelayedBus(self.clock,delay,fault)
        motor = MotorevoMotor(motor_id=1,bus=bus,theta_min=0,theta_max=1.83)
        joint = KneeBridge(lambda:motor,start_worker=False,feedback_timeout=timeout)
        joint.motor = motor
        joint.update(10,True)
        return joint,bus

    def test_measured_usb_outlier_passes_30_ms_but_fails_old_10_ms(self):
        for timeout, passes in ((.01,False),(.03,True)):
            joint,bus = self.joint(timeout=timeout)
            try:
                if passes:
                    joint._control_step(self.clock.now,.02)
                    self.assertEqual(joint.feedback['fault'],0)
                else:
                    with self.assertRaisesRegex(RuntimeError,'feedback timed out'):
                        joint._control_step(self.clock.now,.02)
                self.assertEqual(len(bus.sent),1)
            finally:
                joint.close()

    def test_reply_beyond_budget_still_stops_without_retrying_command(self):
        joint,bus = self.joint(delay=.031)
        try:
            start = self.clock.now
            with self.assertRaisesRegex(RuntimeError,'timed out after 30 ms'):
                joint._control_step(start,.02)
            self.assertAlmostEqual(self.clock.now-start,.03)
            self.assertEqual(len(bus.sent),1)
        finally:
            joint.close()

    def test_delayed_fault_is_not_ignored(self):
        joint,bus = self.joint(delay=.02,fault=13)
        try:
            with self.assertRaisesRegex(RuntimeError,'Position Out of Range'):
                joint._control_step(self.clock.now,.02)
            self.assertEqual(len(bus.sent),1)
        finally:
            joint.close()

    def test_hip_checks_heartbeat_again_after_knee_used_reply_time(self):
        joint,bus = self.joint(delay=.02)
        hip_motor = MotorevoMotor(motor_id=2,bus=bus)
        hip = HipBridge(lambda:hip_motor)
        hip.motor = hip_motor
        hip.update(0,True)
        joint.hip = hip
        try:
            self.clock.now += .49
            joint._control_step(self.clock.now,.02)
            with self.assertRaisesRegex(RuntimeError,'Simulator connection lost'):
                hip._control_step(100.49,.02)
            self.assertEqual([frame.arbitration_id for frame in bus.sent],[1])
        finally:
            joint.close()

    def test_deadline_configuration_is_bounded(self):
        for invalid in (0,.009,.051,float('inf'),float('nan'),True):
            with self.subTest(value=invalid), self.assertRaises(ValueError):
                KneeBridge(None,start_worker=False,feedback_timeout=invalid)


class LoopSchedulingTests(unittest.TestCase):
    def run_cycle(self, duration):
        clock = Clock()
        joint = KneeBridge(None,start_worker=False)
        joint.knee_enabled = True
        joint.armed = True
        joint.last_update = clock.now
        joint.stop_event = Mock()
        joint.stop_event.is_set.side_effect = [False,True]
        def control_step(now, elapsed):
            clock.now += duration
        joint._control_step = Mock(side_effect=control_step)
        try:
            with patch('time.monotonic',clock):
                joint._loop()
            self.assertTrue(joint.armed)
            self.assertIsNone(joint.error)
            return joint.stop_event.wait.call_args.args[0]
        finally:
            joint.close()

    def test_overrun_yields_instead_of_starving_browser_update_lock(self):
        self.assertAlmostEqual(self.run_cycle(.03),CONTROL_OVERRUN_YIELD)

    def test_fast_cycle_keeps_nominal_50_hz_period(self):
        self.assertAlmostEqual(self.run_cycle(.005),DT-.005)


if __name__ == '__main__':
    unittest.main()
