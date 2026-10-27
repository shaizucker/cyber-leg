"""Online settings and protocol verification using fake hardware exclusively."""
import copy
import json
import math
import struct
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import can
from knee_bridge import KneeBridge, HipBridge
from ankle_bridge import AnkleBridge
from motor_console import MotorConsole
from motorevo_driver import MotorevoMotor
from seenpin_driver import SeenpinActuators, parse_frame
from test_knee_bridge import FakeMotor
from test_hip_bridge import FakeHipMotor
from test_ankle_bridge import FakeActuators


class ConsoleMotor(FakeMotor):
    def __init__(self, hip=False):
        super().__init__()
        self.parameters.update({21:1.0,46:1.0})
        if hip:
            self.parameters.update({24:-math.pi/6,25:math.pi/6})
        self.writes=[]

    def write_parameter(self,index,value):
        self.writes.append((index,value))
        self.parameters[index]=value
        return value


class ConsoleActuators(FakeActuators):
    def __init__(self):
        super().__init__()
        self.pid=[[1,.1,.01] for _ in range(3)]
        self.current=[200,200,200]
        self.telemetry={n:{} for n in (1,2,3)}
        self.writes=[]
        self.fail_init=False

    def initialize(self):
        if self.fail_init:
            raise RuntimeError('Fake actuator offline')

    def read(self,node,index,sub,signed=False):
        if index==0x2009:
            return self.current[node-1]
        if index==0x2004:
            return round(self.pid[node-1][sub-1]*100000)
        if index==0x2010 and sub==3:
            return 0
        raise AssertionError((node,index,sub))

    def read_position_pid(self):
        return copy.deepcopy(self.pid)

    def apply_position_pid(self,gains):
        self.pid=[list(gains) for _ in range(3)]
        return self.read_position_pid()

    def write(self,node,index,sub,value,size=4,signed=False):
        self.writes.append((node,index,sub,value))
        if index==0x2009:
            self.current[node-1]=value
        return value


class ConsoleTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.knee,self.hip_motor=ConsoleMotor(),ConsoleMotor(hip=True)
        self.actuators=ConsoleActuators()
        self.ankle=AnkleBridge(lambda:self.actuators)
        self.hip=HipBridge(lambda:self.hip_motor)
        self.bridge=KneeBridge(lambda:self.knee,hip=self.hip,ankle=self.ankle)
        self.path=Path(self.temp.name)/'settings.json'
        self.console=MotorConsole(self.bridge,self.path)

    def tearDown(self):
        self.bridge.close()
        self.temp.cleanup()

    def test_construction_and_status_never_connect_or_write(self):
        state=self.console.status()
        self.assertEqual(self.knee.calls,[])
        self.assertEqual(self.hip_motor.calls,[])
        self.assertIsNone(state['rotary']['hip']['power'])
        self.assertIsNone(state['ankle']['nodes'][0]['torque_nm'])
        self.assertFalse(self.path.exists())

    def test_read_reports_position_fault_without_clearing_zero_lock(self):
        feedback = {**self.knee.feedback(0), 'position_rad': 0.0,
                    'fault': 13, 'fault_desc': 'Position Out of Range'}
        self.bridge.zero_verification = dict(command_confirmed=False, error='previous zero failed')
        with patch.object(self.knee, 'enter_rest_mode', return_value=feedback):
            with self.assertRaisesRegex(RuntimeError, 'fault 13 .*Position Out of Range'):
                self.console.read('knee')
        self.assertEqual(self.bridge.feedback, feedback)
        self.assertFalse(self.bridge.zero_verification['command_confirmed'])
        self.assertNotIn('arm', self.knee.calls)

    def test_rotary_pid_live_changes_are_independent_and_saved(self):
        self.bridge.armed=True
        self.console.apply('hip',{'p':12,'i':0,'d':2})
        self.assertEqual(self.hip.gains,dict(kp_pos=12,kd_pos=2))
        self.assertEqual(self.bridge.gains,dict(kp_pos=20,kd_pos=4))
        self.assertEqual(self.knee.writes,[])
        self.assertEqual(json.loads(self.path.read_text())['rotary']['hip']['p'],12)

    def test_invalid_values_do_not_partially_change_pid_or_limits(self):
        before=self.console.profile()
        for changes in ({'p':math.nan},{'i':True},{'p':10,'d':51},{'limits_deg':[-40,30]},
                        {'limits_deg':[10,20]},{'unknown':1}):
            with self.assertRaises(ValueError):
                self.console.apply('hip',changes)
            self.assertEqual(self.console.profile(),before)
        self.assertFalse(self.hip_motor.writes)

    def test_limits_zero_and_hardware_reads_require_stop_and_disarm(self):
        for running,armed in ((True,False),(False,True)):
            self.bridge.running,self.bridge.armed=running,armed
            for action in (lambda:self.console.apply('hip',{'limits_deg':[-20,20]}),
                           lambda:self.console.zero('all'),lambda:self.console.read('hip'),
                           lambda:self.console.apply('ankle',{'pid':[1,0,0]})):
                with self.assertRaisesRegex(ValueError,'Stop and disarm'):
                    action()

    def test_integral_requires_read_motor_cap_and_cannot_exceed_it(self):
        with self.assertRaisesRegex(ValueError,'Read motor'):
            self.console.apply('hip',{'i':1})
        self.console.read('hip')
        self.console.apply('hip',{'p':0,'i':1,'d':0,'integral_limit_nm':.1})
        self.hip.running=True
        self.hip.setpoint=.3
        self.hip.feedback=self.hip_motor.feedback(2)
        for _ in range(1000):
            self.hip._integral_command(.02)
        self.assertAlmostEqual(self.hip.integral_nm,.1)
        self.hip.running=False
        self.assertEqual(self.hip._integral_command(.02),0)
        with self.assertRaises(ValueError):
            self.console.apply('hip',{'integral_limit_nm':2})

    def test_integral_does_not_wind_up_into_torque_saturation(self):
        self.console.read('hip')
        self.console.apply('hip',{'p':20,'i':10,'d':0,'integral_limit_nm':.5})
        self.hip.running=True
        self.hip.setpoint=.5
        self.hip.feedback=self.hip_motor.feedback(2)
        for _ in range(100):
            self.assertEqual(self.hip._integral_command(.02),0)

    def test_torque_limit_write_is_explicit_and_readback_is_used(self):
        self.console.apply('hip',{'torque_limit_value':.75})
        self.assertEqual(self.hip_motor.writes,[(21,.75)])
        self.assertEqual(self.hip.expected_torque_limit_value,.75)
        self.assertFalse(self.knee.writes)
        self.assertEqual(self.console.status()['rotary']['hip']['hardware']['torque_limit_value'],.75)

    def test_signed_zero_maps_commands_and_measurements_to_same_origin(self):
        self.console.zero('all')
        self.assertAlmostEqual(self.bridge.relative_degrees(.2),0)
        self.assertAlmostEqual(self.hip.relative_degrees(.2),0)
        self.assertEqual(self.ankle.reference_mm,[20,20,20])
        self.bridge.update(-5,True,hip_angle_deg=-5,extensions_mm=[19,20,21])
        self.assertAlmostEqual(self.bridge.goal,.2-math.radians(5))
        self.assertAlmostEqual(self.hip.goal,.2-math.radians(5))
        self.assertAlmostEqual(self.console.status()['rotary']['hip']['actual_angle_deg'],0)
        self.assertFalse(any(call=='arm' for call in self.knee.calls+self.hip_motor.calls))
        self.assertFalse(self.knee.writes or self.hip_motor.writes)

    def test_zero_preserves_existing_physical_command_limits(self):
        self.console.apply('hip',{'limits_deg':[-20,20]})
        self.ankle.soft_limits_mm=[[-2,8]]*3
        self.actuators.positions=[21,22,23]
        self.console.zero('all')
        self.assertAlmostEqual(self.hip.soft_limits_deg[1],20-math.degrees(.2))
        self.assertEqual(self.ankle.soft_limits_mm,[[-3,7],[-4,6],[-5,5]])
        self.assertTrue(self.hip.soft_limits_deg[0]>=-30)

    def test_zero_outside_existing_limits_cannot_expand_them(self):
        self.console.apply('hip',{'limits_deg':[-5,5]})
        before=self.console.profile()
        with self.assertRaisesRegex(ValueError,'outside existing command limits'):
            self.console.zero('hip')
        self.assertEqual(self.console.profile(),before)

    def test_zero_all_is_atomic_if_one_device_fails(self):
        before=self.console.profile()
        self.actuators.fail_init=True
        with self.assertRaises(RuntimeError):
            self.console.zero('all')
        self.assertEqual(self.console.profile(),before)
        self.assertFalse(self.path.exists())

    def test_nonstationary_zero_is_rejected(self):
        with patch.object(self.hip_motor,'enter_rest_mode',return_value={**self.hip_motor.feedback(0),'velocity_rad_s':1}):
            with self.assertRaisesRegex(RuntimeError,'stationary'):
                self.console.zero('hip')
        self.assertEqual(self.hip.reference_rad,0)

    def test_linear_zero_has_signed_readouts_and_translates_nominal_targets(self):
        self.actuators.positions=[23,24,25]
        self.console.zero('ankle')
        self.ankle.update([19,20,21],True)
        self.assertEqual(self.ankle.targets,[22,24,26])
        self.assertEqual(self.ankle.status()['relative_positions_mm'],[0,0,0])
        self.assertEqual(self.ankle.status()['limits_mm'],[[-3,36],[-4,35],[-5,34]])

    def test_all_three_linear_pid_and_separate_current_limits(self):
        self.console.apply('ankle',{'pid':[2,.2,.02],'current_limits_a':[1,1.2,1.4]})
        self.assertEqual(self.actuators.pid,[[2,.2,.02]]*3)
        self.assertEqual(self.console.readbacks['ankle']['pid_by_node'],[[2,.2,.02]]*3)
        self.assertEqual(self.actuators.current,[100,120,140])
        self.assertEqual(self.ankle.shared_pid,[2,.2,.02])
        self.assertEqual(self.ankle.expected_current_limits_a,[1,1.2,1.4])

    def test_invalid_linear_group_does_not_write_any_node(self):
        for data in ({'pid':[1,-1,0]}, {'current_limits_a':[1,2]},
                     {'pid':[1,0,0],'limits_mm':[[-20,40]]*3}):
            with self.assertRaises(ValueError):
                self.console.apply('ankle',data)
        self.assertFalse(self.actuators.writes)
        self.assertEqual(self.actuators.pid,[[1,.1,.01]]*3)

    def test_profile_keeps_inactive_motor_settings(self):
        self.console.apply('knee',{'p':31})
        hip_only=KneeBridge(None,hip=HipBridge(lambda:self.hip_motor))
        try:
            console=MotorConsole(hip_only,self.path)
            console.apply('hip',{'p':32})
            saved=json.loads(self.path.read_text())
            self.assertEqual(saved['rotary']['knee']['p'],31)
            self.assertEqual(saved['rotary']['hip']['p'],32)
        finally:
            hip_only.close()

    def test_invalid_profile_does_not_silently_arm_with_defaults(self):
        self.path.write_text('{"version":99}')
        with self.assertRaises(ValueError):
            MotorConsole(self.bridge,self.path)

    def test_external_power_is_per_device_and_expires(self):
        self.console.ingest_power('hip',48,1.5)
        self.console.ingest_power('a2',24,.25)
        state=self.console.status()
        self.assertEqual(state['rotary']['hip']['power']['electrical_power_w'],72)
        self.assertEqual(state['ankle']['nodes'][1]['power']['electrical_power_w'],6)
        self.assertIsNone(state['ankle']['nodes'][0]['power'])
        self.console.power['hip']['sampled_at']-=3
        self.assertIsNone(self.console.status()['rotary']['hip']['power'])
        with self.assertRaises(ValueError):
            self.console.ingest_power('hip',math.nan,1)


class FailureTests(unittest.TestCase):
    setUp=ConsoleTests.setUp
    tearDown=ConsoleTests.tearDown

    def test_failed_current_group_rolls_back_and_requires_verified_read(self):
        write=self.actuators.write
        failed=False
        def fault(node,index,sub,value,**kwargs):
            nonlocal failed
            if node==2 and not failed:
                failed=True
                self.actuators.current[node-1]=value
                raise RuntimeError('lost acknowledgement')
            return write(node,index,sub,value,**kwargs)
        with patch.object(self.actuators,'write',side_effect=fault):
            with self.assertRaises(RuntimeError):
                self.console.apply('ankle',{'pid':[2,0,0],'current_limits_a':[1,1,1]})
        self.assertEqual(self.actuators.current,[200,200,200])
        self.assertEqual(self.actuators.pid,[[1,.1,.01]]*3)
        self.assertIsNotNone(self.bridge.configuration_error)
        self.console.read('ankle')
        self.assertIsNone(self.bridge.configuration_error)

    def test_hardware_changed_but_profile_save_failed_blocks_arming(self):
        with patch.object(self.console,'_save',side_effect=OSError('disk full')):
            with self.assertRaises(OSError):
                self.console.apply('hip',{'torque_limit_value':.75})
        self.assertEqual(self.hip_motor.parameters[21],.75)
        self.assertIsNotNone(self.bridge.configuration_error)

    def test_software_reference_detects_machine_zero_changes(self):
        self.console.zero('hip')
        self.hip_motor.parameters[23]=.7
        self.bridge.update(20,True,hip_angle_deg=0,extensions_mm=[20,20,20])
        with self.assertRaisesRegex(RuntimeError,'machine zero changed'):
            self.bridge.arm()
        self.assertFalse(self.bridge.armed)

    def test_online_gain_scaling_mismatch_is_rejected_before_enable(self):
        self.hip_motor.parameters[29]=100
        self.bridge.update(20,True,hip_angle_deg=0,extensions_mm=[20,20,20])
        with self.assertRaisesRegex(RuntimeError,'scaling parameter 29'):
            self.bridge.arm()
        self.assertNotIn('arm',self.hip_motor.calls)

    def test_software_pid_apply_cannot_clear_unverified_hardware_failure(self):
        self.console.failed_targets.add('hip')
        self.bridge.configuration_error='Unverified motor write'
        self.console.apply('hip',{'p':10})
        self.assertEqual(self.bridge.configuration_error,'Unverified motor write')
        self.console.read('hip')
        self.assertIsNone(self.bridge.configuration_error)

    def test_configuration_waits_for_serial_worker_to_finish_disarming(self):
        self.ankle.idle.clear()
        with patch.object(self.ankle.idle,'wait',return_value=False):
            with self.assertRaisesRegex(ValueError,'still closing'):
                self.console.read('ankle')
        self.assertFalse(self.actuators.writes)
        self.ankle.idle.set()

    def test_integral_budget_is_conservative_when_firmware_units_differ(self):
        self.hip_motor.parameters[46]=.2
        self.console.read('hip')
        self.assertAlmostEqual(self.hip.torque_limit_nm,.2)
        with self.assertRaises(ValueError):
            self.console.apply('hip',{'i':1,'integral_limit_nm':.5})
        self.console.apply('hip',{'i':1,'integral_limit_nm':.1})


class APIConsoleTests(unittest.TestCase):
    def setUp(self):
        import threading
        from http.server import ThreadingHTTPServer
        from knee_bridge import make_handler
        ConsoleTests.setUp(self)
        self.server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(self.bridge,'test',0))
        self.port=self.server.server_address[1]
        self.server.RequestHandlerClass=make_handler(self.bridge,'test',self.port)
        self.worker=threading.Thread(target=self.server.serve_forever,daemon=True)
        self.worker.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.worker.join()
        ConsoleTests.tearDown(self)

    def post(self,route,data):
        from urllib.request import Request,urlopen
        base=f'http://127.0.0.1:{self.port}'
        request=Request(base+'/api/'+route,data=json.dumps(data).encode(),headers={
            'Content-Type':'application/json','Origin':base,'X-Bridge-Token':'test'})
        with urlopen(request) as response:
            return json.load(response)

    def test_configure_returns_live_settings_and_bad_values_are_atomic(self):
        from urllib.error import HTTPError
        state=self.post('configure',{'target':'hip','settings':{'p':11,'i':0,'d':2}})
        self.assertEqual(state['console']['rotary']['hip']['pid']['p'],11)
        with self.assertRaises(HTTPError) as error:
            self.post('configure',{'target':'hip','settings':{'p':12,'d':True}})
        self.assertEqual(error.exception.code,400)
        self.assertEqual(self.hip.gains['kp_pos'],11)

    def test_reference_and_power_routes_use_same_zero_and_separate_devices(self):
        state=self.post('reference',{'target':'all'})
        self.assertAlmostEqual(state['console']['rotary']['knee']['actual_angle_deg'],0)
        self.assertAlmostEqual(state['console']['rotary']['hip']['actual_angle_deg'],0)
        state=self.post('power',{'device':'a2','voltage_v':24,'bus_current_a':.5})
        self.assertEqual(state['console']['ankle']['nodes'][1]['power']['electrical_power_w'],12)
        self.assertIsNone(state['console']['ankle']['nodes'][0]['power'])

    def test_moving_simulator_cannot_read_or_zero_hardware(self):
        from urllib.error import HTTPError
        self.bridge.running=True
        for route in ('read-hardware','reference'):
            with self.assertRaises(HTTPError) as error:
                self.post(route,{'target':'hip'})
            self.assertEqual(error.exception.code,400)
        self.assertEqual(self.hip_motor.calls,[])


class SDOPort:
    def __init__(self):
        self.pending=bytearray()
        self.values={(n,0x2004,s):100000 for n in (1,2,3) for s in (1,2,3)}
        self.values[(1,0x1800,5)]=0
        self.sent=[]

    @property
    def in_waiting(self):
        return len(self.pending)

    def write(self,raw):
        self.sent.append(raw)
        frame=parse_frame(raw.decode().strip())
        if frame:
            can_id,data,_=frame
            node=can_id-0x600
            index,sub=struct.unpack('<HB',data[1:4])
            key=(node,index,sub)
            if data[0]==0x40:
                value=self.values[key]
                reply=bytes([0x43])+data[1:4]+value.to_bytes(4,'little')
            else:
                size={0x2F:1,0x2B:2,0x23:4}[data[0]]
                self.values[key]=int.from_bytes(data[4:4+size],'little')
                reply=bytes([0x60])+data[1:4]+bytes(4)
            self.pending.extend(f'b{0x580+node:03X}8{reply.hex()}\r'.encode())
        return len(raw)

    def read(self,size):
        data=bytes(self.pending[:size]);del self.pending[:size]
        return data

    def close(self):
        pass


class ProtocolTests(unittest.TestCase):
    def test_write_only_reporting_timer_does_not_upload_after_acknowledgement(self):
        class WriteOnlyTimerPort(SDOPort):
            def write(self, raw):
                frame=parse_frame(raw.decode().strip())
                if frame and frame[1][:4]==bytes.fromhex('40 00 18 05'):
                    self.sent.append(raw)
                    reply=bytes.fromhex('80 00 18 05 01 00 01 06')
                    self.pending.extend(f'b{frame[0]-0x80:03X}8{reply.hex()}\r'.encode())
                    return len(raw)
                return super().write(raw)
        port=WriteOnlyTimerPort()
        driver=SeenpinActuators(serial_factory=lambda *a,**k:port)
        driver.enable_monitoring()
        frames=[parse_frame(raw.decode().strip()) for raw in port.sent]
        self.assertEqual([(f[0],f[1]) for f in frames],
                         [(0x600+n,bytes.fromhex('2F 00 18 05 64 00 00 00')) for n in (1,2,3)])

    def test_reporting_timer_still_requires_acknowledgement_and_reports_abort_code(self):
        class RejectedTimerPort(SDOPort):
            def write(self,raw):
                self.sent.append(raw)
                frame=parse_frame(raw.decode().strip())
                reply=bytes.fromhex('80 00 18 05 02 00 01 06')
                self.pending.extend(f'b{frame[0]-0x80:03X}8{reply.hex()}\r'.encode())
                return len(raw)
        driver=SeenpinActuators(serial_factory=lambda *a,**k:RejectedTimerPort())
        with self.assertRaisesRegex(RuntimeError,'write aborted 1800:05.*0x06010002'):
            driver.enable_monitoring()

    def test_control_writes_cannot_skip_readback(self):
        port=SDOPort()
        driver=SeenpinActuators(serial_factory=lambda *a,**k:port)
        for index,sub in ((0x2004,1),(0x2009,3),(0x607D,1)):
            with self.assertRaisesRegex(ValueError,'require independent readback'):
                driver.write(1,index,sub,1,verify_readback=False)
        self.assertEqual(port.sent,[])

    def test_sdo_write_packet_and_all_node_pid_scaling(self):
        port=SDOPort()
        driver=SeenpinActuators(serial_factory=lambda *a,**k:port)
        result=driver.apply_position_pid([1.25,.01234,2])
        self.assertEqual(result,[[1.25,.01234,2]]*3)
        writes=[parse_frame(raw.decode().strip()) for raw in port.sent if b'2340' in raw]
        self.assertEqual(port.values[(2,0x2004,2)],1234)
        self.assertTrue(any(parse_frame(raw.decode().strip())[1]==bytes.fromhex('23 04 20 01 48 E8 01 00') for raw in port.sent))
        with self.assertRaises(ValueError):
            driver.write(1,0x2009,1,7,size=1)

    def test_pid_partial_failure_rolls_back_every_attempted_node(self):
        port=SDOPort();driver=SeenpinActuators(serial_factory=lambda *a,**k:port)
        before=copy.deepcopy(port.values)
        write=driver.write
        failed=False
        def fault(node,index,sub,value,**kwargs):
            nonlocal failed
            if node==2 and sub==2 and not failed:
                failed=True
                # Simulate a write that took effect before its acknowledgement was lost.
                port.values[(node,index,sub)]=value
                raise RuntimeError('lost acknowledgement')
            return write(node,index,sub,value,**kwargs)
        with patch.object(driver,'write',side_effect=fault):
            with self.assertRaisesRegex(RuntimeError,'rollback verified'):
                driver.apply_position_pid([2,3,4])
        self.assertEqual(port.values,before)

    def test_final_group_read_failure_also_rolls_back(self):
        port=SDOPort();driver=SeenpinActuators(serial_factory=lambda *a,**k:port)
        before=copy.deepcopy(port.values)
        with patch.object(driver,'read_position_pid',side_effect=RuntimeError('final read lost')):
            with self.assertRaisesRegex(RuntimeError,'rollback verified'):
                driver.apply_position_pid([2,3,4])
        self.assertEqual(port.values,before)

    def test_pdo1_decodes_force_current_and_mechanical_power(self):
        port=SDOPort();driver=SeenpinActuators(serial_factory=lambda *a,**k:port)
        data=struct.pack('<Hhhhh',0,2150,30,100,125)+bytes(6)
        port.pending.extend(f'b182A{data.hex()}\r'.encode())
        driver.listen(.005)
        t=driver.telemetry[2]
        self.assertEqual(t['position_mm'],21.5)
        self.assertEqual(t['force_n'],100)
        self.assertEqual(t['motor_current_a'],1.25)
        self.assertAlmostEqual(t['mechanical_power_w'],.3)

    def test_rotary_write_requires_acknowledgement_and_readback(self):
        reply=can.Message(arbitration_id=0x602,is_extended_id=False,
            data=bytes([2,21])+struct.pack('<f',.75)+bytes([0,255]))
        with patch('motorevo_driver.can.Bus') as factory:
            factory.return_value.recv.side_effect=[reply,reply]
            motor=MotorevoMotor(motor_id=2)
            self.assertEqual(motor.write_parameter(21,.75),.75)
            sent=[c.args[0] for c in factory.return_value.send.call_args_list]
            self.assertEqual(bytes(sent[0].data),bytes.fromhex('67 15 00 00 40 3F 15 76'))
            self.assertEqual(sent[1].data[6],0x04)
            with self.assertRaises(ValueError):
                motor.write_parameter(22,1)


if __name__=='__main__':
    unittest.main()
