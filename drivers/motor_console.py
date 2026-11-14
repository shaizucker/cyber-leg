"""Online configuration, signed references and honest device capabilities.

No hardware is opened until an explicit read/apply/zero action. Configuration
files store bridge settings; they never imply a device flash-save command.
"""
import copy
import json
import math
import os
import time
from pathlib import Path
from knee_safety import verify_command_scaling


def number(value, low, high, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name} must be finite, {low:g}..{high:g}')
    return float(value)


def limits(value, name):
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f'{name} requires minimum and maximum')
    lo, hi = [number(v, -360, 360, name) for v in value]
    if not lo <= 0 <= hi or not lo < hi:
        raise ValueError(f'{name} must include zero and have minimum < maximum')
    return [lo, hi]


class MotorConsole:
    def __init__(self, bridge, path=None):
        self.bridge = bridge
        self.path = Path(path) if path else None
        self.readbacks = {}
        self.power = {}
        self._saved = dict(version=1,rotary={},ankle=None)
        self.failed_targets = set()
        if bridge.ankle:
            bridge.ankle.enable_monitoring = True
        self.message = 'Read hardware while stopped and disarmed before changing device limits.'
        if self.path and self.path.exists():
            profile = json.loads(self.path.read_text(encoding='utf-8'))
            self._restore(profile)
            self._saved = copy.deepcopy(profile)
        bridge.console = self
        for joint in self.joints().values():
            joint.verify_online_scaling = True

    def joints(self):
        return {name: joint for name, joint in (('knee', self.bridge if self.bridge.knee_enabled else None),
                                                ('hip', self.bridge.hip)) if joint is not None}

    def stopped(self):
        ankle = self.bridge.ankle
        if (self.bridge.armed or self.bridge.running or
                (ankle and (ankle.status()['armed'] or ankle.status()['arming']))):
            raise ValueError('Stop and disarm all outputs before changing hardware, limits or zero references')
        if ankle and not ankle.idle.wait(2):
            raise ValueError('Ankle transport is still closing; retry after disarming completes')

    def profile(self):
        rotary = {}
        for name, joint in self.joints().items():
            rotary[name] = dict(p=joint.gains['kp_pos'], i=joint.ki_pos, d=joint.gains['kd_pos'],
                integral_limit_nm=joint.integral_limit_nm, limits_deg=list(joint.soft_limits_deg),
                reference_rad=joint.reference_rad, reference_binding=joint.reference_binding,
                torque_limit_value=joint.expected_torque_limit_value)
        ankle = self.bridge.ankle
        return dict(version=1, rotary=rotary, ankle=None if ankle is None else dict(
            reference_mm=list(ankle.reference_mm), limits_mm=copy.deepcopy(ankle.soft_limits_mm),
            pid=copy.deepcopy(ankle.shared_pid), current_limits_a=copy.deepcopy(ankle.expected_current_limits_a)))

    def _restore(self, profile):
        if not isinstance(profile, dict) or profile.get('version') != 1:
            raise ValueError('Unsupported motor configuration file version')
        pending = []
        for name, joint in self.joints().items():
            data = profile.get('rotary', {}).get(name)
            if data is None:
                continue
            gains = dict(kp_pos=number(data['p'],0,250,'P'), kd_pos=number(data['d'],0,50,'D'))
            i = number(data['i'],0,50,'I')
            integral = number(data['integral_limit_nm'],0,50,'Integral cap')
            reference = number(data['reference_rad'],-12.5,12.5,'Reference')
            bound = data.get('reference_binding')
            if bound is not None:
                bound = number(bound,-1e9,1e9,'Machine zero binding')
            expected = data.get('torque_limit_value')
            if expected is not None:
                expected = number(expected,0.001,50,'Motor torque limit')
            soft = limits(data['limits_deg'],'Angle limits')
            self._check_rotary_limits(joint, soft, reference)
            pending.append((joint,gains,i,integral,reference,bound,soft,expected))
        ankle_data = profile.get('ankle') if self.bridge.ankle else None
        if ankle_data is not None:
            ref = ankle_data.get('reference_mm')
            if not isinstance(ref,list) or len(ref) != 3:
                raise ValueError('Three ankle zero references are required')
            ref = [number(x,0,50,'Ankle zero') for x in ref]
            soft = ankle_data.get('limits_mm')
            if soft is not None:
                if not isinstance(soft,list) or len(soft) != 3:
                    raise ValueError('Three ankle limit pairs are required')
                soft = [limits(pair,'Ankle limits') for pair in soft]
                if any(not 0 <= r+lo < r+hi <= 50 for r,(lo,hi) in zip(ref,soft)):
                    raise ValueError('Ankle signed limits exceed absolute 0..50 mm envelope')
            pid = ankle_data.get('pid')
            if pid is not None:
                if not isinstance(pid,list) or len(pid) != 3:
                    raise ValueError('Ankle PID requires three values')
                pid = [number(x,0,42949.67295,'Ankle PID') for x in pid]
        ankle_values = (ref,soft,pid) if ankle_data is not None else None
        expected_currents = ankle_data.get('current_limits_a') if ankle_data is not None else None
        if expected_currents is not None:
            if not isinstance(expected_currents,list) or len(expected_currents)!=3:
                raise ValueError('Three actuator current limits required')
            expected_currents = [number(x,0.01,655.35,'Peak current') for x in expected_currents]
        for joint,gains,i,integral,reference,bound,soft,expected in pending:
            joint.gains, joint.ki_pos, joint.integral_limit_nm = gains,i,integral
            joint.reference_rad, joint.reference_binding, joint.soft_limits_deg = reference,bound,soft
            joint.integral_nm = 0.0
            joint.expected_torque_limit_value = expected
        if ankle_data is not None:
            ankle = self.bridge.ankle
            ankle.reference_mm, ankle.soft_limits_mm, ankle.shared_pid = ankle_values
            ankle.expected_current_limits_a = expected_currents

    def _save(self):
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temp = self.path.with_suffix('.tmp')
            candidate = copy.deepcopy(self._saved)
            current = self.profile()
            candidate['rotary'].update(current['rotary'])
            if self.bridge.ankle:
                candidate['ankle'] = current['ankle']
            temp.write_text(json.dumps(candidate,indent=2,allow_nan=False),encoding='utf-8')
            os.replace(temp,self.path)
            self._saved = candidate

    @staticmethod
    def _check_rotary_limits(joint, soft, reference):
        positions = [joint.sign*joint.angle_scale*math.radians(a-joint.joint_min_deg)+joint.offset+reference for a in soft]
        # The legacy 105-degree knee endpoint saturates at 1.83 rad.
        tolerance = math.radians(0.16) if joint.joint_min_deg == 0 and reference == 0 else 1e-8
        if any(not joint.theta_min-tolerance <= x <= joint.theta_max+tolerance for x in positions):
            raise ValueError('Signed limits exceed the verified motor encoding/protection range')
        if joint.joint_min_deg < 0 and not -30 <= soft[0] < soft[1] <= 30:
            raise ValueError('Hip limits must remain within -30..30 degrees')

    def _read_joint(self, name):
        joint = self.joints().get(name)
        if joint is None:
            raise ValueError(f'{name} output is disabled')
        motor = joint.open_motor()
        try:
            config = joint.read_motor_configuration(motor)
            verify_command_scaling(motor)
            t = motor.enter_rest_mode()
            joint.feedback, joint.feedback_at = t,time.monotonic()
            if not t or t['status'] != 0 or t['fault']:
                raise RuntimeError('Motor must confirm fault-free Rest for configuration: '
                                   + joint.feedback_description(t))
            torque = motor.read_parameter(21)
            constant = motor.read_parameter(46)
            number(torque,0.001,50,'Reported torque limit')
            number(constant,0.000001,1000,'Reported torque constant')
            joint.feedback, joint.feedback_at = t,time.monotonic()
            budget = min(torque,torque*constant)
            if joint.expected_torque_limit_value is not None:
                budget = min(budget,joint.expected_torque_limit_value,joint.expected_torque_limit_value*constant)
            joint.torque_limit_nm = budget
            self.readbacks[name] = dict(configuration=config, torque_limit_value=torque,
                torque_constant=constant, sampled_at=time.monotonic())
            return motor, config, t
        except Exception:
            motor.close()
            raise

    def _ankle_driver(self):
        ankle = self.bridge.ankle
        if ankle is None:
            raise ValueError('Ankle output is disabled')
        driver = ankle.factory()
        try:
            driver.initialize()
            ankle.positions,ankle.limits = list(driver.positions),copy.deepcopy(driver.limits)
            return driver
        except Exception:
            driver.close()
            raise

    def read(self, target):
        with self.bridge.lock:
            self.stopped()
            if target in ('knee','hip'):
                motor,_,_ = self._read_joint(target)
                motor.close()
            elif target == 'ankle':
                driver = self._ankle_driver()
                try:
                    pid = driver.read_position_pid()
                    currents = [driver.read(n,0x2009,3)/100 for n in (1,2,3)]
                    self.readbacks['ankle'] = dict(pid_by_node=pid, current_limits_a=currents,
                        sampled_at=time.monotonic())
                finally:
                    driver.close()
            else:
                raise ValueError('Unknown output')
            if target in self.failed_targets:
                coherent = target != 'ankle' or all(row == self.readbacks['ankle']['pid_by_node'][0] for row in self.readbacks['ankle']['pid_by_node'])
                if coherent:
                    self.failed_targets.discard(target)
                    if not self.failed_targets:
                        self.bridge.configuration_error = None
            self.message = f'{target}: hardware read complete; no output was enabled'
            return self.status()

    def apply(self, target, data):
        with self.bridge.lock:
            if not isinstance(data,dict):
                raise ValueError('Settings must be an object')
            before = self.profile()
            hardware_changed = False
            if target in self.joints():
                joint = self.joints()[target]
                if set(data)-{'p','i','d','integral_limit_nm','limits_deg','torque_limit_value'}:
                    raise ValueError('Unknown rotary setting')
                gains = dict(kp_pos=number(data.get('p',joint.gains['kp_pos']),0,250,'P'),
                             kd_pos=number(data.get('d',joint.gains['kd_pos']),0,50,'D'))
                i = number(data.get('i',joint.ki_pos),0,50,'I')
                cap = number(data.get('integral_limit_nm',joint.integral_limit_nm),0,50,'Integral cap')
                soft = limits(data.get('limits_deg',joint.soft_limits_deg),'Angle limits')
                self._check_rotary_limits(joint,soft,joint.reference_rad)
                if 'limits_deg' in data or 'torque_limit_value' in data:
                    self.stopped()
                if i and (joint.torque_limit_nm is None or cap > joint.torque_limit_nm):
                    raise ValueError('Read motor torque limit first; integral cap must not exceed it')
                if 'torque_limit_value' in data:
                    limit = number(data['torque_limit_value'],0.001,50,'Torque limit value')
                    if i and cap > limit:
                        raise ValueError('Integral cap exceeds requested motor limit')
                    motor,_,_ = self._read_joint(target)
                    old = joint.torque_limit_nm
                    try:
                        motor.write_parameter(21,limit)
                        hardware_changed = True
                        joint.torque_limit_nm = min(limit,limit*self.readbacks[target]['torque_constant'])
                        joint.expected_torque_limit_value = limit
                        self.readbacks[target]['torque_limit_value'] = limit
                    except Exception:
                        try:
                            motor.write_parameter(21,old)
                        except Exception:
                            self.failed_targets.add(target)
                            self.bridge.configuration_error = 'Torque-limit rollback unverified; restart and read hardware before arming'
                        raise
                    finally:
                        motor.close()
                joint.gains,joint.ki_pos,joint.integral_limit_nm,joint.soft_limits_deg = gains,i,cap,soft
                joint.integral_nm = 0.0
            elif target == 'ankle':
                self.stopped()
                if set(data)-{'pid','limits_mm','current_limits_a'}:
                    raise ValueError('Unknown ankle setting')
                ankle = self.bridge.ankle
                if ankle is None:
                    raise ValueError('Ankle output is disabled')
                candidate = self.profile()
                if 'limits_mm' in data:
                    candidate['ankle']['limits_mm'] = data['limits_mm']
                if 'pid' in data:
                    candidate['ankle']['pid'] = data['pid']
                # Validate the full candidate before sending any hardware writes.
                self._restore(candidate)
                self._restore(before)
                currents = data.get('current_limits_a')
                if currents is not None:
                    if not isinstance(currents,list) or len(currents) != 3:
                        raise ValueError('Three independent current limits are required')
                    currents = [number(x,0.01,655.35,'Peak current') for x in currents]
                if currents is not None:
                    candidate['ankle']['current_limits_a'] = currents
                driver = self._ankle_driver()
                attempted = []
                try:
                    if currents is not None:
                        old_currents = [driver.read(n,0x2009,3) for n in (1,2,3)]
                    if currents is not None:
                        for node,old,value in zip((1,2,3),old_currents,currents):
                            attempted.append((node,old))
                            driver.write(node,0x2009,3,round(value*100),size=2)
                    verified_pid = driver.apply_position_pid(candidate['ankle']['pid']) if 'pid' in data else None
                    self._restore(candidate)
                    cached = dict(self.readbacks.get('ankle',{}))
                    if verified_pid is not None:
                        cached['pid_by_node'] = verified_pid
                    if currents is not None:
                        cached['current_limits_a'] = list(currents)
                    cached['sampled_at'] = time.monotonic()
                    self.readbacks['ankle'] = cached
                    hardware_changed = bool(currents is not None or 'pid' in data)
                except Exception:
                    for node,old in reversed(attempted):
                        try:
                            driver.write(node,0x2009,3,old,size=2)
                        except Exception:
                            self.bridge.configuration_error = 'Actuator current-limit rollback unverified; read hardware before arming'
                    self.failed_targets.add(target)
                    self.bridge.configuration_error = self.bridge.configuration_error or 'Ankle configuration failed; verify group settings before arming'
                    raise
                finally:
                    driver.close()
            else:
                raise ValueError('Output is disabled or unknown')
            try:
                self._save()
            except Exception:
                self._restore(before)
                if hardware_changed:
                    self.failed_targets.add(target)
                    self.bridge.configuration_error = 'Hardware changed but profile save failed; read/verify before arming'
                raise
            self.message = f"{target}: settings applied; "+('bridge profile saved' if self.path else 'session only')+'; device flash was not written'
            return self.status()

    def zero(self, target):
        with self.bridge.lock:
            self.stopped()
            if target not in ('all','knee','hip','ankle'):
                raise ValueError('Unknown zero target')
            names = list(self.joints()) if target == 'all' else [target] if target in ('knee','hip') else []
            pending = []
            for name in names:
                joint = self.joints().get(name)
                if joint is None:
                    raise ValueError('Output is disabled')
                motor,config,t = self._read_joint(name)
                try:
                    if abs(t['velocity_rad_s']) > 0.05:
                        raise RuntimeError('Motor must be stationary before capturing zero')
                    old_position = joint.relative_degrees(t['position_rad'])
                    if not joint.soft_limits_deg[0]-1e-6 <= old_position <= joint.soft_limits_deg[1]+1e-6:
                        raise ValueError('Current position is outside existing command limits; zero cannot widen those limits')
                    physical_limits = [joint.mapped_target(a-joint.joint_min_deg) for a in joint.soft_limits_deg]
                    pending.append((name,joint,t['position_rad'],config['machine_offset_rad'],physical_limits))
                finally:
                    motor.close()
            positions = None
            physical_ankle_limits = None
            if target == 'ankle' or (target == 'all' and self.bridge.ankle):
                driver = self._ankle_driver()
                try:
                    for node in (1,2,3):
                        if abs(driver.read(node,0x2010,3,signed=True)/10)>0.1:
                            raise RuntimeError('All ankle actuators must be stationary before capturing zero')
                    positions = list(driver.positions)
                    ankle = self.bridge.ankle
                    physical_ankle_limits = ([[origin+lo,origin+hi] for origin,(lo,hi) in zip(ankle.reference_mm,ankle.soft_limits_mm)]
                        if ankle.soft_limits_mm is not None else copy.deepcopy(driver.limits))
                    if any(not lo <= position <= hi for position,(lo,hi) in zip(positions,physical_ankle_limits)):
                        raise ValueError('Actuator position is outside existing command limits; zero cannot widen those limits')
                finally:
                    driver.close()
            before = self.profile()
            for _,joint,position,binding,physical_limits in pending:
                base = joint.offset-joint.sign*joint.angle_scale*math.radians(joint.joint_min_deg)
                joint.reference_rad = position-base
                joint.reference_binding = binding
                low,high = sorted(joint.relative_degrees(p) for p in physical_limits)
                if joint.joint_min_deg < 0:
                    low,high = max(-30,low),min(30,high)
                joint.soft_limits_deg = [low,high]
                joint.goal = joint.setpoint = position
                joint.integral_nm = 0.0
            if positions is not None:
                ankle = self.bridge.ankle
                ankle.reference_mm = positions
                ankle.soft_limits_mm = [[lo-p,hi-p] for (lo,hi),p in zip(physical_ankle_limits,positions)]
                ankle.targets = ankle.command = list(positions)
            try:
                self._save()
            except Exception:
                self._restore(before)
                raise
            self.message = 'Current measured pose captured as signed zero; no homing or motor-zero command was sent'
            return self.status()

    def ingest_power(self, device, voltage, current):
        allowed = set(self.joints()) | ({'a1','a2','a3'} if self.bridge.ankle else set())
        if device not in allowed:
            raise ValueError('Power sensor target is disabled or unknown')
        v,a = number(voltage,0,150,'Supply voltage'),number(current,-100,100,'DC bus current')
        with self.bridge.lock:
            self.power[device] = dict(voltage_v=v, bus_current_a=a, electrical_power_w=v*a,
                                      sampled_at=time.monotonic(),source='external DC sensor')

    def status(self):
        with self.bridge.lock:
            return self._status()

    def _status(self):
        rotary = {}
        now = time.monotonic()
        for name,joint in self.joints().items():
            feedback = joint.feedback
            sampled = joint.feedback_at
            sensor = self.power.get(name)
            rotary[name] = dict(pid=dict(p=joint.gains['kp_pos'],i=joint.ki_pos,d=joint.gains['kd_pos']),
                integral_limit_nm=joint.integral_limit_nm, integral_nm=joint.integral_nm,
                limits_deg=list(joint.soft_limits_deg), hardware_limits_deg=[math.degrees(joint.can_theta_min),math.degrees(joint.can_theta_max)],
                reference_motor_rad=joint.reference_rad+joint.offset-joint.sign*joint.angle_scale*math.radians(joint.joint_min_deg),
                reference_saved=joint.reference_binding is not None,
                actual_angle_deg=None if not feedback else joint.relative_degrees(feedback['position_rad']),
                command_angle_deg=None if joint.setpoint is None else joint.relative_degrees(joint.setpoint),
                torque_nm=None if not feedback else feedback['torque_nm'],
                mechanical_power_w=None if not feedback else feedback['torque_nm']*feedback['velocity_rad_s'],
                sampled_at=sampled, stale=not sampled or now-sampled>1,
                power=sensor if sensor and now-sensor['sampled_at']<=2 else None,
                hardware=self.readbacks.get(name), expected_torque_limit_value=joint.expected_torque_limit_value,
                capabilities=dict(position_i='bridge integral via torque feedforward; P/D in motor',
                    voltage='external DC sensor required',electrical_power='external DC voltage and bus-current sensor required'))
        ankle = self.bridge.ankle
        ankle_status = None
        if ankle:
            node_data=[]
            for i in range(3):
                t=dict(ankle.telemetry[i])
                sensor=self.power.get(f'a{i+1}')
                node_data.append(dict(node=i+1, position_mm=None if ankle.positions is None else ankle.positions[i]-ankle.reference_mm[i],
                    force_n=t.get('force_n'), motor_current_a=t.get('motor_current_a'),
                    mechanical_power_w=t.get('mechanical_power_w'), torque_nm=None,
                    stale=not t.get('sampled_at') or now-t['sampled_at']>1,
                    power=sensor if sensor and now-sensor['sampled_at']<=2 else None))
            ankle_status=dict(pid=ankle.shared_pid, limits_mm=copy.deepcopy(ankle.soft_limits_mm) if ankle.soft_limits_mm is not None else (None if ankle.limits is None else [[lo-ref,hi-ref] for (lo,hi),ref in zip(ankle.limits,ankle.reference_mm)]),
                reference_mm=list(ankle.reference_mm), current_limits_a=ankle.expected_current_limits_a, nodes=node_data,hardware=self.readbacks.get('ankle'),
                capabilities=dict(torque='force in N is available; no documented motor torque constant',
                    electrical_power='DC supply sensors required; motor phase current is not DC bus current'))
        return dict(rotary=rotary,ankle=ankle_status,message=self.message,
                    profile_saved=bool(self.path and self.path.exists()),configuration_error=self.bridge.configuration_error)
