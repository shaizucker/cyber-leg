"""Seenpin SLCAN CAN-FD transport matching seenpin_extend_return.py.

One worker owns this transport. refresh is called during SDO waits so all
three position commands keep streaming while telemetry is being read.
"""
import math
from collections import deque
import struct
import time

SDO_TIMEOUT = 0.25
SDO_ATTEMPTS = 3
PDO_INTERRUPTED = 0x8002

FD_LENGTHS = (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64)


def parse_frame(line):
    if not line or line[0] not in 'tTdDbB':
        return None
    kind = line[0]
    width = 8 if kind.isupper() else 3
    try:
        can_id = int(line[1:1 + width], 16)
        dlc = int(line[1 + width], 16)
        if kind in 'tT' and dlc > 8:
            return None
        size = dlc if kind in 'tT' else FD_LENGTHS[dlc]
        start = width + 2
        data = bytes.fromhex(line[start:start + size * 2])
        if len(data) != size or can_id > (0x1FFFFFFF if width == 8 else 0x7FF):
            return None
        return can_id, data, kind
    except (ValueError, IndexError):
        return None


def position_payload(target_mm, duration_ms=100):
    if not math.isfinite(target_mm) or not 1 <= duration_ms <= 60000:
        raise ValueError('Invalid Seenpin target or duration')
    target = round(target_mm * 100)
    if not -32768 <= target <= 32767:
        raise ValueError('Seenpin target exceeds signed 16-bit position range')
    return struct.pack('<BhHH', 1, target, duration_ms, 0x000F)


class SeenpinActuators:
    nodes = (1, 2, 3)

    def __init__(self, port='COM7', log=None, serial_factory=None):
        if serial_factory is None:
            import serial
            serial_factory = serial.Serial
        self.log = log or (lambda message: None)
        self.port = serial_factory(port, 115200, timeout=0.002, write_timeout=0.1)
        self.buffer = bytearray()
        self.inbox = []
        self.refresh = None
        self.limits = []
        self.positions = []
        self.startup_faults = {}
        self.last_rx = {}
        self.last_pdo = {}
        self.max_pdo_gap = {}
        self.recent_link = deque(maxlen=24)
        self.adapter_events = deque(maxlen=8)
        self.telemetry = {node: {} for node in self.nodes}

    def send(self, command):
        raw = (command + '\r').encode('ascii')
        if self.port.write(raw) != len(raw):
            raise RuntimeError('Incomplete Seenpin serial write')

    def send_frame(self, can_id, data):
        self.send(f'b{can_id:03X}{len(data):X}{data.hex().upper()}')
        now = time.monotonic()
        if 0x201 <= can_id <= 0x203 and len(data) == 7 and data[0] == 1:
            node = can_id - 0x200
            previous = self.last_pdo.get(node)
            if previous is not None:
                self.max_pdo_gap[node] = max(self.max_pdo_gap.get(node, 0), now - previous)
            self.last_pdo[node] = now
        elif 0x601 <= can_id <= 0x603:
            self.recent_link.append((now, f'TX 0x{can_id:03X} {data.hex(" ").upper()}'))

    def listen(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self.refresh:
                self.refresh()
            self.buffer.extend(self.port.read(min(self.port.in_waiting or 1, 4096)))
            while True:
                delimiters = [i for i in (self.buffer.find(b'\r'), self.buffer.find(b'\a')) if i >= 0]
                if not delimiters:
                    break
                i = min(delimiters)
                raw, sep = bytes(self.buffer[:i]), self.buffer[i]
                del self.buffer[:i + 1]
                if sep == 7:
                    raise RuntimeError('Seenpin adapter rejected a command (BEL)')
                line = raw.decode('ascii', errors='replace').strip()
                if line.startswith('E') and len(line) == 9:
                    now = time.monotonic()
                    self.adapter_events.append((now, line))
                    self.log(f'ANKLE adapter report: {line}')
                frame = parse_frame(line)
                if frame:
                    now = time.monotonic()
                    self.last_rx[frame[0]] = now
                    if 0x181 <= frame[0] <= 0x183 and len(frame[1]) == 16 and frame[2] == 'b':
                        fault, position, speed, force, current = struct.unpack('<Hhhhh', frame[1][:10])
                        self.telemetry[frame[0]-0x180].update(
                            position_mm=position/100, speed_mm_s=speed/10,
                            force_n=force, motor_current_a=current/100,
                            mechanical_power_w=force*speed/10000, sampled_at=now,
                            pdo_fault=fault)
                    self.recent_link.append((now, f'RX {line}'))
                    self.inbox.append(frame)
                    self.inbox = self.inbox[-256:]
            if len(self.buffer) > 16384:
                raise RuntimeError('Invalid Seenpin serial stream')

    def log_link_diagnostics(self, failed_node):
        # Called only once all retries have failed; existing queued logging keeps
        # file/terminal I/O off the control worker so shutdown is not delayed.
        now = time.monotonic()
        def age(stamp):
            return 'never' if stamp is None else f'{max(0, now - stamp):.3f}s ago'
        self.log(f'ANKLE LINK: node {failed_node} read failed; diagnostics only, no automatic recovery')
        for node in self.nodes:
            self.log(f'ANKLE LINK node {node}: heartbeat {age(self.last_rx.get(0x700 + node))}; '
                     f'SDO reply {age(self.last_rx.get(0x580 + node))}; '
                     f'last position TX {age(self.last_pdo.get(node))}; '
                     f'max position TX gap {self.max_pdo_gap.get(node, 0) * 1000:.1f} ms')
        events = ', '.join(f'{line} ({age(stamp)})' for stamp, line in self.adapter_events)
        self.log(f'ANKLE LINK adapter reports: {events or "none received"}')
        for stamp, line in list(self.recent_link)[-8:]:
            self.log(f'ANKLE LINK recent {age(stamp)}: {line}')

    def read(self, node, index, sub, signed=False):
        key = struct.pack('<HB', index, sub)
        seen_ids = set()
        for attempt in range(1, SDO_ATTEMPTS + 1):
            self.inbox.clear()
            self.send_frame(0x600 + node, b'\x40' + key + bytes(4))
            deadline = time.monotonic() + SDO_TIMEOUT
            while time.monotonic() < deadline:
                # listen also refreshes the three movement commands, checks
                # browser freshness and propagates Disarm during every retry.
                self.listen(0.003)
                for can_id, data, kind in self.inbox:
                    seen_ids.add(can_id)
                    if not kind.islower() or can_id != 0x580 + node or len(data) < 4 or data[1:4] != key:
                        continue
                    if data[0] == 0x80:
                        code = f' (code 0x{int.from_bytes(data[4:8], "little"):08X})' if len(data) >= 8 else ''
                        raise RuntimeError(f'Ankle node {node}: SDO abort {index:04X}:{sub:02X}{code}')
                    size = {0x4F: 1, 0x4B: 2, 0x47: 3, 0x43: 4}.get(data[0])
                    if size is not None and len(data) >= 4 + size:
                        if attempt > 1:
                            self.log(f'ANKLE node {node}: SDO {index:04X}:{sub:02X} recovered on attempt {attempt}')
                        return int.from_bytes(data[4:4 + size], 'little', signed=signed)
                self.inbox.clear()
            if attempt < SDO_ATTEMPTS:
                self.log(f'ANKLE node {node}: no SDO {index:04X}:{sub:02X} reply; retry {attempt + 1}/{SDO_ATTEMPTS}')
        self.log_link_diagnostics(node)
        received = ', '.join(f'0x{x:03X}' for x in sorted(seen_ids)) or 'none'
        raise RuntimeError(f'Ankle node {node}: feedback timeout {index:04X}:{sub:02X} '
                           f'after {SDO_ATTEMPTS} attempts; RX IDs: {received}. '
                           'Check node power, CAN connection and node ID')

    def initialize(self):
        time.sleep(1.5)
        self.port.reset_input_buffer()
        for command in ('C', 'V', 'S8', 'Y5', 'M0', 'A0', 'O'):
            self.send(command)
            self.listen(0.1)
        # RH02/CANable 2.5: report adapter CAN errors without changing frame
        # format, bitrate, retry policy or any actuator parameter.
        self.send('ME')
        self.listen(0.01)
        for node in self.nodes:
            fault = self.read(node, 0x2010, 5)
            position = self.read(node, 0x2010, 2, signed=True) / 100
            low = self.read(node, 0x607D, 1, signed=True) / 100
            high = self.read(node, 0x607D, 2, signed=True) / 100
            if fault == PDO_INTERRUPTED:
                self.startup_faults[node] = fault
                self.log(f'ANKLE node {node}: 0x8002 CAN FD PDO interrupt; recovery pending on explicit Arm')
            elif fault:
                raise RuntimeError(f'Ankle node {node}: fault 0x{fault:04X}; startup blocked')
            if not -327.68 <= low < high <= 327.67:
                raise RuntimeError(f'Ankle node {node}: invalid drive limits {low:.2f}..{high:.2f} mm')
            if not low <= position <= high:
                raise RuntimeError(f'Ankle node {node}: position {position:.2f} mm outside {low:.2f}..{high:.2f} mm')
            self.positions.append(position)
            self.limits.append([low, high])
            self.log(f'ANKLE node {node}: {position:.2f} mm, limits {low:.2f}..{high:.2f} mm')

    def write(self, node, index, sub, value, size=4, signed=False, verify_readback=True):
        """Acknowledged SDO download; control settings require separate readback.

        Only the optional PDO reporting timer may use acknowledgement alone:
        deployed firmware accepts its download but aborts an upload/read.
        """
        if node not in self.nodes or (index, sub) not in {
                (0x2004, 1), (0x2004, 2), (0x2004, 3),
                (0x607D, 1), (0x607D, 2), (0x2009, 3), (0x1800, 5)}:
            raise ValueError('Unsupported actuator parameter write')
        if size not in (1, 2, 4):
            raise ValueError('Unsupported SDO size')
        if not verify_readback and (index, sub) != (0x1800, 5):
            raise ValueError('Control parameter writes require independent readback')
        key = struct.pack('<HB', index, sub)
        raw = int(value).to_bytes(size, 'little', signed=signed)
        self.inbox.clear()
        self.send_frame(0x600+node, bytes([{1:0x2F, 2:0x2B, 4:0x23}[size]])+key+raw.ljust(4, b'\0'))
        deadline = time.monotonic()+SDO_TIMEOUT
        while time.monotonic() < deadline:
            self.listen(0.003)
            frames, self.inbox = self.inbox, []
            for can_id, data, kind in frames:
                if can_id != 0x580+node or not kind.islower() or len(data) < 4 or data[1:4] != key:
                    continue
                if data[0] == 0x80:
                    code = f' (code 0x{int.from_bytes(data[4:8], "little"):08X})' if len(data) >= 8 else ''
                    raise RuntimeError(f'Ankle node {node}: write aborted {index:04X}:{sub:02X}{code}')
                if data[0] == 0x60:
                    if not verify_readback:
                        return value
                    readback = self.read(node, index, sub, signed=signed)
                    if readback != value:
                        raise RuntimeError(f'Ankle node {node}: parameter readback mismatch')
                    return readback
        raise RuntimeError(f'Ankle node {node}: write acknowledgement timeout')

    def read_position_pid(self):
        return [[self.read(node, 0x2004, sub)/100000 for sub in (1, 2, 3)] for node in self.nodes]

    def apply_position_pid(self, gains):
        """Treat all three PID sets as one operation; roll back any partial write."""
        if len(gains) != 3 or any(isinstance(x, bool) or not math.isfinite(x) or not 0 <= x <= 42949.67295 for x in gains):
            raise ValueError('PID values must be finite and representable as unsigned 32-bit / 100000')
        before = [[self.read(node, 0x2004, sub) for sub in (1,2,3)] for node in self.nodes]
        attempted = []
        try:
            for i, node in enumerate(self.nodes):
                for sub, value in enumerate(gains, 1):
                    attempted.append((node, sub, before[i][sub-1]))
                    self.write(node, 0x2004, sub, round(value*100000))
            verified = self.read_position_pid()
            if any(round(actual*100000) != round(expected*100000) for row in verified for actual,expected in zip(row,gains)):
                raise RuntimeError('Final group PID readback differs from requested values')
            return verified
        except Exception as exc:
            failures = []
            for node, sub, value in reversed(attempted):
                try:
                    self.write(node, 0x2004, sub, value)
                except Exception:
                    failures.append(f'{node}:{sub}')
            raise RuntimeError(f'Ankle group PID failed: {exc}; rollback '+
                               ('unverified for '+', '.join(failures) if failures else 'verified')) from exc

    def enable_monitoring(self):
        # Documented PDO1 reporting period, 100 ms per node (not the control rate).
        # This timer is write-only on deployed firmware. Require the matching
        # download acknowledgement; actual PDO arrival establishes monitoring.
        for node in self.nodes:
            self.write(node, 0x1800, 5, 100, size=1, verify_readback=False)

    def validate_targets(self, targets):
        if len(targets) != 3 or len(self.limits) != 3:
            raise ValueError('Three ankle targets and initialized limits are required')
        for node, target, (low, high) in zip(self.nodes, targets, self.limits):
            if not math.isfinite(target) or not low <= target <= high:
                raise ValueError(f'Ankle node {node}: target {target:.2f} mm outside {low:.2f}..{high:.2f}')

    def enable(self):
        # Only a known, latched PDO-stream fault may enter this recovery path.
        # Never clear encoder, voltage, current, travel or unknown faults here.
        for node in self.startup_faults:
            self.send_frame(0x200 + node, bytes.fromhex('00 00 00 00 00 04 00'))
            self.log(f'ANKLE node {node}: disable/re-enable for latched PDO interruption')
        for node in self.nodes:
            self.send_frame(0x200 + node, bytes.fromhex('00 00 00 00 00 07 00'))

    def verify_startup(self):
        # refresh must hold measured positions until every node is fault-free.
        for attempt in range(3):
            faults = {node: self.read(node, 0x2010, 5) for node in self.nodes}
            for node, fault in faults.items():
                if fault not in (0, PDO_INTERRUPTED):
                    raise RuntimeError(f'Ankle node {node}: fault 0x{fault:04X}; startup blocked')
            if not any(faults.values()):
                self.log('ANKLE nodes 1/2/3: fault-free startup confirmed; simulator tracking enabled')
                self.startup_faults.clear()
                return
            if attempt < 2:
                self.listen(0.04)
        failed = ', '.join(f'node {node}=0x{fault:04X}' for node, fault in faults.items() if fault)
        raise RuntimeError(f'Ankle PDO interruption persists after re-enable ({failed}); '
                           'stop, support the platform, and check/restart the affected drive')

    def move(self, targets):
        self.validate_targets(targets)  # Validate the entire group before sending any target.
        for node, target in zip(self.nodes, targets):
            self.send_frame(0x200 + node, position_payload(target))

    def poll(self, index):
        node = self.nodes[index]
        fault = self.read(node, 0x2010, 5)
        if fault:
            raise RuntimeError(f'Ankle node {node}: fault 0x{fault:04X}')
        position = self.read(node, 0x2010, 2, signed=True) / 100
        low, high = self.limits[index]
        if not low - 0.2 <= position <= high + 0.2:
            raise RuntimeError(f'Ankle node {node}: measured position outside drive limits')
        self.positions[index] = position

    def close(self):
        self.refresh = None
        try:
            for node in self.nodes:
                try:
                    self.send_frame(0x200 + node, bytes.fromhex('00 00 00 00 00 04 00'))
                except Exception:
                    pass
            try:
                self.send('C')
            except Exception:
                pass
        finally:
            self.port.close()
