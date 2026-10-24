"""Seenpin SLCAN CAN-FD transport matching seenpin_extend_return.py.

One worker owns this transport. refresh is called during SDO waits so all
three position commands keep streaming while telemetry is being read.
"""
import math
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

    def send(self, command):
        raw = (command + '\r').encode('ascii')
        if self.port.write(raw) != len(raw):
            raise RuntimeError('Incomplete Seenpin serial write')

    def send_frame(self, can_id, data):
        self.send(f'b{can_id:03X}{len(data):X}{data.hex().upper()}')

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
                frame = parse_frame(raw.decode('ascii', errors='replace').strip())
                if frame:
                    self.inbox.append(frame)
                    self.inbox = self.inbox[-256:]
            if len(self.buffer) > 16384:
                raise RuntimeError('Invalid Seenpin serial stream')

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
                        raise RuntimeError(f'Ankle node {node}: SDO abort {index:04X}:{sub:02X}')
                    size = {0x4F: 1, 0x4B: 2, 0x47: 3, 0x43: 4}.get(data[0])
                    if size is not None and len(data) >= 4 + size:
                        if attempt > 1:
                            self.log(f'ANKLE node {node}: SDO {index:04X}:{sub:02X} recovered on attempt {attempt}')
                        return int.from_bytes(data[4:4 + size], 'little', signed=signed)
                self.inbox.clear()
            if attempt < SDO_ATTEMPTS:
                self.log(f'ANKLE node {node}: no SDO {index:04X}:{sub:02X} reply; retry {attempt + 1}/{SDO_ATTEMPTS}')
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
            if not low <= 20 <= high:
                raise RuntimeError(f'Ankle node {node}: neutral 20 mm is outside drive limits')
            self.positions.append(position)
            self.limits.append([low, high])
            self.log(f'ANKLE node {node}: {position:.2f} mm, limits {low:.2f}..{high:.2f} mm')

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
