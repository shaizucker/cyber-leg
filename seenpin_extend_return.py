#!/usr/bin/env python3
"""Seenpin three-actuator motion sequence. Requires pyserial. Positive position means extension.
Protocol: supplied Seenpin V2.0 CAN-FD manual; SLCAN: normaldotcom/canable2-fw.
Ctrl+C attempts to disable all drives; this can release a supported load.
"""
import argparse
from datetime import datetime
from pathlib import Path
import sys
import time
import random

READ_NODE = bytes.fromhex('40 01 20 01 00 00 00 00')
FD_LENGTHS = (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64)


def parse_frame(line):
    """Decode data frames, allowing a trailing SLCAN timestamp."""
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
        data = bytes.fromhex(line[start:start + 2 * size])
        if len(data) != size or can_id > (0x1FFFFFFF if width == 8 else 0x7FF):
            return None
        return can_id, data, kind
    except (ValueError, IndexError):
        return None


def decode_sdo(data, key):
    """Accept compact (4 + value bytes) and padded expedited SDO replies."""
    if len(data) < 4 or data[1:4] != key:
        return None
    if data[0] == 0x80:
        if len(data) < 8:
            return None
        return ('abort', int.from_bytes(data[4:8], 'little'))
    size = {0x4F: 1, 0x4B: 2, 0x47: 3, 0x43: 4}.get(data[0])
    if size is None or len(data) < 4 + size:
        return None
    return ('value', data[4:4 + size])


class Link:
    def __init__(self, port, log, on_frame):
        self.port, self.log, self.on_frame = port, log, on_frame
        self.buffer = bytearray()
        self.tick = None

    def send(self, command):
        self.log('TX serial: ' + command)
        self.port.write((command + '\r').encode('ascii'))
        self.port.flush()

    def listen(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self.tick is not None:
                self.tick()
            self.buffer.extend(self.port.read(min(self.port.in_waiting or 1, 4096)))
            while True:
                indices = [i for i in (self.buffer.find(b'\r'), self.buffer.find(b'\a')) if i >= 0]
                if not indices:
                    break
                idx = min(indices)
                raw, sep = bytes(self.buffer[:idx]), self.buffer[idx]
                del self.buffer[:idx + 1]
                if sep == 7:
                    raise RuntimeError('Adapter returned BEL: command rejected or adapter error; scan stopped.')
                line = raw.decode('ascii', errors='replace').strip()
                if line:
                    self.log('RX serial: ' + line)
                    frame = parse_frame(line)
                    if frame:
                        self.on_frame(*frame)
            if len(self.buffer) > 16384:
                raise RuntimeError('Unrecognized serial stream (no SLCAN delimiters).')


def main():
    ap = argparse.ArgumentParser(description='Extend nodes 1/2/3 together by 20/25/27 mm, move independently at random within 10 mm windows for 20 seconds, then return slowly. Fixed CAN FD+BRS 1M/5M. Close CANgaroo first.')
    ap.add_argument('--port', default='COM7')
    ap.add_argument('--seconds', type=float, default=5.0, help='Extension duration, default 5 seconds')
    ap.add_argument('--return-seconds', type=float, default=15.0, help='Slow return duration, default 15 seconds')
    ap.add_argument('--random-seconds', type=float, default=20.0, help='Random phase duration, default 20 seconds; 0 skips it')
    ap.add_argument('--seed', type=int, help='Optional random seed for a repeatable pattern')
    ap.add_argument('--dry-run', action='store_true', help='Read positions, faults and limits; print plan without enabling/moving')
    ap.add_argument('--disable-at-end', action='store_true', help='Compatibility option; drives are now always disabled after returning')
    args = ap.parse_args()
    if not 0.5 <= args.seconds <= 60 or not 0.5 <= args.return_seconds <= 60:
        ap.error('--seconds and --return-seconds must be 0.5..60')
    if not 0 <= args.random_seconds <= 300:
        ap.error('--random-seconds must be 0..300')
    rng = random.Random(args.seed)
    try:
        import serial
        import struct
    except ImportError:
        ap.exit(2, 'Install dependency: py -m pip install pyserial\n')
    nodes = (1, 2, 3)
    path = Path('seenpin_motion_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.txt')
    with path.open('w', encoding='utf-8') as output:
        def log(message):
            line = datetime.now().strftime('%H:%M:%S.%f')[:-3] + ' ' + message
            print(line, flush=True)
            output.write(line + '\n'); output.flush()
        inbox = []
        def on_frame(canid, data, kind):
            if kind.islower() and 0x581 <= canid <= 0x5FF:
                inbox.append((canid, data))
                if len(inbox) > 256:
                    del inbox[:-256]
        with serial.Serial(args.port, 115200, timeout=0.02, write_timeout=1) as port:
            link = Link(port, log, on_frame)
            motion_started = False
            active_targets = {}
            next_refresh = 0.0
            def send_frame(canid, data):
                link.send(f'b{canid:03X}{len(data):X}{data.hex().upper()}')
            def read(node, index, sub, signed=False):
                key = struct.pack('<HB', index, sub)
                for attempt in range(3):
                    inbox.clear()
                    send_frame(0x600 + node, b'\x40' + key + bytes(4))
                    end = time.monotonic() + .4
                    while time.monotonic() < end:
                        link.listen(.02)
                        for canid, data in inbox:
                            if canid != 0x580 + node:
                                continue
                            decoded = decode_sdo(data, key)
                            if decoded is None:
                                continue
                            if decoded[0] == 'abort':
                                raise RuntimeError(f'Node {node}: SDO {index:04X}:{sub:02X} abort 0x{decoded[1]:08X}')
                            return int.from_bytes(decoded[1], 'little', signed=signed)
                raise RuntimeError(f'Node {node}: no reply to {index:04X}:{sub:02X}')
            def fault_check(node):
                fault = read(node, 0x2010, 5)
                if fault:
                    raise RuntimeError(f'Node {node}: fault 0x{fault:04X}')
            def release():
                link.tick = None
                # Best effort only: communication loss can prevent this reaching a drive.
                for node in nodes:
                    try:
                        send_frame(0x200 + node, bytes.fromhex('00 00 00 00 00 04 00'))
                    except Exception as exc:
                        log(f'Could not send disable to node {node}: {exc}')
            def move_together(targets, seconds):
                finish = time.monotonic() + seconds
                for node, target in targets.items():
                    active_targets[node] = (target, finish)
                for node, target in targets.items():
                    log(f'MOVE node {node} to {target/100:.2f} mm over {seconds:.1f} s')
                    send_frame(0x200 + node, struct.pack('<BhHH', 1, target, round(seconds * 1000), 0x000F))
            def refresh_commands():
                nonlocal next_refresh
                now = time.monotonic()
                if now < next_refresh:
                    return
                next_refresh = now + .1
                for node, (target, finish) in tuple(active_targets.items()):
                    # Keep the target fixed, count down movement time, and never use zero
                    # (the manual defines zero as maximum speed).
                    remaining_ms = max(100, min(60000, round((finish - now) * 1000)))
                    send_frame(0x200 + node, struct.pack('<BhHH', 1, target, remaining_ms, 0x000F))
            def wait_targets(targets, seconds):
                # Monitor all drives, including those holding position between moves.
                deadline = time.monotonic() + seconds + 10
                started = time.monotonic()
                settled = 0
                while time.monotonic() < deadline:
                    positions = {}
                    for node in nodes:
                        fault_check(node)
                        positions[node] = read(node, 0x2010, 2, signed=True)
                    close = all(abs(positions[n] - target) <= 20 for n, target in targets.items())
                    settled = settled + 1 if close else 0
                    if settled >= 3 and time.monotonic() - started >= seconds:
                        log('Reached: ' + ', '.join(f'{n}={positions[n]/100:.2f} mm' for n in targets))
                        return
                    link.listen(.08)
                raise RuntimeError('Movement timeout; last positions: ' + str({n:p/100 for n,p in positions.items()}))
            def random_motion(windows):
                stop = time.monotonic() + args.random_seconds
                due = {node: time.monotonic() for node in nodes}
                log(f'Random motion begins: {args.random_seconds:.1f} seconds, independent 10 mm windows.')
                while time.monotonic() < stop:
                    for node in nodes:
                        now = time.monotonic()
                        if now >= stop:
                            break
                        if now >= due[node] and stop - now >= .5:
                            seconds = min(rng.uniform(.8, 1.8), stop - now)
                            target = rng.randint(*windows[node])
                            active_targets[node] = (target, now + seconds)
                            due[node] = now + seconds
                            log(f'RANDOM node {node}: {target/100:.2f} mm over {seconds:.2f} s')
                            send_frame(0x200 + node, struct.pack('<BhHH', 1, target, max(100, round(seconds*1000)), 0x000F))
                        fault_check(node)
                        pos = read(node, 0x2010, 2, signed=True)
                        # Enforce physical software limits with measurement tolerance.
                        low, high = limits[node]
                        if not low - 20 <= pos <= high + 20:
                            raise RuntimeError(f'Node {node}: feedback {pos/100:.2f} mm outside software limits')
                    remaining = stop - time.monotonic()
                    if remaining > 0:
                        link.listen(min(.04, remaining))
                log('Random phase complete; starting slow return.')
            try:
                time.sleep(1.5)
                port.reset_input_buffer()
                for command in ('C', 'V', 'S8', 'Y5', 'M0', 'A0', 'O'):
                    link.send(command); link.listen(.1)
                originals, targets, windows, limits = {}, {}, {}, {}
                for node, delta in zip(nodes, (2000, 2500, 2700)):
                    fault_check(node)
                    original = read(node, 0x2010, 2, signed=True)
                    low = read(node, 0x607D, 1, signed=True)
                    high = read(node, 0x607D, 2, signed=True)
                    target = original + delta  # Positive position means extension, verified in prior run.
                    if not low < high:
                        raise RuntimeError(f'Node {node}: invalid software limits {low/100}..{high/100} mm; not moving')
                    if not low <= original <= high or not low <= target <= high or not -32768 <= target <= 32767:
                        raise RuntimeError(f'Node {node}: start/target outside limits {low/100}..{high/100} mm; not moving')
                    originals[node], targets[node] = original, target
                    limits[node] = (low, high)
                    if args.random_seconds:
                        if high - low < 1000:
                            raise RuntimeError(f'Node {node}: software range is smaller than the requested 10 mm random window')
                        window_low = max(low, min(target - 500, high - 1000))
                        windows[node] = (window_low, window_low + 1000)
                        log(f'RANDOM RANGE node {node}: {window_low/100:.2f}..{(window_low+1000)/100:.2f} mm (10 mm total)')
                    log(f'PLAN node {node}: start={original/100:.2f} mm -> {target/100:.2f} mm (+{delta/100:.2f} mm) -> {original/100:.2f} mm; limits {low/100:.2f}..{high/100:.2f}')
                log('Saved original positions (0.01 mm units): ' + repr(originals))
                if args.dry_run:
                    log('Dry run complete; no enable or movement commands sent.')
                    return 0
                for node in nodes:
                    fault_check(node)
                motion_started = True
                for node in nodes:
                    send_frame(0x200 + node, bytes.fromhex('00 00 00 00 00 07 00'))
                for node in nodes:
                    active_targets[node] = (originals[node], time.monotonic() + .1)
                link.tick = refresh_commands
                link.listen(.2)
                log('Extending all three together from measured starting positions.')
                move_together(targets, args.seconds)
                wait_targets(targets, args.seconds)
                if args.random_seconds:
                    random_motion(windows)
                log('Returning all three slowly to saved starting positions together.')
                move_together(originals, args.return_seconds)
                wait_targets(originals, args.return_seconds)
                release()
                log('Returned to saved starting positions and disabled; holding force released.')
                return 0
            except (Exception, KeyboardInterrupt) as exc:
                log('ABORT: ' + (str(exc) or 'Ctrl+C'))
                if motion_started:
                    log('Sending best-effort disable to all nodes; no automatic return after an error. Holding force may be lost.')
                    release()
                return 2
            finally:
                link.tick = None
                try:
                    link.send('C')
                except Exception:
                    pass
                print('Log:', path.resolve())


if __name__ == '__main__':
    sys.exit(main())
