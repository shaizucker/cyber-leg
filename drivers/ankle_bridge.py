"""Independent Seenpin worker: serial/SDO waits never block the knee loop."""
import math
import threading
import time
from ankle_trajectory import AnkleTrajectory


class AnkleBridge:
    def __init__(self, driver_factory, log=None, max_speed=20.0, response_time=0.10):
        AnkleTrajectory([20, 20, 20], max_speed=max_speed, response_time=response_time)
        self.max_speed, self.response_time = max_speed, response_time
        self.factory = driver_factory
        self.log = log or (lambda message: None)
        self.lock = threading.RLock()
        self.wake = threading.Event()
        self.idle = threading.Event()
        self.idle.set()
        self.stop = threading.Event()
        self.wanted = self.armed = self.arming = False
        self.targets = self.command = None
        self.positions = None
        self.limits = None
        self.running = False
        self.updated = 0.0
        self.error = None
        self.reference_mm = [20.0, 20.0, 20.0]
        self.soft_limits_mm = None
        self.shared_pid = None
        self.expected_current_limits_a = None
        self.telemetry = [{}, {}, {}]
        self.enable_monitoring = False
        self.generation = 0
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def update(self, targets, running):
        if not isinstance(targets, list) or len(targets) != 3 or any(
                isinstance(x, bool) or not isinstance(x, (int, float)) or
                not math.isfinite(x) or not 0 <= x <= 50 for x in targets):
            raise ValueError('Provide three finite ankle extensions in mm, each 0..50')
        with self.lock:
            targets = [origin + value - 20 for origin, value in zip(self.reference_mm, targets)]
            if self.limits and any(not lo <= x <= hi for x, (lo, hi) in zip(targets, self.limits)):
                raise ValueError('Ankle target exceeds measured drive limits')
            if self.soft_limits_mm and any(not lo <= x-origin <= hi for x, origin, (lo, hi)
                    in zip(targets, self.reference_mm, self.soft_limits_mm)):
                raise ValueError('Ankle target exceeds configured limits relative to zero')
            self.targets = list(targets)
            self.running = running
            self.updated = time.monotonic()

    def check_ready(self):
        with self.lock:
            if self.targets is None or time.monotonic() - self.updated > 0.5:
                raise ValueError('Current simulator ankle targets are required; reload the simulator')

    def arm(self):
        self.check_ready()
        with self.lock:
            if self.wanted:
                return
            self.error = None
            self.wanted = self.arming = True
            self.generation += 1
        self.wake.set()

    def disarm(self):
        with self.lock:
            self.wanted = self.armed = self.arming = False
            self.generation += 1
        self.wake.set()

    def status(self):
        with self.lock:
            limits = self.soft_limits_mm
            if limits is None and self.limits:
                limits = [[lo-origin, hi-origin] for (lo,hi),origin in zip(self.limits,self.reference_mm)]
            return dict(enabled=True, armed=self.armed, arming=self.arming,
                        closing=not self.wanted and not self.idle.is_set(),
                        error=self.error, max_speed_mm_s=self.max_speed, response_time_s=self.response_time, nodes=[1, 2, 3],
                        limits_mm=None if limits is None else [[lo+20,hi+20] for lo,hi in limits],
                        hardware_limits_mm=self.limits, reference_mm=list(self.reference_mm),
                        relative_positions_mm=None if self.positions is None else [p-z for p,z in zip(self.positions,self.reference_mm)],
                        shared_pid=self.shared_pid, telemetry=[dict(t) for t in self.telemetry],
                        positions_mm=None if self.positions is None else list(self.positions),
                        command_mm=None if self.command is None else list(self.command))

    def _loop(self):
        while not self.stop.is_set():
            self.wake.wait(0.05)
            self.wake.clear()
            with self.lock:
                wanted, generation = self.wanted, self.generation
                if wanted:
                    self.idle.clear()
            if not wanted:
                continue
            driver = None
            try:
                driver = self.factory()
                driver.initialize()
                if self.enable_monitoring and hasattr(driver, 'enable_monitoring'):
                    driver.enable_monitoring()
                if self.soft_limits_mm and any(not low <= ref+lo < ref+hi <= high
                        for (low,high),ref,(lo,hi) in zip(driver.limits,self.reference_mm,self.soft_limits_mm)):
                    raise RuntimeError('Saved ankle limits exceed measured drive limits')
                if self.soft_limits_mm and any(not lo-0.001 <= pos-ref <= hi+0.001
                        for pos,ref,(lo,hi) in zip(driver.positions,self.reference_mm,self.soft_limits_mm)):
                    raise RuntimeError('Measured actuator position is outside configured signed limits')
                if self.expected_current_limits_a is not None:
                    actual = [driver.read(n,0x2009,3)/100 for n in (1,2,3)]
                    if any(abs(a-b)>0.005 for a,b in zip(actual,self.expected_current_limits_a)):
                        raise RuntimeError('Actuator current limits differ from saved settings; read/apply configuration before arming')
                if self.shared_pid is not None:
                    actual = driver.read_position_pid()
                    if any(any(abs(a-b)>0.000011 for a,b in zip(row,self.shared_pid)) for row in actual):
                        raise RuntimeError('Ankle PID differs from saved group settings; read/apply configuration while disarmed')
                with self.lock:
                    self.limits = [list(pair) for pair in driver.limits]
                    self.positions = list(driver.positions)
                    self.command = list(driver.positions)
                planner = AnkleTrajectory(driver.positions, self.max_speed, self.response_time)
                tracking_ready = False
                next_send = 0.0
                last_send = time.monotonic()

                def refresh():
                    nonlocal next_send, last_send
                    now = time.monotonic()
                    with self.lock:
                        if self.stop.is_set() or not self.wanted or generation != self.generation:
                            raise InterruptedError('Ankle disarmed')
                        if now - self.updated > 0.5:
                            raise RuntimeError('Simulator connection lost; ankle disarmed')
                        running = self.running and tracking_ready
                        targets = list(self.targets if running else self.command)
                    driver.validate_targets(targets)
                    if now >= next_send:
                        if running:
                            command = planner.advance(targets, now - last_send)
                        else:
                            # Stop holds immediately and clears residual filter motion.
                            planner.reset(targets)
                            command = targets
                        command = [round(x, 2) for x in command]
                        driver.move(command)
                        with self.lock:
                            self.command = command
                        last_send = now
                        next_send = now + 0.02

                # Check cancellation, freshness and targets before enabling any node.
                with self.lock:
                    if not self.wanted or generation != self.generation or self.stop.is_set():
                        raise InterruptedError('Ankle arming cancelled')
                    if time.monotonic() - self.updated > 0.5:
                        raise RuntimeError('Simulator updates stopped during ankle arming')
                    driver.validate_targets(self.targets if self.running else self.command)
                driver.enable()
                driver.refresh = refresh
                refresh()
                driver.verify_startup()
                tracking_ready = True
                with self.lock:
                    self.armed, self.arming = True, False
                index = 0
                next_poll = 0.0
                while True:
                    refresh()
                    now = time.monotonic()
                    if now >= next_poll:
                        # One node per 33 ms: each node about 10 Hz, with
                        # fault and position reads. Commands remain at 50 Hz.
                        driver.poll(index)
                        with self.lock:
                            self.positions = list(driver.positions)
                            if hasattr(driver, 'telemetry'):
                                self.telemetry = [dict(driver.telemetry.get(n, {})) for n in (1,2,3)]
                        index = (index + 1) % 3
                        next_poll = time.monotonic() + 1 / 30
                    driver.listen(0.005)
            except InterruptedError:
                pass
            except Exception as exc:
                with self.lock:
                    if generation == self.generation:
                        self.error = str(exc)
                        self.wanted = False
                self.log(f'Ankle stopped: {exc}')
            finally:
                if driver:
                    try:
                        driver.close()
                    except Exception as exc:
                        self.log(f'Ankle close failed: {exc}')
                with self.lock:
                    self.armed = False
                    if generation == self.generation:
                        self.arming = False
                    self.idle.set()

    def close(self):
        self.disarm()
        self.stop.set()
        self.wake.set()
        self.thread.join()
