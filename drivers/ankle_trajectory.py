"""Shared three-axis reference filter in millimetres.

A common speed-limited lead preserves actuator ratios for a fixed target.
Two low-pass stages ease starts and reversals. All stages remain inside the
convex bounds of their starting position and accepted targets.
"""
import math


class AnkleTrajectory:
    def __init__(self, positions, max_speed=20.0, response_time=0.10, deadband=0.03):
        if not all(math.isfinite(x) and x > 0 for x in (max_speed, response_time)):
            raise ValueError('Ankle speed and response time must be positive and finite')
        if not math.isfinite(deadband) or deadband < 0:
            raise ValueError('Invalid ankle target deadband')
        self.max_speed, self.response_time, self.deadband = max_speed, response_time, deadband
        self.reset(positions)

    def reset(self, positions):
        if len(positions) != 3 or not all(math.isfinite(x) for x in positions):
            raise ValueError('Three finite ankle positions are required')
        self.position = list(positions)
        self.middle = list(positions)
        self.lead = list(positions)
        self.goal = list(positions)

    def advance(self, targets, elapsed):
        if len(targets) != 3 or not all(math.isfinite(x) for x in targets) or not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError('Invalid ankle target or timestep')
        if max(abs(x - y) for x, y in zip(targets, self.goal)) >= self.deadband:
            self.goal = list(targets)
        # Never make a large catch-up step after a serial or scheduler pause.
        elapsed = min(elapsed, 0.04)
        count = max(1, math.ceil(elapsed / 0.005))
        h = elapsed / count
        alpha = -math.expm1(-h / self.response_time)
        for _ in range(count):
            delta = [x - y for x, y in zip(self.goal, self.lead)]
            distance = max(abs(x) for x in delta)
            fraction = min(1.0, self.max_speed * h / distance) if distance else 0.0
            for i in range(3):
                self.lead[i] += fraction * delta[i]
                self.middle[i] += alpha * (self.lead[i] - self.middle[i])
                self.position[i] += alpha * (self.middle[i] - self.position[i])
        return list(self.position)
