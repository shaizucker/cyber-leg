"""Continuous two-stage reference filter, with a bounded command speed.

State is retained across targets. This shapes commanded motion; it is not
a feedback controller and cannot guarantee physical overshoot limits.
"""
import math


class KneeTrajectory:
    def __init__(self, position, max_speed=2.0, response_time=0.06):
        if not all(math.isfinite(v) for v in (position, max_speed, response_time)) or max_speed <= 0 or response_time <= 0:
            raise ValueError('Invalid trajectory settings')
        self.position = self.lead = position
        self.max_speed = max_speed
        self.response_time = response_time
        self.velocity = 0.0

    def reset(self, position):
        self.position = self.lead = position
        self.velocity = 0.0

    def advance(self, target, dt):
        if not math.isfinite(target) or not math.isfinite(dt) or dt <= 0:
            raise ValueError('Invalid target or timestep')
        # Substeps reduce dependence on host scheduling jitter.
        count = max(1, math.ceil(dt / 0.005))
        h = dt / count
        alpha = -math.expm1(-h / self.response_time)
        previous = self.position
        for _ in range(count):
            limit = self.max_speed * h
            self.lead += max(-limit, min(limit, (target - self.lead) * alpha))
            self.position += max(-limit, min(limit, (self.lead - self.position) * alpha))
        self.velocity = (self.position - previous) / dt
        return self.position
