"""Bounded quarantine of discontinuous EV measurements; never fabricate poses."""
import math
import numpy as np
from uav_localization.vio_geometry import rotation


class VioRecovery:
    def __init__(self, stable_time=.5, timeout=2., max_correction=.75,
                 max_angle_deg=20., sample_gap=.1, residual=.03):
        values = (stable_time, timeout, max_correction, max_angle_deg, sample_gap, residual)
        if not all(math.isfinite(v) and v > 0 for v in values) or stable_time >= timeout:
            raise ValueError('VIO recovery limits must be positive; stable_time < timeout')
        self.stable_time, self.timeout = stable_time, timeout
        self.max_correction, self.max_angle = max_correction, math.radians(max_angle_deg)
        self.sample_gap, self.residual = sample_gap, residual
        self.anchor = self.previous = None
        self.started = self.stable_since = None

    @property
    def active(self):
        return self.started is not None

    def begin(self, now, stamp, position, quat, velocity):
        self.started = now
        self.anchor = (stamp, position.copy(), quat.copy(), rotation(quat) @ velocity)
        self.previous = self.stable_since = None

    def expired(self, now):
        return self.active and now - self.started > self.timeout

    def accept(self, now, stamp, pos, quat, velocity):
        """Only admit bounded corrections followed by a stable fresh sequence."""
        if self.expired(now):
            return False
        at, ap, aq, av = self.anchor
        angle = 2 * math.acos(float(np.clip(abs(np.dot(aq, quat)), 0., 1.)))
        world_velocity = rotation(quat) @ velocity
        bounded = (0 < stamp - at <= self.timeout + self.sample_gap and
                   np.linalg.norm(pos - (ap + av * (stamp - at))) <= self.max_correction and
                   angle <= self.max_angle)
        continuous = False
        if bounded and self.previous is not None:
            pt, pp, pv = self.previous
            dt = stamp - pt
            continuous = (0 < dt <= self.sample_gap and
                          np.linalg.norm(pos - pp - pv * dt) <= self.residual)
        if not bounded:
            self.previous = self.stable_since = None
            return False
        if not continuous:
            self.stable_since = stamp
        self.previous = (stamp, pos.copy(), world_velocity)
        return stamp - self.stable_since >= self.stable_time
